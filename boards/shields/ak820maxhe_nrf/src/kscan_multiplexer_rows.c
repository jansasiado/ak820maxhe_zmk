/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Hall-effect kscan driver: analog multiplexer with row-enable GPIOs.
 * Compatible: he,kscan-multiplexer-rows
 *
 * For each row (GPIO enabled one at a time), cycles through all mux addresses
 * and reads every configured ADC channel. Reports INPUT_EV_HE events with
 * INPUT_HE_RC(row, col) where col = group_col_offset + (address - range_min).
 */

#define DT_DRV_COMPAT he_kscan_multiplexer_rows

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(kscan_he_mux_rows, CONFIG_ZMK_LOG_LEVEL);

#define MAX_GROUPS 8
#define MAX_ROWS 8
#define MAX_COLS 16

/* Actuation point (height in um): a key is "pressed" while its computed
 * height is below this value. Set low enough that rest-position ADC noise
 * (~2650-2800um with default calibration) does not trigger false presses.
 * Increase this value once per-key calibration is done. */
#define KSCAN_HE_ACTUATION_POINT_UM 1800

/* Per-group (per mux output / ADC channel) configuration */
struct he_group_cfg {
    struct adc_dt_spec adc;
    uint8_t address_range_min;
    uint8_t address_range_max;
    uint16_t switch_height;
    bool switch_pressed_is_higher;
    uint8_t column_offset;  /* matrix col for addr == range_min */
    uint8_t column_stride;  /* matrix cols per addr increment */
};

struct he_mux_rows_config {
    const struct gpio_dt_spec *address_gpios;
    uint8_t address_gpio_count;
    const struct gpio_dt_spec *row_gpios;
    uint8_t row_count;
    const struct gpio_dt_spec *mux_enable_gpios;
    uint8_t mux_enable_count;
    struct he_group_cfg *groups; /* mutable for col_offset init */
    uint8_t group_count;

    uint32_t address_to_read_delay_us;
    uint32_t row_to_address_delay_us;
    uint32_t wait_period_idle_ms;
    uint32_t wait_period_press_ms;

    int16_t default_cal_min;
    int16_t default_cal_max;
    int16_t default_dz_top_um;
    int16_t default_dz_bottom_um;

    const uint32_t *polyfit; /* float coefficients as hex uint32 */
    uint8_t polyfit_len;
    uint8_t resolution;
    bool calibrate;

    const uint8_t *skip_positions; /* flat (row,col,row,col,...), length = skip_len */
    uint8_t skip_len;              /* total bytes (= 2 * number of pairs) */
};

struct he_mux_rows_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool any_pressed;

    /* Multi-channel ADC read state. Filled in by kscan_he_init. */
    uint32_t channel_mask;             /* OR of all groups' channel_ids */
    uint8_t  group_buf_idx[MAX_GROUPS]; /* position of each group's value in buf */
    int16_t  adc_buf[MAX_GROUPS];      /* one DMA read fills all groups' samples */

    /* Press/release state per position. Updated only on transitions across
     * KSCAN_HE_ACTUATION_POINT_UM. Initialized to false (released) at boot. */
    bool     position_pressed[MAX_ROWS * MAX_COLS];

    kscan_callback_t callback;
};

/* Evaluate polynomial with coefficients stored as IEEE-754 hex uint32 */
static float polyeval(const uint32_t *coeffs, uint8_t len, float x)
{
    union { uint32_t u; float f; } v;
    float r = 0.0f;

    for (uint8_t i = 0; i < len; i++) {
        v.u = coeffs[i];
        r = r * x + v.f;
    }
    return r;
}

static int16_t map_height(int16_t raw, int16_t cal_min, int16_t cal_max,
                          uint16_t switch_height_um, bool pressed_is_higher,
                          const uint32_t *polyfit, uint8_t polyfit_len)
{
    if (cal_min == 0 && cal_max == 0) {
        return switch_height_um / 2; /* uncalibrated — return mid-travel */
    }
    float norm = (float)(raw - cal_min) / (float)(cal_max - cal_min);

    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    if (pressed_is_higher) {
        norm = 1.0f - norm;
    }
    if (polyfit && polyfit_len > 0) {
        norm = polyeval(polyfit, polyfit_len, norm);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
    }
    return (int16_t)(norm * switch_height_um);
}

static bool position_is_skipped(const struct he_mux_rows_config *cfg,
                                uint8_t row, uint8_t col)
{
    for (uint8_t i = 0; i + 1 < cfg->skip_len; i += 2) {
        if (cfg->skip_positions[i] == row &&
            cfg->skip_positions[i + 1] == col) {
            return true;
        }
    }
    return false;
}

static int set_mux_address(const struct he_mux_rows_config *cfg, uint8_t addr)
{
    for (uint8_t i = 0; i < cfg->address_gpio_count; i++) {
        int err = gpio_pin_set_dt(&cfg->address_gpios[i], (addr >> i) & 1);

        if (err) {
            LOG_ERR("Failed to set address GPIO %u: %d", i, err);
            return err;
        }
    }
    return 0;
}

static void scan_row(const struct device *dev, uint8_t row, bool *any_pressed)
{
    const struct he_mux_rows_config *cfg = dev->config;
    struct he_mux_rows_data *data = dev->data;
    uint8_t global_max_addr = 0;

    for (uint8_t g = 0; g < cfg->group_count; g++) {
        if (cfg->groups[g].address_range_max > global_max_addr) {
            global_max_addr = cfg->groups[g].address_range_max;
        }
    }

    for (uint8_t addr = 0; addr <= global_max_addr; addr++) {
        /* Pre-check: if every group at this (row, addr) is skipped, don't
         * even touch the mux GPIOs — keeps the previously-set address held
         * and avoids briefly routing floating mux inputs to the ADC pins. */
        bool any_active = false;

        for (uint8_t g = 0; g < cfg->group_count; g++) {
            const struct he_group_cfg *grp = &cfg->groups[g];

            if (addr < grp->address_range_min || addr > grp->address_range_max) {
                continue;
            }
            uint8_t col = grp->column_offset +
                          (addr - grp->address_range_min) * grp->column_stride;

            if (!position_is_skipped(cfg, row, col)) {
                any_active = true;
                break;
            }
        }
        if (!any_active) {
            continue;
        }

        int err = set_mux_address(cfg, addr);

        if (err) {
            continue;
        }
        if (cfg->address_to_read_delay_us > 0) {
            k_busy_wait(cfg->address_to_read_delay_us);
        }

        /* One ADC read fills the per-group buffer for every active channel.
         * SAADC samples channels back-to-back in one DMA transaction. */
        struct adc_sequence seq = {
            .channels    = data->channel_mask,
            .buffer      = data->adc_buf,
            .buffer_size = sizeof(data->adc_buf),
            .resolution  = cfg->resolution,
        };

        err = adc_read(cfg->groups[0].adc.dev, &seq);
        if (err) {
            LOG_ERR("ADC read error at addr %u: %d", addr, err);
            continue;
        }

        for (uint8_t g = 0; g < cfg->group_count; g++) {
            const struct he_group_cfg *grp = &cfg->groups[g];

            if (addr < grp->address_range_min || addr > grp->address_range_max) {
                continue;
            }

            uint8_t col = grp->column_offset +
                          (addr - grp->address_range_min) * grp->column_stride;

            /* Skip unpopulated sensor sites */
            if (position_is_skipped(cfg, row, col)) {
                continue;
            }

            int16_t buf = data->adc_buf[data->group_buf_idx[g]];

            if (cfg->calibrate) {
                int64_t now = k_uptime_get();

                printk("~%lld,r%u,g%u,a%u: %d\n", now, row, g, addr, buf);
                *any_pressed = true;
                continue;
            }

            int16_t height = map_height(buf,
                                        cfg->default_cal_min,
                                        cfg->default_cal_max,
                                        grp->switch_height,
                                        grp->switch_pressed_is_higher,
                                        cfg->polyfit,
                                        cfg->polyfit_len);

            if (height > cfg->default_dz_bottom_um &&
                height < (int16_t)(grp->switch_height - cfg->default_dz_top_um)) {
                *any_pressed = true;
            }

            /* Press-event log — only fires when key crosses below half-travel */
            if (height < (grp->switch_height / 2)) {
                LOG_INF("PRESS r%u c%u raw=%d height=%dum",
                        row, col, buf, height);
            }

            size_t pos_idx = (size_t)row * MAX_COLS + col;

            /* Direct press/release detection — bypasses the broken upstream
             * adjustable_actuation key_states[0] issue. Fires the kscan event
             * straight at the forwarder on transitions. */
            bool is_pressed = (height < KSCAN_HE_ACTUATION_POINT_UM);

            if (is_pressed != data->position_pressed[pos_idx]) {
                data->position_pressed[pos_idx] = is_pressed;
                LOG_INF("kscan r%u c%u %s (h=%dum) cb=%s",
                        row, col, is_pressed ? "PRESS" : "REL", height,
                        data->callback ? "ok" : "NULL");
                if (data->callback) {
                    data->callback(dev, row, col, is_pressed);
                }
            }
        }
    }
}

static void kscan_he_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct he_mux_rows_data *data =
        CONTAINER_OF(dwork, struct he_mux_rows_data, work);
    const struct device *dev = data->dev;
    const struct he_mux_rows_config *cfg = dev->config;

    data->any_pressed = false;

    int64_t t_start = k_uptime_ticks();

    for (uint8_t row = 0; row < cfg->row_count; row++) {
        gpio_pin_set_dt(&cfg->row_gpios[row], 1);
        if (cfg->row_to_address_delay_us > 0) {
            k_busy_wait(cfg->row_to_address_delay_us);
        }

        scan_row(dev, row, &data->any_pressed);

        gpio_pin_set_dt(&cfg->row_gpios[row], 0);
    }

    /* Once-per-second scan-duration report */
    static int64_t last_report;
    int64_t now = k_uptime_get();

    if (now - last_report >= 1000) {
        int64_t scan_us = k_ticks_to_us_floor64(k_uptime_ticks() - t_start);

        LOG_INF("scan took %lld us", scan_us);
        last_report = now;
    }

    uint32_t next_ms = data->any_pressed ? cfg->wait_period_press_ms
                                         : cfg->wait_period_idle_ms;

    k_work_reschedule(&data->work, K_MSEC(next_ms));
}

static int kscan_he_configure(const struct device *dev, kscan_callback_t cb)
{
    struct he_mux_rows_data *data = dev->data;

    data->callback = cb;
    return 0;
}

static int kscan_he_enable(const struct device *dev)
{
    struct he_mux_rows_data *data = dev->data;

    k_work_reschedule(&data->work, K_NO_WAIT);
    return 0;
}

static int kscan_he_disable(const struct device *dev)
{
    struct he_mux_rows_data *data = dev->data;

    k_work_cancel_delayable(&data->work);
    return 0;
}

static const struct kscan_driver_api kscan_he_api = {
    .config           = kscan_he_configure,
    .enable_callback  = kscan_he_enable,
    .disable_callback = kscan_he_disable,
};

static int kscan_he_init(const struct device *dev)
{
    struct he_mux_rows_data *data = dev->data;
    const struct he_mux_rows_config *cfg = dev->config;

    data->dev = dev;

    /* Configure address GPIOs as outputs */
    for (uint8_t i = 0; i < cfg->address_gpio_count; i++) {
        if (!gpio_is_ready_dt(&cfg->address_gpios[i])) {
            LOG_ERR("Address GPIO %u not ready", i);
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->address_gpios[i], GPIO_OUTPUT_INACTIVE);
    }

    /* Configure row GPIOs as outputs, initially inactive */
    for (uint8_t i = 0; i < cfg->row_count; i++) {
        if (!gpio_is_ready_dt(&cfg->row_gpios[i])) {
            LOG_ERR("Row GPIO %u not ready", i);
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->row_gpios[i], GPIO_OUTPUT_INACTIVE);
    }

    /* Assert mux enable pin(s) once and leave them on */
    for (uint8_t i = 0; i < cfg->mux_enable_count; i++) {
        if (!gpio_is_ready_dt(&cfg->mux_enable_gpios[i])) {
            LOG_ERR("Mux enable GPIO %u not ready", i);
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->mux_enable_gpios[i], GPIO_OUTPUT_ACTIVE);
    }

    /* Set up ADC channels and compute combined channel mask */
    data->channel_mask = 0;
    for (uint8_t g = 0; g < cfg->group_count; g++) {
        struct he_group_cfg *grp = &cfg->groups[g];

        if (!adc_is_ready_dt(&grp->adc)) {
            LOG_ERR("ADC for group %u not ready", g);
            return -ENODEV;
        }
        int err = adc_channel_setup_dt(&grp->adc);

        if (err) {
            LOG_ERR("ADC channel setup error group %u: %d", g, err);
            return err;
        }
        data->channel_mask |= BIT(grp->adc.channel_id);
    }

    /* Compute each group's index inside the multi-channel buffer.
     * SAADC writes samples in ascending channel_id order. */
    for (uint8_t g = 0; g < cfg->group_count; g++) {
        uint8_t channel_id = cfg->groups[g].adc.channel_id;
        uint8_t idx = 0;

        for (uint8_t ch = 0; ch < channel_id; ch++) {
            if (data->channel_mask & BIT(ch)) {
                idx++;
            }
        }
        data->group_buf_idx[g] = idx;
    }

    k_work_init_delayable(&data->work, kscan_he_work_handler);
    LOG_INF("kscan_he init complete: %u rows, %u groups, %u address bits",
            cfg->row_count, cfg->group_count, cfg->address_gpio_count);
    return 0;
}

/* ---------- DT instantiation macros ---------- */

#define ADDR_GPIO(idx, inst) GPIO_DT_SPEC_INST_GET_BY_IDX(inst, address_gpios, idx)
#define ROW_GPIO(idx, inst)  GPIO_DT_SPEC_INST_GET_BY_IDX(inst, row_gpios, idx)
#define MUXEN_GPIO(idx, inst) GPIO_DT_SPEC_INST_GET_BY_IDX(inst, mux_enable_gpios, idx)
#define POLYFIT_VAL(idx, inst) DT_INST_PROP_BY_IDX(inst, polyfit, idx)

/* Emit one uint8_t per element of skip-positions */
#define SKIP_ELEM(node_id, prop, idx) \
    (uint8_t)DT_PROP_BY_IDX(node_id, prop, idx)

#define GROUP_CFG(node_id)                                          \
    {                                                               \
        .adc = ADC_DT_SPEC_GET(node_id),                           \
        .address_range_min = DT_PROP(node_id, address_range_min),  \
        .address_range_max = DT_PROP(node_id, address_range_max),  \
        .switch_height = DT_PROP(node_id, switch_height),          \
        .switch_pressed_is_higher =                                 \
            DT_PROP(node_id, switch_pressed_is_higher),            \
        .column_offset = DT_PROP(node_id, column_offset),          \
        .column_stride = DT_PROP(node_id, column_stride),          \
    },

#define HE_MUX_ROWS_INST(inst)                                                      \
    static const struct gpio_dt_spec addr_gpios_##inst[] = {                        \
        LISTIFY(DT_INST_PROP_LEN(inst, address_gpios), ADDR_GPIO, (,), inst)        \
    };                                                                               \
    static const struct gpio_dt_spec row_gpios_##inst[] = {                         \
        LISTIFY(DT_INST_PROP_LEN(inst, row_gpios), ROW_GPIO, (,), inst)             \
    };                                                                               \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, mux_enable_gpios),                       \
        (static const struct gpio_dt_spec muxen_gpios_##inst[] = {                   \
            LISTIFY(DT_INST_PROP_LEN(inst, mux_enable_gpios), MUXEN_GPIO, (,), inst) \
         };),                                                                        \
        (static const struct gpio_dt_spec muxen_gpios_##inst[] = {};))               \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, skip_positions),                         \
        (static const uint8_t skip_pos_##inst[] = {                                  \
            DT_INST_FOREACH_PROP_ELEM_SEP(inst, skip_positions, SKIP_ELEM, (,))      \
         };),                                                                        \
        (static const uint8_t skip_pos_##inst[] = {};))                              \
    static const uint32_t polyfit_##inst[] = {                                      \
        LISTIFY(DT_INST_PROP_LEN(inst, polyfit), POLYFIT_VAL, (,), inst)            \
    };                                                                               \
    static struct he_group_cfg groups_##inst[] = {                                  \
        DT_INST_FOREACH_CHILD(inst, GROUP_CFG)                                      \
    };                                                                               \
    static struct he_mux_rows_data data_##inst;                                     \
    static const struct he_mux_rows_config config_##inst = {                        \
        .address_gpios       = addr_gpios_##inst,                                   \
        .address_gpio_count  = ARRAY_SIZE(addr_gpios_##inst),                       \
        .row_gpios           = row_gpios_##inst,                                    \
        .row_count           = ARRAY_SIZE(row_gpios_##inst),                        \
        .mux_enable_gpios    = muxen_gpios_##inst,                                  \
        .mux_enable_count    = ARRAY_SIZE(muxen_gpios_##inst),                      \
        .skip_positions      = skip_pos_##inst,                                     \
        .skip_len            = ARRAY_SIZE(skip_pos_##inst),                         \
        .groups              = groups_##inst,                                       \
        .group_count         = ARRAY_SIZE(groups_##inst),                           \
        .address_to_read_delay_us = DT_INST_PROP(inst, address_to_read_delay),      \
        .row_to_address_delay_us  = DT_INST_PROP(inst, row_to_address_delay),       \
        .wait_period_idle_ms = DT_INST_PROP(inst, wait_period_idle),                \
        .wait_period_press_ms = DT_INST_PROP(inst, wait_period_press),              \
        .default_cal_min     = DT_INST_PROP(inst, default_calibration_min),         \
        .default_cal_max     = DT_INST_PROP(inst, default_calibration_max),         \
        .default_dz_top_um   = DT_INST_PROP(inst, default_deadzone_top),            \
        .default_dz_bottom_um = DT_INST_PROP(inst, default_deadzone_bottom),        \
        .polyfit             = polyfit_##inst,                                      \
        .polyfit_len         = ARRAY_SIZE(polyfit_##inst),                          \
        .resolution          = DT_INST_PROP(inst, resolution),                      \
        .calibrate           = DT_INST_PROP(inst, calibrate),                       \
    };                                                                               \
    DEVICE_DT_INST_DEFINE(inst, kscan_he_init, NULL,                                \
                          &data_##inst, &config_##inst,                             \
                          POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,                  \
                          &kscan_he_api);

DT_INST_FOREACH_STATUS_OKAY(HE_MUX_ROWS_INST)
