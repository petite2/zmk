/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_trackball_key_press

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>

#include <drivers/sensor.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/mouse_button_state_changed.h>
#include <zmk/events/trackball_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

enum orientation {
    EAST,
    NORTH,
    SOUTH,
    WEST
};

struct behavior_sensor_trackball_key_press_config {
    int scan_interval_ms;
    enum orientation orientation;
};

static int behavior_sensor_trackball_key_press_init(const struct device *dev) { return 0; };

static int on_sensor_binding_triggered(struct zmk_behavior_binding *binding,
                                       const struct device *sensor, int64_t timestamp) {
    struct sensor_value x_value;
    struct sensor_value y_value;
    struct sensor_value click;
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_sensor_trackball_key_press_config * config = dev->config;
    int err;
    
    LOG_DBG("trackball setup 0x%02X click 0x%02X", binding->param1, binding->param2);

    err = sensor_channel_get(sensor, SENSOR_CHAN_POS_DX, &x_value);

    if (err) {
        LOG_WRN("Failed to ge sensor x position value: %d", err);
        return err;
    }

    err = sensor_channel_get(sensor, SENSOR_CHAN_POS_DY, &y_value);

    if (err) {
        LOG_WRN("Failed to ge sensor y position value: %d", err);
        return err;
    }

    err = sensor_channel_get(sensor, SENSOR_CHAN_POS_DZ, &click);

    if (err) {
        LOG_WRN("Failed to ge sensor click value: %d", err);
        return err;
    }

    int rc = 0;
    if ((click.val1 == 0b01) || (click.val1 == 0b10)) {
        LOG_DBG("SEND %d", binding->param2);
        rc = ZMK_EVENT_RAISE(zmk_mouse_button_state_changed_from_encoded(binding->param2, (click.val1 == 0b01), timestamp));
        // k_msleep(5);
    } else {
        int32_t x = x_value.val1, y = y_value.val1;
        switch(config->orientation) {
            case EAST:
                x = y_value.val1;
                y = -x_value.val1;
                break;
            case NORTH:
                x = -x_value.val1;
                y = -y_value.val1;
                break;
            case WEST:
                x = -y_value.val1;
                y = x_value.val1;
                break;
            case SOUTH:
            default:
                break;
        }
        rc = ZMK_EVENT_RAISE(zmk_trackball_state_changed_from_encoded(binding->param1, x_value.val1, y_value.val1, timestamp));
        k_msleep(config->scan_interval_ms);
    }

    return rc;
}

static const struct behavior_driver_api behavior_sensor_trackball_key_press_driver_api = {
    .sensor_binding_triggered = on_sensor_binding_triggered};

#define KP_INST(n)                                                                                                              \
    static struct behavior_sensor_trackball_key_press_config behavior_sensor_trackball_key_press_config_##n = {                 \
        .scan_interval_ms = DT_INST_PROP(n, scan_interval_ms),                                                                  \
        .orientation = DT_ENUM_IDX(DT_DRV_INST(n), orientation),                                                                \
    };                                                                                                                          \
    DEVICE_DT_INST_DEFINE(n, behavior_sensor_trackball_key_press_init, device_pm_control_nop, NULL,                             \
                          &behavior_sensor_trackball_key_press_config_##n, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                          &behavior_sensor_trackball_key_press_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
