/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <drivers/behavior.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/trackball_state_changed.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/trackball.h>
#if IS_ENABLED(CONFIG_PIM447)
#include <drivers/sensor.h>
#include <drivers/ext_power.h>
#include <bluetooth/services/bas.h>
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#endif

static void listener_trackball_mouse_move(const struct zmk_trackball_state_changed *ev) {
    LOG_DBG("trackball move scale: 0x%02X, x: 0x%02X, y: 0x%02X", ev->scale, ev->x, ev->y);
    int16_t x = CLAMP(ev->scale * ev->x * ev->x, INT16_MIN, INT16_MAX);
    int16_t y = CLAMP(ev->scale * ev->y * ev->y, INT16_MIN, INT16_MAX);
    // x increases from left to right, y increases from up to down
    if (ev->x < 0) {
        x = -x;
    }
    if (ev->y > 0) {
        y = -y;
    }
    LOG_DBG("trackball move update: x: 0x%02X, y: 0x%02X", x, y);
    zmk_hid_mouse_movement_set(x, y);
    zmk_endpoints_send_mouse_report();
    zmk_hid_mouse_movement_set(0, 0);
}

static void listener_trackball_mouse_scroll(const struct zmk_trackball_state_changed *ev) {
    LOG_DBG("trackball scroll scale: 0x%02X, x: 0x%02X, y: 0x%02X", ev->scale, ev->x, ev->y);
    int8_t x = CLAMP(ev->scale * ev->x * ev->x, INT8_MIN, INT8_MAX);
    int8_t y = CLAMP(ev->scale * ev->y * ev->y, INT8_MIN, INT8_MAX);
    LOG_DBG("trackball scroll update: x: 0x%02X, y: 0x%02X", x, y);
    if (ev->x < 0) {
        x = -x;
    }
    if (ev->y < 0) {
        y = -y;
    }
    zmk_hid_mouse_scroll_set(x, y);
    zmk_endpoints_send_mouse_report();
    zmk_hid_mouse_scroll_set(0, 0);
}

int zmk_trackball_listener(const zmk_event_t *eh) {
    const struct zmk_trackball_state_changed *tb_ev = as_zmk_trackball_state_changed(eh);
    if (tb_ev) {
        switch (tb_ev->mode) {
        case MOUSE_MOVE_MODE:
            listener_trackball_mouse_move(tb_ev);
            break;
        case SCROLL_MOVE_MODE:
            listener_trackball_mouse_scroll(tb_ev);
            break;
        default:
            LOG_DBG("Unsupported trackball move mode %d", tb_ev->mode);
            return -ENOTSUP;
        }
    }

    return 0;
}

ZMK_LISTENER(zmk_trackball_listener, zmk_trackball_listener);
ZMK_SUBSCRIPTION(zmk_trackball_listener, zmk_trackball_state_changed);

#if IS_ENABLED(CONFIG_PIM447)
int32_t get_rgb_from_batt_level(uint8_t batt_level, bool usb_present) {
    int32_t rgb_val = 0;
    if (usb_present) {
        rgb_val = 0x00000f00; // Blue
    } else {
        if (batt_level > 95) {
            rgb_val = 0x000f0000; // Green
        } else if (batt_level > 65) {
            rgb_val = 0x0f0f0000; // Yellow
        } else if (batt_level > 30) {
            rgb_val = 0x0f080000; // Orange
        } else if (batt_level > 10) {
            rgb_val = 0x0f000000; // Red
        } else {
            rgb_val = 0x0; // Off
        }
    }
    return rgb_val;
}
int zmk_trackball_led_batt_level_listener(const zmk_event_t *eh) {
    const struct device *ext_power = device_get_binding("EXT_POWER");
    if (ext_power != NULL) {
        if (ext_power_get(ext_power) <= 0) {
            // external power off, skip led control
            return 0;
        }
    }
    const struct device *trackball = device_get_binding("TRACKBALL");
    bool usb_present = false;
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    usb_present = zmk_usb_is_powered();
#endif
    uint8_t batt_level = bt_bas_get_battery_level();
    struct sensor_value val = {.val1 = get_rgb_from_batt_level(batt_level, usb_present)};
    if (trackball != NULL) {
        sensor_attr_set(trackball, 0, SENSOR_ATTR_PRIV_START + 1, &val);
    }
    return 0;
}

ZMK_LISTENER(zmk_trackball_led_batt_level_listener, zmk_trackball_led_batt_level_listener)
ZMK_SUBSCRIPTION(zmk_trackball_led_batt_level_listener, zmk_battery_state_changed);
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(zmk_trackball_led_batt_level_listener, zmk_usb_conn_state_changed);
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
#endif