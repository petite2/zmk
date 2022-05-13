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