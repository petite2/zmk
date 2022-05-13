
/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr.h>
#include <zmk/event_manager.h>
#include <zmk/trackball.h>

struct zmk_trackball_state_changed {
    uint8_t mode;
    int16_t scale;
    int16_t x;
    int16_t y;
    int64_t timestamp;
};

ZMK_EVENT_DECLARE(zmk_trackball_state_changed);

static inline struct zmk_trackball_state_changed_event *
zmk_trackball_state_changed_from_encoded(uint32_t encoded, int32_t x, int32_t y,
                                         int64_t timestamp) {
    // uint8_t rotation = TB_ROTATION_DECODE(encoded);
    int16_t rotated_x = (x & 0xFFFF), rotated_y = (y & 0xFFFF);
    // switch(rotation) {
    //     case ROT_90:
    //         rotated_x = -rotated_y;
    //         rotated_y = rotated_x;
    //         break;
    //     case ROT_180:
    //         rotated_x = -rotated_x;
    //         rotated_y = -rotated_y;
    //         break;
    //     case ROT_270:
    //         rotated_x = rotated_y;
    //         rotated_y = -rotated_x;
    //         break;
    //     default:
    //         break;
    // }

    return new_zmk_trackball_state_changed(
        (struct zmk_trackball_state_changed){.mode = TB_MODE_DECODE(encoded),
                                             .scale = TB_SCALE_DECODE(encoded),
                                             .x = rotated_x,
                                             .y = rotated_y,
                                             .timestamp = timestamp});
}