/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr.h>
#include <dt-bindings/zmk/trackball.h>

#if IS_ENABLED(CONFIG_PIM447)
int32_t get_rgb_from_batt_level(uint8_t batt_level, bool usb_present);
#endif