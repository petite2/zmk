/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <dt-bindings/zmk/mouse.h>

/* Trackball move behavior */
/* 0 for mouse movement, 1 for scrolling */
#define MOUSE_MOVE_MODE (0x0)
#define SCROLL_MOVE_MODE (0x1)
#define MOVE_MODE(mode) (((mode) & 0x1) << 18) 
/* 0 for no rotation, 1 for 90 degree, 2 for 180 degree, 3 for 270 degree */
#define ROT_0 (0x0)
#define ROT_90 (0x1)
#define ROT_180 (0x2)
#define ROT_270 (0x3)
#define ROTATION(rot) (((rot) & 0x3) << 16)
/* Resolution of the movement */
#define MOVE_SCALE(scale) ((scale) & 0xFFFF)

#define TB_MOVE(mode,rot,scale) (MOVE_MODE(mode) + ROTATION(rot) + MOVE_SCALE(scale))
#define TB_MODE_DECODE(encoded) ((encoded >> 18) & 0x1)
#define TB_ROTATION_DECODE(encoded) ((encoded >> 16) & 0x3)
#define TB_SCALE_DECODE(encoded) (encoded & 0x0000FFFF) 

#define TB_MOUSE_MODE(rot,scale) TB_MOVE(MOUSE_MOVE_MODE,rot,scale)
#define TB_SCROLL_MODE(rot,scale) TB_MOVE(SCROLL_MOVE_MODE,rot,scale)

#define TB_MOUSE_0(scale) TB_MOUSE_MODE(ROT_0,((scale) & 0xFFFF))
#define TB_MOUSE_90(scale) TB_MOUSE_MODE(ROT_90,((scale) & 0xFFFF))
#define TB_MOUSE_180(scale) TB_MOUSE_MODE(ROT_180,((scale) & 0xFFFF))
#define TB_MOUSE_270(scale) TB_MOUSE_MODE(ROT_270,((scale) & 0xFFFF))

#define TB_SCROLL_0(scale) TB_SCROLL_MODE(ROT_0,((scale) & 0xFF))
#define TB_SCROLL_90(scale) TB_SCROLL_MODE(ROT_90,((scale) & 0xFF))
#define TB_SCROLL_180(scale) TB_SCROLL_MODE(ROT_180,((scale) & 0xFF))
#define TB_SCROLL_270(scale) TB_SCROLL_MODE(ROT_270,((scale) & 0xFF))

#define TB_MOUSE TB_MOUSE_180(6)
#define TB_SCROLL TB_SCROLL_180(1)