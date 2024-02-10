/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later versжion.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the iььmplied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifdef VIA_ENABLE
/* VIA configuration. */
#    define DYNAMIC_KEYMAP_LAYER_COUNT 3
#endif // VIA_ENABLE

// nkro
#define FORCE_NKRO

#undef LOCKING_SUPPORT_ENABLE
#undef LOCKING_RESYNC_ENABLE

// permissive hold
#define PERMISSIVE_HOLD_PER_KEY
#define HOLD_ON_OTHER_KEY_PRESS_PER_KEY

// tap toggle config
#define TAPPING_TOGGLE 3

// pointer device
#define POINTING_DEVICE_AUTO_MOUSE_ENABLE


#define CHARYBDIS_DRAGSCROLL_REVERSE_Y

#define CHARYBDIS_MINIMUM_DEFAULT_DPI 1200
#define CHARYBDIS_MINIMUM_SNIPING_DPI 600
#define CHARYBDIS_DRAGSCROLL_DPI 300
#define CHARYBDIS_DRAGSCROLL_BUFFER_SIZE 8

#define PMW33XX_LIFTOFF_DISTANCE 0x03

// rgb effects, undef default
#define RGB_MATRIX_KEYPRESSES

#undef RGB_MATRIX_DEFAULT_VAL
#define RGB_MATRIX_DEFAULT_VAL 128

// #define ENABLE_RGB_MATRIX_CUSTOM_DEFAULT_ANIM
// #define ENABLE_RGB_MATRIX_CUSTOM_D_ANIM

#undef RGB_MATRIX_DEFAULT_MODE
#define RGB_MATRIX_DEFAULT_MODE RGB_MATRIX_CUSTOM_DEFAULT_ANIM

/* Charybdis-specific features. */

#ifdef POINTING_DEVICE_ENABLE
// Automatically enable the pointer layer when moving the trackball.  See also:
// - `CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS`
// - `CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD`
// #define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#endif // POINTING_DEVICE_ENABLE
