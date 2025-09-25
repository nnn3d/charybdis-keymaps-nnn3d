/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#include <math.h>
#include <stdint.h>
#include "usb_main.h"

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#    include "timer.h"
#endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

enum charybdis_keymap_layers {
    LAYER_BASE = 0,
    LAYER_LOWER,
    LAYER_RAISE,
    LAYER_POINTER,
    LAYER_D,
    LAYER_DM,
    LAYER_D2,
    LAYER_RAISE_NUM
};

// Tap Dance helpers
typedef struct {
    uint16_t tap;
    uint16_t hold;
    uint16_t held;
} tap_dance_tap_hold_t;

void tap_dance_tap_hold_each_tap(tap_dance_state_t *state, void *user_data) {
}

void tap_dance_tap_hold_finished(tap_dance_state_t *state, void *user_data) {
    tap_dance_tap_hold_t *tap_hold = (tap_dance_tap_hold_t *)user_data;

    if (state->pressed) {
        // if (state->count == 1 && !state->interrupted) {
        if (state->count == 1 && !state->interrupted) {
            register_code16(tap_hold->hold);
            tap_hold->held = tap_hold->hold;
        } else {
            register_code16(tap_hold->tap);
            tap_hold->held = tap_hold->tap;
        }
    }
}

void tap_dance_tap_hold_reset(tap_dance_state_t *state, void *user_data) {
    tap_dance_tap_hold_t *tap_hold = (tap_dance_tap_hold_t *)user_data;

    if (tap_hold->held) {
        unregister_code16(tap_hold->held);
        tap_hold->held = 0;
    }
}

#define ACTION_TAP_DANCE_TAP_HOLD(tap, hold) \
    { .fn = {NULL, tap_dance_tap_hold_finished, tap_dance_tap_hold_reset}, .user_data = (void *)&((tap_dance_tap_hold_t){tap, hold, 0}), }


// Tap Dance for GUI
void dance_sft_on_each_tap(tap_dance_state_t *state, void *user_data) {
    if (state->count == 2) {
        caps_word_on();
        state->finished = true;
    }
}

void dance_sft_finished(tap_dance_state_t *state, void *user_data) {
    register_code16(KC_LGUI);
}

void dance_sft_reset(tap_dance_state_t *state, void *user_data) {
    if (state->count == 1) {
        wait_ms(TAP_CODE_DELAY);
        unregister_code16(KC_LGUI);
    }
}


// Tap Dance declarations
enum {
    TD_ALT,
    TD_GUI,
    TD_DQT,
    TD_MPV,
    TD_DSL,
};

// Tap Dance definitions
tap_dance_action_t tap_dance_actions[] = {
    [TD_ALT] = ACTION_TAP_DANCE_DOUBLE(KC_LALT, LGUI(KC_SPC)),
    [TD_GUI] = ACTION_TAP_DANCE_FN_ADVANCED(dance_sft_on_each_tap, dance_sft_finished, dance_sft_reset),
    [TD_DQT] = ACTION_TAP_DANCE_TAP_HOLD(KC_DQUO, KC_RSFT),
    [TD_MPV] = ACTION_TAP_DANCE_TAP_HOLD(KC_MPRV, KC_LCTL),
    // do nothing on hold since has additional logic in pre_process_record_user
    [TD_DSL] = ACTION_TAP_DANCE_TAP_HOLD(DRAGSCROLL_MODE_TOGGLE, XXXXXXX),
};

#define KCT_ALT TD(TD_ALT)
#define KCT_GUI TD(TD_GUI)
#define KCT_DQT TD(TD_DQT)
#define KCT_PRV TD(TD_MPV)
#define KCT_DSL TD(TD_DSL)

void process_tap_hold_keys(uint16_t keycode, keyrecord_t *record) {
    tap_dance_action_t *action;

    switch (keycode) {
        // list all tap dance keycodes with tap-hold configurations
        case KCT_DQT:
        case KCT_DSL:
        case KCT_PRV:
            action = &tap_dance_actions[TD_INDEX(keycode)];
            if (!record->event.pressed && action->state.count && !action->state.finished) {
                tap_dance_tap_hold_t *tap_hold = (tap_dance_tap_hold_t *)action->user_data;
                if (tap_hold->tap == DRAGSCROLL_MODE_TOGGLE) {
                    charybdis_set_pointer_dragscroll_enabled(!charybdis_get_pointer_dragscroll_enabled());
                } else {
                    tap_code16(tap_hold->tap);
                }
            }
            break;
    }
}

enum custom_keycodes {
    MOD_BN1 = SAFE_RANGE,
};

// base
#define TT_LOWER TT(LAYER_LOWER)
#define TT_RAISE TT(LAYER_RAISE)
#define RZ_A LT(LAYER_RAISE, KC_A)
#define RZN_LBRC LT(LAYER_RAISE_NUM, KC_LBRC)
#define LWR_SCLN LT(LAYER_LOWER, KC_SCLN)
#define MOD_SLH LT(LAYER_POINTER, KC_SLSH)
#define MOD_Z    LCTL_T(KC_Z)
#define MOD_QUOT RSFT_T(KC_QUOT)
#define MOD_RBRC RCTL_T(KC_RBRC)
#define MOD_BSLS RCTL_T(KC_BSLS)

// d layer
// #define MLT_SPC ALT_T(KC_SPC)

#ifndef POINTING_DEVICE_ENABLE
#    define DRGSCRL KC_NO
#    define DPI_MOD KC_NO
#    define S_D_MOD KC_NO
#    define SNIPING KC_NO
#endif // !POINTING_DEVICE_ENABLE

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT(
  // ╭───────────────────────────────────────────────-──────╮ ╭──────────────────────────────────────────────────────╮
        KC_ESC,    KC_1,    KC_2,    KC_3,    KC_4,    KC_5,       KC_6,    KC_7,    KC_8,    KC_9,    KC_0, KC_MINS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
        KC_TAB,    KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,       KC_Y,    KC_U,    KC_I,    KC_O,    KC_P, RZN_LBRC,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       KC_LSFT,    RZ_A,    KC_S,    KC_D,    KC_F,    KC_G,       KC_H,    KC_J,    KC_K,    KC_L,LWR_SCLN, MOD_QUOT,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       KC_LCTL,    MOD_Z,    KC_X,    KC_C,    KC_V,    KC_B,       KC_N,    KC_M, KC_COMM,  KC_DOT,MOD_SLH, MOD_RBRC,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  KCT_ALT,  KC_SPC,TT_LOWER,   KC_BSPC, RALT_T(KC_ENT),
                                           KC_ESC, KCT_GUI,   KC_DEL
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_LOWER] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
        KC_F12,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,      KC_F6,   KC_F7,  KC_F8,   KC_F9,   KC_F10,  KC_F11,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
        KC_GRV,  KC_INS,   KC_NO, KC_WBAK, KC_WFWD, KC_PSCR,    XXXXXXX, KC_HOME,  KC_END, XXXXXXX,  EE_CLR, QK_BOOT,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, DRGSCRL, KC_VOLD, KC_VOLU, MS_BTN1, XXXXXXX,    KC_LEFT, KC_DOWN,   KC_UP, KC_RGHT, _______, KC_RSFT,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, KCT_PRV, KC_MNXT, KC_MPLY, KC_MUTE, XXXXXXX,    XXXXXXX, MS_BTN1, MS_BTN2, MS_BTN3, KCT_DSL, KC_RCTL,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_RAISE] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      EE_CLR,   KC_EXLM,   KC_AT, KC_HASH,  KC_DLR, KC_PERC,    KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_UNDS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
   TO(LAYER_D), KC_PSLS,    KC_7,    KC_8,    KC_9, KC_PAST,   KC_GRV,  KC_LCBR, KC_RCBR, KC_MINS, KC_PLUS,  KC_EQL,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______,    KC_4,    KC_5,    KC_6, KC_PMNS,    KC_LBRC, KC_LPRN, KC_RPRN, KC_RBRC, KC_COLN, KCT_DQT,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______,  KC_NUM,    KC_1,    KC_2,    KC_3, KC_PPLS,    KC_AMPR, KC_PIPE,   KC_LT,   KC_GT, KC_QUES, MOD_BSLS,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                     KC_0, _______,  KC_EQL,    KC_UNDS, S(KC_ENT),
                                            KC_DOT, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_RAISE_NUM] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      EE_CLR,   KC_EXLM,   KC_AT, KC_HASH,  KC_DLR, KC_PERC,    KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_UNDS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
   TO(LAYER_D), KC_PSLS,    KC_P7,  KC_P8,   KC_P9, KC_PAST,   KC_GRV,  KC_LCBR, KC_RCBR, KC_MINS, KC_PLUS,  _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______,   KC_P0,    KC_P4,  KC_P5,   KC_P6, KC_PMNS,    KC_LBRC, KC_LPRN, KC_RPRN, KC_RBRC, KC_COLN, KCT_DQT,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______,  KC_NUM,    KC_P1,  KC_P2,   KC_P3, KC_PPLS,    KC_AMPR, KC_PIPE,   KC_LT,   KC_GT, KC_QUES, MOD_BSLS,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                    KC_P0, _______,  KC_EQL,    KC_UNDS, S(KC_ENT),
                                            KC_DOT, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_POINTER] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, MOD_BN1, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, MS_BTN1, MS_BTN2, MS_BTN3, KCT_DSL, _______,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_D] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______,    KC_A, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______,    KC_Z, _______, _______, _______, _______,    _______, _______, _______, _______, _______,TO(LAYER_BASE),
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                             KC_LALT, _______, TT(LAYER_D2),    _______, _______,
                                      TG(LAYER_DM), _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯"
  ),

  [LAYER_DM] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, MS_BTN1, MS_BTN2, MS_BTN3, KCT_DSL, _______,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_D2] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
        KC_F12,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,      KC_F6,   KC_F7,  KC_F8,   KC_F9,   KC_F10,  KC_F11,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),
};
// clang-format on

bool is_mouse_record_user(uint16_t keycode, keyrecord_t* record) {
    switch(keycode) {
        case KCT_DSL:
        case MOD_BN1:
            return true;
        default:
            return false;
    }
}

void pointing_device_init_user(void) {
    set_auto_mouse_layer(LAYER_POINTER);
    set_auto_mouse_enable(true);
}

static void process_scroll(report_mouse_t* mouse_report) {
    static int8_t history[10] = { 0 };
    static uint8_t index = 0;
    static bool enabled = false;
    static int8_t sum = 0;

    const int8_t treshold = 4;

    if (enabled != charybdis_get_pointer_dragscroll_enabled()) {
        if (enabled) {
            for (int i = 0; i < sizeof(history); i++)
                history[i] = 0;
            sum = 0;
        }

        enabled = !enabled;
    }

    if (enabled) {
        history[index] = mouse_report->v;
        index++;
        if (index >= sizeof(history)) {
            index = 0;
        }

        if (mouse_report->v != 0 || abs(sum) >= treshold) {
            sum = 0;

            for (int i = 0; i < sizeof(history); i++)
                sum += history[i];

            if (abs(sum) >= treshold) {
                mouse_report->v += sum / treshold;
            }
        }
    }
}

void process_mouse(report_mouse_t* mouse_report) {
    int16_t x = mouse_report->x;
    int16_t y = mouse_report->y;

    if (x != 0 || y != 0) {
        uint16_t magnitude = sqrt(x * x + y * y);
        magnitude = magnitude + pow(magnitude, 2) / 25;
        uint16_t sum = abs(x) + abs(y);

        int16_t resX = magnitude * x / sum;
        int16_t resY = magnitude * y / sum;

        mouse_report->x = (int8_t)(MAX(MIN(resX, INT8_MAX), INT8_MIN));
        mouse_report->y = (int8_t)(MAX(MIN(resY, INT8_MAX), INT8_MIN));
    }
}

report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    process_scroll(&mouse_report);
    process_mouse(&mouse_report);

    return mouse_report;
}

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MOD_Z:
        case MOD_QUOT:
        case MOD_RBRC:
        case MOD_BSLS:
        case KCT_DQT:
        case RZ_A:
        case LWR_SCLN:
        case ALT_T(KC_SPC):
            // Immediately select the hold action when another key is tapped.
            return true;
        default:
            // Do not select the hold action when another key is tapped.
            return false;
    }
}

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MOD_QUOT:
            // Immediately select the hold action when another key is pressed.
            return true;
        default:
            // Do not select the hold action when another key is pressed.
            return false;
    }
}

bool pre_process_record_user(uint16_t keycode, keyrecord_t *record) {
  static bool prev_dragscroll = false;
  static bool dragscroll_pressed = false;

  switch (keycode) {
    // enable drag scroll immediately on key press
    case RZ_A:
    case MOD_SLH:
    case KCT_DSL:
        if (record->event.pressed) {
            if (!dragscroll_pressed) {
                // disable dragscroll and ignore input when click slsh with enabled dragscroll
                if (keycode == MOD_SLH && charybdis_get_pointer_dragscroll_enabled()) {
                    prev_dragscroll = false;
                    return false;
                } else {
                    prev_dragscroll = charybdis_get_pointer_dragscroll_enabled();
                }
                charybdis_set_pointer_dragscroll_enabled(true);

                dragscroll_pressed = true;
            }
        } else {
            charybdis_set_pointer_dragscroll_enabled(prev_dragscroll);
            dragscroll_pressed = false;
        }
        return true;
    case TT_LOWER:
        charybdis_set_pointer_sniping_enabled(record->event.pressed);
        return true;
    // case MLT_SPC:
    //     if (record->event.pressed) {
    //         layer_on(LAYER_D2);
    //     } else {
    //         layer_off(LAYER_D2);
    //     }
    //     return true;
    default:
        return true;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MOD_BN1:
            if (get_mods()) {
                if (record->event.pressed) {
                    auto_mouse_reset_trigger(true);
                    register_code16(KC_F);
                } else {
                    unregister_code16(KC_F);
                }
            } else {
                if (record->event.pressed) {
                    register_code16(MS_BTN1);
                } else {
                    unregister_code16(MS_BTN1);
                }
            }
            return false;
        default:
            process_tap_hold_keys(keycode, record);
            return true;
    }
}


layer_state_t layer_state_set_user(layer_state_t state) {
    // auto mouse layer
    switch(get_highest_layer(remove_auto_mouse_layer(state, true))) {
        case LAYER_BASE:
        case LAYER_D:
            set_auto_mouse_enable(true);
            break;
        default:
            // remove_auto_mouse_target must be called to adjust state *before* setting enable
            state = remove_auto_mouse_layer(state, false);
            set_auto_mouse_enable(false);
            break;
    }

    // d layer anim
    if (layer_state_cmp(state, LAYER_DM)) {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_DM_ANIM);
    } else if (layer_state_cmp(state, LAYER_D)) {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_D_ANIM);
    } else {
        rgb_matrix_mode(RGB_MATRIX_DEFAULT_MODE);
    }

    return state;
}

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);
#endif

// bool shutdown_user(bool jump_to_bootloader) {
// #ifdef RGBLIGHT_ENABLE
//     rgblight_enable_noeeprom();
//     rgblight_mode_noeeprom(1);
//     rgblight_setrgb(RGB_RED);
// #endif // RGBLIGHT_ENABLE
// #ifdef RGB_MATRIX_ENABLE
//     rgb_matrix_set_color_all(RGB_RED);
//     rgb_matrix_update_pwm_buffers();
// #endif // RGB_MATRIX_ENABLE
//     return true;
// }

// fix keayboard shut down on computer wake up with hub
void suspend_wakeup_init_user(void) {

    wait_ms(500);

    usbDisconnectBus(&USB_DRIVER);
    usbStop(&USB_DRIVER);
    
    wait_ms(1000);
    
    restart_usb_driver(&USB_DRIVER);

    wait_ms(500);

    clear_keyboard();
}
