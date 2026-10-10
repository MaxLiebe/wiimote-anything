/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_IDS_H
#define WIIMOTE_IDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Which extension the ESP32 presents on the virtual expansion bus. */
typedef enum {
    WIIMOTE_EXT_NONE = 0,
    WIIMOTE_EXT_NUNCHUK,
    WIIMOTE_EXT_CLASSIC,
    WIIMOTE_EXT_CLASSIC_PRO,
    WIIMOTE_EXT_MOTION_PLUS,
    WIIMOTE_EXT_MOTION_PLUS_NUNCHUK
} wiimote_extension_t;

/**
 * Buttons the core remote and the shipped extensions can report.
 * Classic buttons are independent of the core buttons that share a name.
 */
typedef enum {
    WIIMOTE_BUTTON_LEFT = 0,
    WIIMOTE_BUTTON_RIGHT,
    WIIMOTE_BUTTON_DOWN,
    WIIMOTE_BUTTON_UP,
    WIIMOTE_BUTTON_PLUS,
    WIIMOTE_BUTTON_TWO,
    WIIMOTE_BUTTON_ONE,
    WIIMOTE_BUTTON_B,
    WIIMOTE_BUTTON_A,
    WIIMOTE_BUTTON_MINUS,
    WIIMOTE_BUTTON_HOME,
    WIIMOTE_BUTTON_NUNCHUK_C,
    WIIMOTE_BUTTON_NUNCHUK_Z,
    WIIMOTE_BUTTON_CLASSIC_A,
    WIIMOTE_BUTTON_CLASSIC_B,
    WIIMOTE_BUTTON_CLASSIC_X,
    WIIMOTE_BUTTON_CLASSIC_Y,
    WIIMOTE_BUTTON_CLASSIC_L,
    WIIMOTE_BUTTON_CLASSIC_R,
    WIIMOTE_BUTTON_CLASSIC_ZL,
    WIIMOTE_BUTTON_CLASSIC_ZR,
    WIIMOTE_BUTTON_CLASSIC_PLUS,
    WIIMOTE_BUTTON_CLASSIC_MINUS,
    WIIMOTE_BUTTON_CLASSIC_HOME,
    WIIMOTE_BUTTON_CLASSIC_UP,
    WIIMOTE_BUTTON_CLASSIC_DOWN,
    WIIMOTE_BUTTON_CLASSIC_LEFT,
    WIIMOTE_BUTTON_CLASSIC_RIGHT,
    WIIMOTE_BUTTON_COUNT
} wiimote_button_t;

/** Analog controls. Sticks are bipolar; triggers are unipolar. */
typedef enum {
    WIIMOTE_AXIS_NUNCHUK_X = 0,
    WIIMOTE_AXIS_NUNCHUK_Y,
    WIIMOTE_AXIS_CLASSIC_LX,
    WIIMOTE_AXIS_CLASSIC_LY,
    WIIMOTE_AXIS_CLASSIC_RX,
    WIIMOTE_AXIS_CLASSIC_RY,
    WIIMOTE_AXIS_CLASSIC_LT,
    WIIMOTE_AXIS_CLASSIC_RT,
    WIIMOTE_AXIS_COUNT
} wiimote_axis_t;

#ifdef __cplusplus
}
#endif

#endif
