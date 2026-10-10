/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_HW_H
#define WIIMOTE_HW_H

#include <stdbool.h>

#include "../include/wiimote.h"

#ifdef __cplusplus
extern "C" {
#endif

void wiimote_hw_init(const wiimote_pins_t *pins);
void wiimote_hw_poll(void);
void wiimote_hw_set_leds(uint8_t mask);
void wiimote_hw_set_rumble(bool on);
void wiimote_hw_blink(uint8_t led_count, bool lit);

#ifdef __cplusplus
}
#endif

#endif
