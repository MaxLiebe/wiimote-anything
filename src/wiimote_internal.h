/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_INTERNAL_H
#define WIIMOTE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "wiimote_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

void wiimote_lock(void);
void wiimote_unlock(void);
wiimote_proto_t *wiimote_state(void);
bool wiimote_ready(void);

void wiimote_set_connected(bool up);
bool wiimote_connected_flag(void);
uint8_t wiimote_current_battery(void);
void wiimote_note_leds(uint8_t mask);
void wiimote_note_rumble(bool on);
/** Backend: the discoverable window is closed (timeout, stop, or connection). */
void wiimote_note_sync_ended(void);

#ifdef __cplusplus
}
#endif

#endif
