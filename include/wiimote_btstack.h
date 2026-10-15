/* SPDX-License-Identifier: MIT */
/**
 * BTstack-specific integration for WiimoteAnything.
 *
 * Only include this when building against BTstack.
 *
 * When WiimoteAnything shares BTstack with an HID host stack such as
 * Bluepad32, both need L2CAP PSM 0x11 / 0x13: the Wii connects to us there,
 * and bonded gamepads reconnect to us there as well. WiimoteAnything owns those
 * two PSMs and passes every channel that does not belong to the console to the
 * fallback handler:
 *
 *     btstack_init();
 *     wiimote_btstack_set_fallback_handler(uni_bt_packet_handler);
 *     wiimote_init(&cfg);            // cfg.external_stack = true
 *     uni_platform_set_custom(...);
 *     uni_init(0, NULL);
 *     btstack_run_loop_execute();
 *
 * With a fallback handler set, an incoming connection is the console when its
 * class of device is not "peripheral" (the Wii is 0x000448), or when it comes
 * from the console synced earlier. Everything else goes to the handler.
 * Without a fallback handler every incoming host is treated as a console.
 */
#ifndef WIIMOTE_BTSTACK_H
#define WIIMOTE_BTSTACK_H

#include "btstack_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Set before wiimote_init(). Pass NULL for a standalone remote. */
void wiimote_btstack_set_fallback_handler(btstack_packet_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif
