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
 * With a fallback handler set, an incoming connection is the console when it
 * comes from the console synced earlier, when its class of device is the
 * Wii's (0x000448), or, while the sync window is open, when it is any
 * non-peripheral host (Dolphin, Wii U). Everything else goes to the handler.
 * Without a fallback handler every incoming host is treated as a console.
 *
 * The library leaves the host stack's links alone: incoming host-stack
 * channels are offered once the link meets the host stack's GAP security
 * level (as its own PSM services would require), and radio tuning of a
 * gamepad link is left to wiimote_btstack_tune_link().
 */
#ifndef WIIMOTE_BTSTACK_H
#define WIIMOTE_BTSTACK_H

#include "btstack_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Set before wiimote_init(). Pass NULL for a standalone remote. */
void wiimote_btstack_set_fallback_handler(btstack_packet_handler_t handler);

/**
 * Tune a host-stack link (a gamepad) for sharing the radio with the console:
 * ESP32 becomes master, sniff mode is turned off, and a short poll interval is
 * requested. Call it on the BTstack thread once the device is fully set up
 * (Bluepad32: on_device_ready). Doing this while the host stack is still
 * pairing makes some gamepads give up, so the library never does it on its own.
 * BR/EDR links only; LE handles and the console link are ignored.
 */
void wiimote_btstack_tune_link(hci_con_handle_t handle, const bd_addr_t addr);

#ifdef __cplusplus
}
#endif

#endif
