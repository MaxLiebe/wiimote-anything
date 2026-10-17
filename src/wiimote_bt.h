/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_BT_H
#define WIIMOTE_BT_H

#include <stdbool.h>

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Transport backend interface. Implemented by wiimote_bt_btstack.inc (primary)
 * or wiimote_bt_bluedroid.inc (Arduino fallback). */

void wiimote_bt_set_external_stack(bool external);
esp_err_t wiimote_bt_start(void);
/** Open the discoverable window. wiimote.c closes it with wiimote_bt_listen_only(). */
esp_err_t wiimote_bt_sync(void);
void wiimote_bt_listen_only(void);
esp_err_t wiimote_bt_stop(void);
/** Page the last synced console (like pressing a button on a bonded remote). */
esp_err_t wiimote_bt_connect(void);
esp_err_t wiimote_bt_disconnect(void);
esp_err_t wiimote_bt_forget_host(void);
bool wiimote_bt_has_host(void);
void wiimote_bt_send_now(void);
bool wiimote_bt_link_is_master(void);
bool wiimote_bt_is_connecting(void);
uint8_t wiimote_bt_last_disconnect_reason(void);

esp_err_t wiimote_btstack_register_services(void);

#ifdef __cplusplus
}
#endif

#endif
