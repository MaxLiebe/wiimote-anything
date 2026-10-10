/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_BT_H
#define WIIMOTE_BT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wiimote_bt_start(void);
esp_err_t wiimote_bt_sync(void);
void wiimote_bt_listen_only(void);
esp_err_t wiimote_bt_stop(void);

#ifdef __cplusplus
}
#endif

#endif
