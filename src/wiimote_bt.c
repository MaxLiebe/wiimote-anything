/* SPDX-License-Identifier: MIT */
/*
 * Bluetooth Classic transport selection (see wiimote_target.h).
 *
 * BTstack is the primary backend (standalone, or shared with Bluepad32).
 * Bluedroid is the fallback for stock Arduino-ESP32 / ESP-IDF builds.
 */
#include "wiimote_target.h"
#include "wiimote_bt.h"

#if defined(WIIMOTE_BACKEND_BTSTACK)
#include "wiimote_bt_btstack.inc"
#else
#include "wiimote_bt_bluedroid.inc"
#endif
