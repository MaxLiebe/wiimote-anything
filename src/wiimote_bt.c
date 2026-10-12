/* SPDX-License-Identifier: MIT */
/*
 * Bluetooth Classic transport selection.
 *
 * BTstack is the primary, lightweight backend designed for direct integration
 * with Bluepad32, ESP-IDF, and modern Bluetooth projects.
 *
 * If compiling in a stock Arduino-ESP32 environment without BTstack installed,
 * or when WIIMOTE_BACKEND_BLUEDROID is explicitly defined, the library falls back
 * to the Bluedroid backend.
 */
#include "wiimote_bt.h"

#if defined(WIIMOTE_BACKEND_BLUEDROID) || (!defined(WIIMOTE_BACKEND_BTSTACK) && !__has_include("btstack.h"))
#include "wiimote_bt_bluedroid.inc"
#else
#include "wiimote_bt_btstack.inc"
#endif
