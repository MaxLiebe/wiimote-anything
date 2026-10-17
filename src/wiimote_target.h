/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_TARGET_H
#define WIIMOTE_TARGET_H

#include "sdkconfig.h"
#include "esp_idf_version.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "WiimoteAnything needs the original ESP32 (the chip with Bluetooth Classic). ESP32-S3, C3, C6, and H2 cannot host a Wiimote."
#endif

/*
 * Backend selection. Define WIIMOTE_BACKEND_BTSTACK or WIIMOTE_BACKEND_BLUEDROID
 * to force one. Otherwise BTstack is used whenever its headers are visible
 * (Bluepad32, BTstack's ESP32 port), and Bluedroid otherwise (stock Arduino).
 */
#if !defined(WIIMOTE_BACKEND_BTSTACK) && !defined(WIIMOTE_BACKEND_BLUEDROID)
#if defined(__has_include)
#if __has_include("btstack.h")
#define WIIMOTE_BACKEND_BTSTACK 1
#endif
#endif
#if !defined(WIIMOTE_BACKEND_BTSTACK)
#define WIIMOTE_BACKEND_BLUEDROID 1
#endif
#endif

#if defined(WIIMOTE_BACKEND_BTSTACK)
#if !defined(CONFIG_BT_ENABLED)
#error "Enable the Bluetooth controller (CONFIG_BT_ENABLED=y, CONFIG_BT_CONTROLLER_ONLY=y) for BTstack."
#endif
#if defined(CONFIG_BTSTACK_INTERNAL_BT) && !defined(CONFIG_BTDM_CTRL_MODE_BTDM)
#error "BTstack's ESP32 port only builds its Bluetooth Classic sources with CONFIG_BTDM_CTRL_MODE_BTDM=y (dual mode). Set it, even if you do not use BLE."
#endif
#else
#if !defined(CONFIG_BT_CLASSIC_ENABLED) || !defined(CONFIG_BT_BLUEDROID_ENABLED)
#error "Enable Bluetooth Classic with Bluedroid. This library does not speak BLE, and the HID device profile is not required."
#endif

/*
 * ESP-IDF 5 can omit the L2CAP and SDP stacks unless those options are set.
 * Arduino-ESP32 defines ARDUINO on every library translation unit and ships a
 * fixed sdkconfig, so a sketch cannot turn the options on. The prebuilt
 * Classic library still exports L2CA_* and SDP_* (A2DP/SPP pull them in).
 */
#if ESP_IDF_VERSION_MAJOR >= 5 && !defined(ARDUINO)
#if !defined(CONFIG_BT_L2CAP_ENABLED) || !defined(CONFIG_BT_SDP_COMMON_ENABLED)
#error "Enable CONFIG_BT_L2CAP_ENABLED and CONFIG_BT_SDP_COMMON_ENABLED. Classic HID device profile is not required."
#endif
#endif
#endif

#endif
