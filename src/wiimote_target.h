/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_TARGET_H
#define WIIMOTE_TARGET_H

#include "sdkconfig.h"
#include "esp_idf_version.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "WiimoteAnything needs the original ESP32 (the chip with Bluetooth Classic). ESP32-S3, C3, C6, and H2 cannot host a Wiimote."
#endif

#if !defined(CONFIG_BT_CLASSIC_ENABLED) || !defined(CONFIG_BT_BLUEDROID_ENABLED)
#error "Enable Bluetooth Classic with Bluedroid. This library does not speak BLE, and the HID device profile is not required."
#endif

/* IDF 4.4 builds L2CAP and SDP with Classic. IDF 5 leaves both off unless set. */
#if ESP_IDF_VERSION_MAJOR >= 5
#if !defined(CONFIG_BT_L2CAP_ENABLED) || !defined(CONFIG_BT_SDP_COMMON_ENABLED)
#error "Enable CONFIG_BT_L2CAP_ENABLED and CONFIG_BT_SDP_COMMON_ENABLED. Classic HID device profile is not required."
#endif
#endif

#endif
