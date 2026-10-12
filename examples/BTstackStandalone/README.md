# Standalone BTstack Wiimote

Runs WiimoteAnything as a Bluetooth Classic HID peripheral on an ESP32 using BTstack directly (without Bluepad32 or Arduino).

## Features

- Uses BlueKitchen BTstack for L2CAP and SDP services.
- Emulates Nintendo RVL-CNT-01 Wiimote.
- Low RAM usage (~35 KB RAM vs ~150 KB with Bluedroid).
- Clean legacy PIN pairing with the Wii console.

## Building with ESP-IDF

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```
