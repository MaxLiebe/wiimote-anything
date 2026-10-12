# Bluepad32 to Wiimote Adapter

An all-in-one wireless controller adapter for the Nintendo Wii, powered by an original ESP32!

## Overview

This example bridges modern Bluetooth gamepads to a real Nintendo Wii console simultaneously using a single ESP32:
- **Host Role (Bluepad32):** Connects PS5 DualSense, PS4 DualShock 4, Xbox Wireless, Nintendo Switch Pro, Joy-Cons, 8BitDo, and other supported gamepads.
- **Device Role (WiimoteAnything):** Emulates a Nintendo RVL-CNT-01 (Wiimote) with a Classic Controller extension over Bluetooth Classic HID.
- **Shared BTstack:** Both the Host and Device connections run concurrently on the same BTstack engine driving the ESP32's BR/EDR radio.

## Hardware Requirements

- **Original ESP32** (dual-core with Classic Bluetooth BR/EDR, e.g. ESP32 Dev Module, NodeMCU-32S).
- A real Nintendo Wii console (or Wii U / Dolphin Emulator).
- Any Bluetooth gamepad supported by Bluepad32.

## Mapping

- **Left Stick / Right Stick:** Classic Controller Left and Right analog sticks.
- **D-Pad:** Wiimote / Classic D-Pad.
- **A / B / X / Y:** Classic Controller A, B, X, Y buttons.
- **L1 / R1 / L2 / R2:** Classic Controller L, R, ZL, ZR buttons / analog triggers.
- **Start / Select / Home:** Wiimote Plus (+), Minus (-), and Home buttons.
- **Rumble:** Force feedback sent by the Wii console is dynamically routed back to the physical gamepad's rumble motors.

## Building

This example builds as an ESP-IDF project with Bluepad32 and WiimoteAnything:

1. Clone Bluepad32 into your `components/` directory:
   ```bash
   git clone --recursive https://github.com/ricardoquesada/bluepad32.git components/bluepad32
   ```

2. Point `EXTRA_COMPONENT_DIRS` at WiimoteAnything and build:
   ```bash
   idf.py set-target esp32
   idf.py build
   idf.py flash monitor
   ```

3. While the ESP32 is running, press the red **SYNC** button on the front of your Wii console to pair.
4. Put your wireless controller into pairing mode; Bluepad32 will pair and connect to it automatically.
