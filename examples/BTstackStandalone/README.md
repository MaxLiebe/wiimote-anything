# Standalone BTstack Wiimote

WiimoteAnything starts and owns BTstack (no Bluepad32). Buttons on GPIO 4 (A)
and 5 (B) to GND, LED on GPIO 26, rumble on GPIO 25.

BTstack is taken from Bluepad32's checkout if `components/bluepad32` exists
next to the library, from `components/btstack`, or from `$BTSTACK_ROOT`.

```bash
idf.py set-target esp32
idf.py build flash monitor
```

First boot: the sync window opens; press the red SYNC button on the Wii.
Later boots: press A and the ESP32 pages the Wii, the way a bonded remote
does. Press BOOT to sync again.
