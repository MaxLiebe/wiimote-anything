# WiimoteAnything

The ESP32 is the Wiimote. A real Wii is the host.

This library speaks Bluetooth Classic HID the way a Nintendo RVL-CNT-01 does: device name `Nintendo RVL-CNT-01`, class of device `0x002504`, L2CAP PSM `0x11` (control) and `0x13` (interrupt), report headers `0xA2` / `0xA1`, and the extension bus games actually initialize. Pairing during sync uses the Wii's BD_ADDR reversed as the PIN. There is no Wi-Fi and no cloud.

The library features dual Bluetooth backends:
1. **BTstack Backend (Primary / Recommended):** Lightweight, standards-compliant, and built to seamlessly combine with **[Bluepad32](https://github.com/ricardoquesada/bluepad32)** to create a controller-to-Wiimote adapter on a single ESP32. Does not suffer from Bluedroid's 8-attribute SDP limitation.
2. **Bluedroid Backend (Fallback):** Provides zero-setup plug-and-play compatibility for stock Arduino-ESP32 installations where BTstack is not installed.

The stack is C and uses ESP-IDF / BTstack APIs. `src/Wiimote.h` is a thin C++ wrapper for Arduino sketches. It does not pull Arduino types into the protocol code.

## Hardware

**Original ESP32 only** (the dual-core chip with Bluetooth Classic / BR/EDR).

ESP32-S3, C3, C6, and H2 do not have a BR/EDR radio. They cannot host this library. This is not a BLE gamepad.

Targeted host SDKs:

- **Bluepad32 / BTstack (ESP-IDF 4.4 & 5.x):** The premier setup for controller-to-Wiimote adapters. Both Bluepad32 (HID Host) and WiimoteAnything (HID Device) run concurrently on the same BTstack engine on a single ESP32!
- **ESP-IDF with BTstack component:** Dedicated standalone BTstack builds (`examples/BTstackStandalone`).
- **Arduino-ESP32 2.0.17 and 3.x:** Board **ESP32 Dev Module**. Automatically uses the Bluedroid fallback if BTstack is not present, or BTstack if available.
- **ESP-IDF with Bluedroid Classic enabled:** On IDF 5 enable `CONFIG_BT_L2CAP_ENABLED` and `CONFIG_BT_SDP_COMMON_ENABLED`.

## Bluepad32 Controller Adapter

Combining WiimoteAnything with Bluepad32 allows you to connect modern wireless gamepads (DualSense, DualShock 4, Xbox Wireless, Switch Pro, 8BitDo) to the ESP32 via Bluetooth and output them to a Nintendo Wii as a Wiimote + Classic Controller in real time:

See [`examples/Bluepad32_Wiimote_Adapter`](examples/Bluepad32_Wiimote_Adapter) for the complete implementation!

```c
// Bluepad32 manages BTstack; WiimoteAnything attaches to it:
wiimote_config_t cfg = {
    .extension = WIIMOTE_EXT_CLASSIC,
    .battery = 0xC0,
    .callbacks = &callbacks,
    .external_stack = true,
};
wiimote_init(&cfg);
wiimote_sync();
```

## What a Wii will accept

Implemented from the public Wiibrew Wiimote, extension, and MotionPlus notes:

- Sync runs for 20 seconds (player LEDs blink; how many depends on the battery byte). The ESP32 is connectable and limited-discoverable (`LIAC`); it does not inquire. Press the red SYNC button on the Wii console face during that window. The Wii's player LEDs blink while it searches, then it pages the remote. After the window the ESP32 stays connectable and non-discoverable, the way a bonded remote does. Pairing is legacy PIN only: the host BD_ADDR reversed, 6 bytes. SSP is turned off.
- Status (`0x20`), memory/register read (`0x21`) and write, ack (`0x22`).
- Player LEDs, the rumble bit on every output report, and battery level (low-battery flag below `0x20`).
- Data modes `0x30`–`0x37`, `0x3d`, and interleaved `0x3e`/`0x3f`. Core reports run at 100 Hz after the Wii sends `0x12`. `0x3e`/`0x3f` alternate, so a full IR sample is 50 Hz. Unsolicited extension-change status stops reports until the next `0x12`.
- EEPROM image with the published IR block and an accelerometer calibration (0 g = `0x200`, +1 g = `0x26A`) whose checksum matches what we transmit. Idle pose is face-up: X = Y = `0x200`, Z = `0x26A`.
- Extension init `0x55` at `0xF0`, `0x00` at `0xFB`, I2C address `0x52`, and the extension cipher (constant `0x97` pads, or the 16-byte key at `0x40`). Identifier bytes `0xFA`–`0xFF` stay plaintext.
- Extensions, changed at runtime with `wiimote_set_extension`:
  - None
  - Nunchuk (`00 00 A4 20 00 00`)
  - Classic Controller (`00 00 A4 20 01 01`), 6-byte report, buttons active-low
  - Classic Controller Pro (`01 00 A4 20 01 01`)
  - MotionPlus (`0xA60000`, inactive id `00 00 A6 20 00 05`, activate with `0x04` at `0xFE` after `0x55` at `0xF0`, active id `00 00 A4 20 04 05`)
  - MotionPlus + Nunchuk passthrough (activate with `0x05`, id `00 00 A4 20 05 05`, 200 Hz combined, gyro and nunchuk frames alternate; byte 5 bit 1 marks a gyro frame)
- MotionPlus gyro uses the slow scale near rest (`0x1F7F`) and the fast scale past about 440 deg/s.
- IR is optional. `wiimote_set_ir_dot` feeds up to four points (no camera). Basic, extended, and full layouts follow the reporting mode. Full-mode bounding boxes are synthesized from the dot.
- If no IMU is bound, reports keep that idle accel and a resting gyro so the remote still connects.

Deliberately not implemented:

- Speaker PCM (`0x14` / `0x18` / `0x19` and `0xA20000` are accepted and ignored; the speaker-enabled status bit does follow `0x14`)
- Classic Controller packed inside MotionPlus (a write of `0x07` activates MotionPlus alone)
- Classic data formats `0x02` and `0x03` (the format byte is stored; reports stay in the standard 6-byte layout)
- The power button

## Arduino

Copy this folder into your Arduino libraries directory, or point the CLI at it:

```bash
arduino-cli config set directories.user ~/Arduino   # if you have not already
mkdir -p ~/Arduino/libraries
ln -s "$PWD" ~/Arduino/libraries/WiimoteAnything
```

Board: **ESP32 Dev Module**. Leave flash and PSRAM at the board defaults. Upload any sketch under `examples/`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 examples/BasicButtons
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 examples/BasicButtons
```

Open the serial monitor at 115200. Reset the ESP32, then while it is inside the 20 second window press the red SYNC button on the Wii console face (not just power). The Wii's player LEDs blink while it searches. The ESP32 should log an inbound ACL and a PIN request, not an inquiry result for the console. The sketch prints `connected` after both HID channels are up.

On Arduino-ESP32 3.x the wrapper includes `esp32-hal-alloc-bt-classic-mem.h` when that header exists, so the core keeps Classic controller memory.

`esp_bluedroid_init_with_cfg()` is used only when the header defines `BT_BLUEDROID_INIT_CONFIG_DEFAULT`. Arduino-ESP32 2.0.17 and 3.0.7 (IDF 5.1.4) do not have `esp_bluedroid_config_t`; those cores call `esp_bluedroid_init()`.

The 3.x prebuilt sdkconfig leaves `CONFIG_BT_L2CAP_ENABLED` unset (and, from 3.2 onward, `CONFIG_BT_SDP_COMMON_ENABLED` unset as well). That does not remove the stack. A2DP and SPP are enabled, and `libbt.a` for the original ESP32 still exports `L2CA_Register`, `SDP_CreateRecord`, `SDP_AddAttribute`, `BTM_SetSecurityLevel`, and `btu_task_post`. Checked with `nm` on these archives, not by running the Arduino compiler:

| Core | ESP-IDF | `esp_bluedroid_config_t` | L2CAP / SDP symbols in `libbt.a` |
| --- | --- | --- | --- |
| 2.0.17 | 4.4.7 | no | present |
| 3.0.7 | 5.1.4 | no | present |
| 3.2.1 | 5.4.2 | yes | present |
| 3.3.12 | 5.5.5 | yes | present |

None of those four cores omit the symbols, so they should link. HID device objects in the same archive are empty. Cores that were not unpacked (for example 3.1.x) were not checked.

## ESP-IDF component

Point `EXTRA_COMPONENT_DIRS` at this repo (the root `CMakeLists.txt` is the component). The component name is the directory name of your checkout. `examples/idf` is a tiny project that already does this. From that directory, after sourcing ESP-IDF:

```bash
idf.py set-target esp32
idf.py build
```

That project builds with ESP-IDF 5.x for the original ESP32.

### BTstack / Bluepad32 Configuration (sdkconfig.defaults):

```
CONFIG_BT_ENABLED=y
CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=y
CONFIG_BT_BLUEDROID_ENABLED=n
CONFIG_BT_NIMBLE_ENABLED=n
CONFIG_BT_CONTROLLER_ONLY=y
```

### Bluedroid Fallback Configuration:

If using the legacy Bluedroid backend on ESP-IDF 5 instead:

```
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=y
CONFIG_BT_L2CAP_ENABLED=y
CONFIG_BT_SDP_COMMON_ENABLED=y
```

```c
#include "wiimote.h"

wiimote_pins_t pins;
wiimote_pins_init(&pins);
pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
pins.led_gpio[0] = 26;
pins.rumble_gpio = 25;

wiimote_config_t cfg = {
    .extension = WIIMOTE_EXT_NONE,
    .battery = 0xC0,
    .pins = &pins,
};
ESP_ERROR_CHECK(wiimote_init(&cfg));
ESP_ERROR_CHECK(wiimote_sync());
```

`wiimote_pins_init` sets every GPIO to `-1`, buttons active-low, and ADC spans `0..4095` centered at `2048` with a deadzone of `200`. A battery value of `0` in the config selects `0xC0`.

## Pins

| Role | API | Notes |
| --- | --- | --- |
| Buttons | `button_gpio[]`, `buttons_active_low` | One level for the whole map. Internal pull-ups are on when active-low. GPIO 34–39 have no pulls. |
| Sticks / triggers | `axis[]` | ADC1 only: GPIO 36, 39, 32, 33, 34, 35 (and 37/38 when they are free). Center, min, max, deadzone are raw counts. Sticks map to the nunchuk `0–255` or classic `6-bit` / `5-bit` ranges. Triggers map `0..1`. |
| Rumble | `rumble_gpio`, `rumble_pwm` | Digital GPIO, or LEDC PWM at about 1 kHz when `rumble_pwm` is set. |
| Player LEDs | `led_gpio[4]` | Driven from the Wii's LED report, bit 0 = LED 1, active high. `callbacks.leds` still runs if you leave the pins at `-1`. |
| IR | `wiimote_set_ir_dot` / `wiimote_set_ir_raw` | No camera required. `size < 0` hides a dot. |
| Accel / gyro | driver or `wiimote_set_motion_callback` | See below. Manual `wiimote_set_accel` / `wiimote_set_gyro` work until the next sample from a bound driver or callback. |

Avoid strapping pins 0, 2, 12, and 15, and never use the flash pins 6–11. The examples use 4, 5, 13, 14, 16–19, 21–23, 25–27, 32, and 33.

Callbacks run on the Bluetooth task. Set a flag and print from `loop()`. Do not block in a callback.

## Extensions

Select one with `wiimote_config_t.extension` or `wiimote_set_extension` after `wiimote_init`. Changing it while connected sends the extension-change status report, which is what makes a game re-read the bus.

Nunchuk C and Z are `WIIMOTE_BUTTON_NUNCHUK_C` / `WIIMOTE_BUTTON_NUNCHUK_Z`. Classic buttons are the `WIIMOTE_BUTTON_CLASSIC_*` values (they are not the core A/B/+ buttons). Nunchuk stick axes are `WIIMOTE_AXIS_NUNCHUK_X` / `Y`. Classic sticks and triggers are `WIIMOTE_AXIS_CLASSIC_*`.

`WIIMOTE_EXT_MOTION_PLUS_NUNCHUK` presents a nunchuk until the game writes `0x05` to `0xA600FE`. After that, frames alternate. A normal nunchuk's `0x55` at `0xA400F0` does not look like a MotionPlus deactivate; that only happens while MotionPlus is active.

## MPU6050 wiring

GY-521 (or any MPU6050) to the original ESP32:

| Module | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| AD0 | GND (address `0x68`; `0x69` if AD0 is high) |
| INT | leave open |

```c
wiimote_use_mpu6050(21, 22, 0x68);
```

The driver wakes the chip, sets accel to ±2 g and gyro to ±2000 deg/s, and uses I2C port 0 at 400 kHz. Pass `0` for the address to try the default and then the alternate. The same call pattern exists for the MPU6500 / MPU9250 (`wiimote_use_mpu6500`, WHO_AM_I `0x70` or `0x71`) and the LSM6DS3 / LSM6DSOX (`wiimote_use_lsm6ds3`, addresses `0x6A` / `0x6B`).

One IMU feeds the Wiimote body. Mapped sensor X/Y/Z are copied onto the Wiimote axes; with identity map `{1, 2, 3}`, +Z up while the board lies face-up is +1 g. Gyro pitch is mapped X, roll is mapped Y, yaw is mapped Z. Nunchuk accel stays at the idle pose unless you call `wiimote_set_nunchuk_accel`.

`wiimote_set_motion_callback` replaces the driver. Fill milli-g and degrees per second through the pointers. `wiimote_set_imu_map` takes three values in ±1, ±2, ±3. A negative value flips that axis.

## Host tests

Protocol, extension, and cipher code does not include ESP-IDF headers.

```bash
make -C tests
```

That covers button and accel packing, nunchuk and classic bytes, extension init, a missing-extension error, MotionPlus activate and passthrough alternation, the extension cipher, EEPROM calibration, status, rumble, LEDs, and a basic IR dot.

## License

MIT. See `LICENSE`.
