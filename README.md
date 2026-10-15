# WiimoteAnything

The ESP32 is the Wiimote. A real Wii is the host.

This library speaks Bluetooth Classic HID the way a Nintendo RVL-CNT-01 does: device name `Nintendo RVL-CNT-01`, class of device `0x002504`, L2CAP PSM `0x11` (control) and `0x13` (interrupt), report headers `0xA2` / `0xA1`, and the extension bus games actually initialize (Nunchuk, Classic Controller, MotionPlus). Pairing during sync uses the Wii's BD_ADDR reversed as the PIN. There is no Wi-Fi and no cloud.

Feed it inputs from GPIO buttons and analog sticks, an IMU, or your own code, and the Wii sees a remote.

Built with it: **[Wii Gamepad Adapter](https://github.com/MaxLiebe/wii-gamepad-adapter)**, which turns a DualSense, Switch Pro, 8BitDo, Xbox, ... gamepad into a Wii Remote with a Classic Controller.

## Hardware

**Original ESP32 only** (the dual-core chip with Bluetooth Classic / BR/EDR), for example an ESP32-WROOM-32 dev board.

ESP32-S3, C3, C6, and H2 do not have a BR/EDR radio. They cannot host this library. This is not a BLE gamepad.

## Quick start (Arduino)

No ESP-IDF install needed: the stock Arduino-ESP32 core (2.0.17 or 3.x) is enough.

1. Install the **esp32** boards package (Boards Manager, by Espressif).
2. Install this library: Sketch → Include Library → Add .ZIP Library (a ZIP of this repo), or clone it into your `Arduino/libraries` folder.
3. Board: **ESP32 Dev Module**. Open File → Examples → WiimoteAnything → **SerialRemote** and upload it.
4. Open the serial monitor at 115200, then press the red **SYNC** button on the Wii (behind the SD card door) within 20 s. Type `a`, `h`, `w`, ... to press buttons.

```cpp
#include <Wiimote.h>

Wiimote wiimote;

void setup() {
    wiimote.onConnect([] { Serial.println("connected"); });
    wiimote.begin(WIIMOTE_EXT_CLASSIC);   // or WIIMOTE_EXT_NONE, _NUNCHUK, _MOTION_PLUS
    wiimote.sync();                       // 20 s window: press SYNC on the Wii
}

void loop() {
    wiimote.setButton(WIIMOTE_BUTTON_A, digitalRead(4) == LOW);
    wiimote.setAxis(WIIMOTE_AXIS_CLASSIC_LX, (analogRead(36) - 2048) / 2048.0f);
    delay(5);
}
```

| Example | What it shows | Wiring |
| --- | --- | --- |
| [`SerialRemote`](examples/SerialRemote) | Remote + Classic Controller driven from the serial monitor | none |
| [`BasicButtons`](examples/BasicButtons) | GPIO buttons, player LEDs, rumble | buttons, LEDs |
| [`NunchukFromJoysticks`](examples/NunchukFromJoysticks) | An analog stick as a Nunchuk | buttons, stick |
| [`MotionPlusImu`](examples/MotionPlusImu) | MPU6050 as accelerometer + MotionPlus | MPU6050 |

`Wiimote` (in `src/Wiimote.h`) is a thin C++ wrapper: `begin()`, `sync()`, `press()` / `release()` / `setButton()`, `setAxis()`, `setExtension()`, `setIrDot()`, `useMpu6050()`, and `onConnect` / `onDisconnect` / `onLeds` / `onRumble` / `onSync` handlers. Everything else is the C API in [`include/wiimote.h`](include/wiimote.h), which you can call directly from a sketch.

Handlers run on the Bluetooth task. Set a flag and print from `loop()`; do not block in a handler.

### Arduino notes

On the stock Arduino core the library uses the core's **Bluedroid** stack. That works for syncing, but a Bluedroid remote cannot reconnect by itself: after a reboot, press SYNC on the Wii again. Reconnecting without SYNC (`connect()`), the way a bonded remote does, needs the BTstack backend (ESP-IDF, below, or the Bluepad32 Arduino core, which ships BTstack; there set `external_stack = true`).

On Arduino-ESP32 3.x the wrapper includes `esp32-hal-alloc-bt-classic-mem.h` when that header exists, so the core keeps Classic controller memory. The 3.x prebuilt sdkconfig leaves `CONFIG_BT_L2CAP_ENABLED` unset (and, from 3.2 onward, `CONFIG_BT_SDP_COMMON_ENABLED` as well). That does not remove the stack: `libbt.a` for the original ESP32 still exports `L2CA_Register`, `SDP_CreateRecord`, `SDP_AddAttribute`, `BTM_SetSecurityLevel`, and `btu_task_post`. Checked with `nm` on these cores:

| Core | ESP-IDF | `esp_bluedroid_config_t` | L2CAP / SDP symbols in `libbt.a` |
| --- | --- | --- | --- |
| 2.0.17 | 4.4.7 | no | present |
| 3.0.7 | 5.1.4 | no | present |
| 3.2.1 | 5.4.2 | yes | present |
| 3.3.12 | 5.5.5 | yes | present |

All four examples compile with Arduino-ESP32 3.3.12. `esp_bluedroid_init_with_cfg()` is used only when the header defines `BT_BLUEDROID_INIT_CONFIG_DEFAULT`; older cores call `esp_bluedroid_init()`.

From the command line:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --library . examples/SerialRemote
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 examples/SerialRemote
```

## Sync, reconnect, forget

- `wiimote_sync()` is the red SYNC button on the back of a remote: discoverable for 20 s. Press SYNC on the Wii. The console that pairs is stored (BTstack TLV / NVS) together with its link key.
- `wiimote_connect()` is pressing a button on a bonded remote: page the stored console and open the HID channels. Works after a reboot without syncing again. Like a real remote it does not authenticate the console itself; the console (system menu or libogc homebrew) does that with the stored link key. `wiimote_is_connecting()` is true while a page is under way. `ESP_ERR_NOT_FOUND` if nothing was synced. BTstack backend only.
- `wiimote_has_paired_host()`, `wiimote_forget_host()`, `wiimote_disconnect()`, `wiimote_is_syncing()`.
- `callbacks.sync_changed(active)` reports the sync window opening and closing.

## Bluetooth backends

The backend is chosen at compile time: BTstack when `btstack.h` is on the include path, Bluedroid otherwise. Define `WIIMOTE_BACKEND_BTSTACK` or `WIIMOTE_BACKEND_BLUEDROID` to force one.

1. **BTstack.** Standalone, or sharing one BTstack instance with **[Bluepad32](https://github.com/ricardoquesada/bluepad32)** so a single ESP32 is a gamepad host and a Wii Remote at the same time. Remembers the Wii it synced with and reconnects by paging it, the way a bonded remote does.
2. **Bluedroid.** What the stock Arduino-ESP32 core and a default ESP-IDF project have. Sync only (no reconnect-by-paging).

The stack is C and uses ESP-IDF / BTstack APIs (which the Arduino core includes). It does not pull Arduino types into the protocol code.

## ESP-IDF (optional)

Point `EXTRA_COMPONENT_DIRS` at this repo; the root `CMakeLists.txt` is the component, named after the directory of your checkout. Two small projects already do this:

| Project | Backend | |
| --- | --- | --- |
| [`examples/esp-idf/btstack`](examples/esp-idf/btstack) | BTstack, owned by the library | Reconnects after a reboot when you press A |
| [`examples/esp-idf/bluedroid`](examples/esp-idf/bluedroid) | Bluedroid | Smallest possible app |

```bash
cd examples/esp-idf/btstack
idf.py set-target esp32
idf.py build flash monitor
```

The BTstack example takes BTstack from Bluepad32's checkout in `components/bluepad32` (`git clone --recursive https://github.com/ricardoquesada/bluepad32.git components/bluepad32`), from `components/btstack`, or from `$BTSTACK_ROOT`. Both build with ESP-IDF 5.x and 6.x.

### BTstack configuration (sdkconfig.defaults)

BTstack is the host, so the IDF provides only the controller. BTstack's ESP32 port compiles its Classic sources only in dual mode, so use BTDM even if you never touch BLE (`wiimote_target.h` stops the build otherwise):

```
CONFIG_BT_ENABLED=y
CONFIG_BT_CONTROLLER_ONLY=y
CONFIG_BTDM_CTRL_MODE_BTDM=y
CONFIG_BTDM_CTRL_MODEM_SLEEP=n
CONFIG_FREERTOS_HZ=1000
```

### Bluedroid configuration

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

`wiimote_pins_init` sets every GPIO to `-1`, buttons active-low, and ADC spans `0..4095` centered at `2048` with a deadzone of `200`. A battery value of `0` in the config selects `0xC0`. `pins` may be `NULL` when all input comes from code.

### Sharing BTstack with Bluepad32

The Wii (host to us) and a reconnecting gamepad (device to us) both open L2CAP PSM `0x11`/`0x13`, so WiimoteAnything owns those PSMs and forwards every channel that is not the console to Bluepad32. A connection is the console when it comes from the console synced before, when its class of device is the Wii's (`0x000448`), or, during the sync window, when it is any non-peripheral host. WiimoteAnything hides the console's connection request and PIN request from Bluepad32, so Bluepad32 does not allocate a gamepad slot for the Wii or answer the PIN with the wrong code.

It otherwise leaves gamepad links alone: an incoming gamepad channel is offered to Bluepad32 once the link meets Bluepad32's GAP security level (as Bluepad32's own PSM services would require), and radio tuning of a gamepad link (ESP32 master, no sniff, short poll interval) happens only when the application calls `wiimote_btstack_tune_link()` once the gamepad is ready. It never starts or stops gamepad scanning; that is the application's job.

```c
#include "wiimote.h"
#include "wiimote_btstack.h"

btstack_init();
wiimote_btstack_set_fallback_handler(uni_bt_packet_handler);  // non-Wii traffic -> Bluepad32
wiimote_config_t cfg = {
    .extension = WIIMOTE_EXT_CLASSIC,
    .callbacks = &callbacks,
    .external_stack = true,          // the app owns BTstack
};
wiimote_init(&cfg);                  // before uni_init(): we must see the Wii first
uni_platform_set_custom(&platform);  // on_device_ready: wiimote_btstack_tune_link(...)
uni_init(0, NULL);
btstack_run_loop_execute();
```

With the BTstack backend all callbacks run on the BTstack thread, so they may call BTstack and Bluepad32 `_unsafe` functions directly. [Wii Gamepad Adapter](https://github.com/MaxLiebe/wii-gamepad-adapter) is the complete example.

## What a Wii will accept

Implemented from the public Wiibrew Wiimote, extension, and MotionPlus notes:

- Sync runs for 20 seconds (player LEDs blink; how many depends on the battery byte). The ESP32 is connectable and limited-discoverable (`LIAC`); it does not inquire. Press the red SYNC button on the Wii console face during that window. The Wii's player LEDs blink while it searches, then it pages the remote. After the window the ESP32 stays connectable and non-discoverable, the way a bonded remote does. Pairing is legacy PIN: the host BD_ADDR reversed, 6 bytes. The Wii has no Secure Simple Pairing, so it never uses SSP even while SSP stays on for gamepads.
- Status (`0x20`), memory/register read (`0x21`) and write, ack (`0x22`).
- Player LEDs, the rumble bit on every output report, and battery level (low-battery flag below `0x20`).
- Data modes `0x30`–`0x37`, `0x3d`, and interleaved `0x3e`/`0x3f`. Core reports run at 100 Hz after the Wii sends `0x12`. `0x3e`/`0x3f` alternate, so a full IR sample is 50 Hz. Unsolicited extension-change status stops reports until the next `0x12`.
- EEPROM image with the published IR block and an accelerometer calibration (0 g = `0x200`, +1 g = `0x26A`) whose checksum matches what we transmit. Idle pose is face-up: X = Y = `0x200`, Z = `0x26A`.
- Extension init: unencrypted (`0x55` at `0xF0`, `0x00` at `0xFB`) or encrypted (`0xAA` at `0xF0` plus the 16-byte key the console writes to `0x40`–`0x4F`, in either order, as retail software does). While `0xF0` holds `0xAA` every extension register read is encrypted, including the identifier at `0xFA`. The key schedule matches Dolphin's (`tests/test_proto.c` has a Dolphin-generated vector). I2C address `0x52`.
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
make -C tests            # BTSTACK_ROOT=... if Bluepad32 is not in components/bluepad32
```

`test_proto` covers button and accel packing, nunchuk and classic bytes (decoded the way Linux `hid-wiimote` decodes them), extension init, a missing-extension error, MotionPlus activate and passthrough alternation, the extension cipher, EEPROM calibration, status, rumble, LEDs, and a basic IR dot.

`test_btstack_backend` runs the BTstack backend against fake BTstack calls: a Wii syncing (PIN, events hidden from the host stack, channels, status / read / write / report mode replies, SET_REPORT handshake), a gamepad reconnecting while the Wii is connected (forwarded untouched, its channels held until the link is encrypted, tuned only on request), reconnecting by paging, a page that times out, and forgetting the console.

## License

MIT. See `LICENSE`.
