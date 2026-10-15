# Bluepad32 → Wii adapter

One original ESP32 is an HID host for one modern gamepad (Bluepad32) and, at
the same time, a Wii Remote with a Classic Controller plugged in
(WiimoteAnything). Both share one BTstack instance.

Anything Bluepad32 supports over Bluetooth Classic should work: DualSense,
DualShock 4, Switch Pro, 8BitDo (SN30 Pro in Switch mode: hold **Start + Y**
to power on). Xbox controllers on current firmware use BLE; set
`ADAPTER_ENABLE_BLE 1` in `main/main.c` for those (BLE scanning costs some
radio time on the Wii link).

## Build

1. Clone Bluepad32 (with submodules) next to the library:

   ```bash
   git clone --recursive https://github.com/ricardoquesada/bluepad32.git components/bluepad32
   ```

   (`components/bluepad32` in the repo root, or inside this example folder.)

2. Build and flash (ESP-IDF 5.5 or 6.x):

   ```bash
   cd examples/Bluepad32_Wiimote_Adapter
   idf.py set-target esp32
   idf.py build flash monitor
   ```

   If you have an `sdkconfig` from an older version of this example, delete
   it so the new `sdkconfig.defaults` apply.

## First-time pairing

The gamepad always connects first and the Wii second. The ESP32 radio
cannot reach a gamepad while the Wii link exists (it is the slave there; the
log shows page timeout `0x04`), so the adapter only holds the Wii link while
a gamepad is connected and pages the Wii again when the gamepad comes back.

1. Boot the ESP32 and put the gamepad in pairing mode (DualSense: hold
   **Create + PS** until the light bar flashes; 8BitDo: **Start + Y**, then
   hold **Pair**). The log shows `Gamepad ready`.
2. With no Wii remembered, the sync window then opens by itself for 20 s:
   press the red **SYNC** button on the Wii (behind the SD card door). The log
   shows `Console ... connected`.

Both are remembered (NVS).

## Every day

- Turn the gamepad on (PS / Home). It reconnects to the ESP32, and the ESP32
  pages the Wii. If the Wii was not ready, any button press retries (every 6 s).
- When the Wii drops the remote while it stays on (starting a game, radio
  trouble), the ESP32 pages it again by itself for up to 60 s.
- Wii turned off (or HOME menu → disconnect): press a button to reconnect.
- Gamepad switched off or out of range: the Wii link stays up with all inputs
  released, so the game sees an idle remote until the gamepad is back.

## Controls

| Gamepad (position) | Wii |
| --- | --- |
| Left / right stick | Classic Controller left / right stick |
| D-pad | Classic D-pad (and Wii Remote D-pad) |
| East / South / North / West face buttons | Classic **a / b / x / y** (Nintendo layout by position) |
| L1 / R1 | Classic **ZL / ZR** |
| L2 / R2 (analog) | Classic **L / R** analog, click at 85 % |
| Select / Start | **− / +** |
| PS / Home / Guide | **HOME** (Classic and Wii Remote) |

Wii Remote A / B / 1 / 2 / + / − / D-pad are mirrored too
(`ADAPTER_MIRROR_CORE_BUTTONS`), so "Press A" screens and the HOME menu work.
Set `ADAPTER_LABEL_LAYOUT 1` in `main.c` to map by label (Xbox A → Wii a)
instead of by position.

Rumble from the Wii drives the gamepad's motors; the Wii's player LEDs show on
the gamepad.

## Buttons on the adapter

| Action | Effect |
| --- | --- |
| **BOOT** short press | Sync window (pair with a Wii, or re-pair) |
| Hold **Select + Start** 3 s | Same, from the gamepad |
| Hold **BOOT** 5 s | Forget the Wii and all gamepads |

The Bluepad32 console on the serial port also works (`list_bluetooth_keys`,
`del_bluetooth_keys`, ...).

## How the sharing works

- `wiimote_init()` runs after `btstack_init()` and before `uni_init()`.
  WiimoteAnything takes L2CAP PSM 0x11 / 0x13 and forwards every channel that is
  not the console to `uni_bt_packet_handler`.
- The console is recognised by its class of device (the Wii is `0x000448`,
  gamepads are "peripheral") or by being the console synced before. Its
  connection request and PIN request are hidden from Bluepad32.
- The ESP32 radio stalls when it is slave on one link and master on another,
  so the ESP32 stays master on both: it pages the Wii itself with role
  switching refused, and turns off role switching and sniff mode on the Wii
  link. When the Wii paged us (sync), the adapter hangs up and pages it back.
- Order matters: gamepad link first, then the Wii. Without a gamepad the Wii
  link is released (the pairing is kept).
- Gamepad scanning (inquiry) runs only while no gamepad is connected and no
  sync window is open. Bonded gamepads reconnect by paging the ESP32, which
  needs no scanning.
- BLE is off by default (`ADAPTER_ENABLE_BLE`), so BLE scanning does not take
  radio time from the Wii link.
