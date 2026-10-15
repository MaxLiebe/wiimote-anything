/* SPDX-License-Identifier: MIT */
#pragma once

#include "../include/wiimote.h"

/**
 * Arduino-facing wrapper around the C API. The protocol stack stays in C.
 *
 * Quick start, nothing wired:
 *
 *     Wiimote wiimote;
 *     wiimote.onConnect([] { Serial.println("connected"); });
 *     wiimote.begin(WIIMOTE_EXT_CLASSIC);
 *     wiimote.sync();                       // press SYNC on the Wii
 *     ...
 *     wiimote.press(WIIMOTE_BUTTON_A);
 *     wiimote.setAxis(WIIMOTE_AXIS_CLASSIC_LX, 0.5f);
 *
 * There is one Wiimote per ESP32. Event handlers run on the Bluetooth task:
 * keep them short (set a flag, print from loop()).
 */
class Wiimote {
public:
    typedef void (*EventHandler)();
    typedef void (*LedHandler)(uint8_t mask);
    typedef void (*RumbleHandler)(bool on);
    typedef void (*SyncHandler)(bool active);

    /** Start with no wiring: inputs come from press()/setAxis()/... */
    bool begin(wiimote_extension_t extension = WIIMOTE_EXT_NONE);
    /** Start with GPIO buttons, sticks, LEDs and rumble (see wiimote_pins_t). */
    bool begin(wiimote_extension_t extension, const wiimote_pins_t &pins);
    /**
     * Full control. The handlers set with onConnect() and friends are not
     * used: put your callbacks in config->callbacks instead.
     */
    bool begin(const wiimote_config_t *config);

    /** Set before begin(). */
    void onConnect(EventHandler handler) { s_on_connect = handler; }
    void onDisconnect(EventHandler handler) { s_on_disconnect = handler; }
    void onLeds(LedHandler handler) { s_on_leds = handler; }
    void onRumble(RumbleHandler handler) { s_on_rumble = handler; }
    void onSync(SyncHandler handler) { s_on_sync = handler; }

    /** Red SYNC button: discoverable for 20 s. Press SYNC on the Wii. */
    bool sync();
    bool syncing() const;
    /** Reconnect to the console synced before (BTstack backend only). */
    bool connect();
    bool connecting() const;
    bool disconnect();
    bool hasPairedHost() const;
    bool forgetHost();
    void stop();

    bool connected() const;
    /** Player LEDs last set by the Wii. Bit 0 is LED 1. */
    uint8_t ledMask() const;

    bool setExtension(wiimote_extension_t extension);
    bool setButton(wiimote_button_t button, bool pressed);
    bool press(wiimote_button_t button) { return setButton(button, true); }
    bool release(wiimote_button_t button) { return setButton(button, false); }
    /** Sticks -1..1 (center 0), triggers 0..1. */
    bool setAxis(wiimote_axis_t axis, float value);
    bool setNunchukStick(uint8_t x, uint8_t y);
    bool setAccel(int16_t x, int16_t y, int16_t z);
    bool setNunchukAccel(int16_t x, int16_t y, int16_t z);
    bool setGyro(float pitch_dps, float roll_dps, float yaw_dps);
    bool setIrDot(int index, int x, int y, int size);
    bool setIrRaw(const uint8_t *data, size_t len);
    bool setBattery(uint8_t level);
    /** Send changed input now instead of on the next 5 ms tick. */
    void sendNow();

    bool setMotionCallback(wiimote_motion_cb_t cb, void *user);
    bool setImuMap(const int8_t map[3]);
    bool useMpu6050(int sda, int scl, uint8_t addr = 0);
    bool useMpu6500(int sda, int scl, uint8_t addr = 0);
    bool useLsm6ds3(int sda, int scl, uint8_t addr = 0);

private:
    bool start(wiimote_extension_t extension, const wiimote_pins_t *pins);

    static EventHandler s_on_connect;
    static EventHandler s_on_disconnect;
    static LedHandler s_on_leds;
    static RumbleHandler s_on_rumble;
    static SyncHandler s_on_sync;
    static void connectedThunk(void *user);
    static void disconnectedThunk(void *user);
    static void ledsThunk(uint8_t mask, void *user);
    static void rumbleThunk(bool on, void *user);
    static void syncThunk(bool active, void *user);
};
