/* SPDX-License-Identifier: MIT */
#pragma once

#include "../include/wiimote.h"

/** Thin Arduino-facing wrapper. The protocol stack stays in the C API. */
class Wiimote {
public:
    bool begin(const wiimote_config_t *config);
    bool sync();
    void stop();
    bool registerBtstackServices();

    bool setExtension(wiimote_extension_t extension);
    bool setButton(wiimote_button_t button, bool pressed);
    bool setAxis(wiimote_axis_t axis, float value);
    bool setNunchukStick(uint8_t x, uint8_t y);
    bool setAccel(int16_t x, int16_t y, int16_t z);
    bool setNunchukAccel(int16_t x, int16_t y, int16_t z);
    bool setGyro(float pitch_dps, float roll_dps, float yaw_dps);
    bool setIrDot(int index, int x, int y, int size);
    bool setIrRaw(const uint8_t *data, size_t len);
    bool setBattery(uint8_t level);
    bool setMotionCallback(wiimote_motion_cb_t cb, void *user);
    bool setImuMap(const int8_t map[3]);
    bool useMpu6050(int sda, int scl, uint8_t addr);
    bool useMpu6500(int sda, int scl, uint8_t addr);
    bool useLsm6ds3(int sda, int scl, uint8_t addr);

    bool connected() const;
    uint8_t ledMask() const;
};
