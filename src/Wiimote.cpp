/* SPDX-License-Identifier: MIT */
#include "Wiimote.h"

#if defined(ARDUINO)
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif
#if __has_include("esp32-hal-alloc-ble-mem.h")
#include "esp32-hal-alloc-ble-mem.h"
#endif
#endif

bool Wiimote::begin(const wiimote_config_t *config)
{
    return wiimote_init(config) == ESP_OK;
}

bool Wiimote::sync()
{
    return wiimote_sync() == ESP_OK;
}

bool Wiimote::connect()
{
    return wiimote_connect() == ESP_OK;
}

bool Wiimote::disconnect()
{
    return wiimote_disconnect() == ESP_OK;
}

bool Wiimote::hasPairedHost() const
{
    return wiimote_has_paired_host();
}

bool Wiimote::forgetHost()
{
    return wiimote_forget_host() == ESP_OK;
}

void Wiimote::stop()
{
    wiimote_stop();
}

bool Wiimote::registerBtstackServices()
{
    return wiimote_btstack_register_services() == ESP_OK;
}

bool Wiimote::setExtension(wiimote_extension_t extension)
{
    return wiimote_set_extension(extension) == ESP_OK;
}

bool Wiimote::setButton(wiimote_button_t button, bool pressed)
{
    return wiimote_set_button(button, pressed) == ESP_OK;
}

bool Wiimote::setAxis(wiimote_axis_t axis, float value)
{
    return wiimote_set_axis(axis, value) == ESP_OK;
}

bool Wiimote::setNunchukStick(uint8_t x, uint8_t y)
{
    return wiimote_set_nunchuk_stick(x, y) == ESP_OK;
}

bool Wiimote::setAccel(int16_t x, int16_t y, int16_t z)
{
    return wiimote_set_accel(x, y, z) == ESP_OK;
}

bool Wiimote::setNunchukAccel(int16_t x, int16_t y, int16_t z)
{
    return wiimote_set_nunchuk_accel(x, y, z) == ESP_OK;
}

bool Wiimote::setGyro(float pitch_dps, float roll_dps, float yaw_dps)
{
    return wiimote_set_gyro(pitch_dps, roll_dps, yaw_dps) == ESP_OK;
}

bool Wiimote::setIrDot(int index, int x, int y, int size)
{
    return wiimote_set_ir_dot(index, x, y, size) == ESP_OK;
}

bool Wiimote::setIrRaw(const uint8_t *data, size_t len)
{
    return wiimote_set_ir_raw(data, len) == ESP_OK;
}

bool Wiimote::setBattery(uint8_t level)
{
    return wiimote_set_battery(level) == ESP_OK;
}

bool Wiimote::setMotionCallback(wiimote_motion_cb_t cb, void *user)
{
    return wiimote_set_motion_callback(cb, user) == ESP_OK;
}

bool Wiimote::setImuMap(const int8_t map[3])
{
    return wiimote_set_imu_map(map) == ESP_OK;
}

bool Wiimote::useMpu6050(int sda, int scl, uint8_t addr)
{
    return wiimote_use_mpu6050(sda, scl, addr) == ESP_OK;
}

bool Wiimote::useMpu6500(int sda, int scl, uint8_t addr)
{
    return wiimote_use_mpu6500(sda, scl, addr) == ESP_OK;
}

bool Wiimote::useLsm6ds3(int sda, int scl, uint8_t addr)
{
    return wiimote_use_lsm6ds3(sda, scl, addr) == ESP_OK;
}

bool Wiimote::connected() const
{
    return wiimote_is_connected();
}

uint8_t Wiimote::ledMask() const
{
    return wiimote_led_mask();
}
