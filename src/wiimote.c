/* SPDX-License-Identifier: MIT */
#include "../include/wiimote.h"

#include "wiimote_bt.h"
#include "wiimote_hw.h"
#include "wiimote_imu.h"
#include "wiimote_internal.h"
#include "wiimote_target.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "wiimote";

static SemaphoreHandle_t s_lock;
static wiimote_proto_t *s_proto;
static wiimote_callbacks_t s_cb;
static wiimote_motion_cb_t s_motion;
static void *s_motion_user;
static int8_t s_map[3] = {1, 2, 3};
static uint8_t s_battery = 0xc0;
static volatile bool s_connected;
static volatile uint8_t s_leds;
static volatile bool s_syncing;
static bool s_sync_reported;
static int64_t s_sync_deadline_us;
static int64_t s_blink_us;
static bool s_blink_lit;
static TaskHandle_t s_task;

static int16_t mapped_axis(const int16_t in[3], int8_t which)
{
    int index = abs((int)which) - 1;
    int16_t value = 0;
    if (index >= 0 && index < 3) {
        value = in[index];
    }
    if (which < 0) {
        value = (int16_t)-value;
    }
    return value;
}

static int16_t milli_g_to_raw(int mg)
{
    int value = 0x200 + (mg * 0x6a) / 1000;
    if (value < 0) {
        value = 0;
    }
    if (value > 1023) {
        value = 1023;
    }
    return (int16_t)value;
}

static int stick_from_unit(float value, int lo, int hi, int center)
{
    if (value < -1.f) value = -1.f;
    if (value > 1.f) value = 1.f;
    if (value >= 0.f) {
        return center + (int)(value * (float)(hi - center) + 0.5f);
    }
    return center + (int)(value * (float)(center - lo) - 0.5f);
}

static void sample_motion(void)
{
    int16_t ain[3] = {0, 0, 0};
    int16_t gin[3] = {0, 0, 0};
    int16_t aout[3];
    int16_t gout[3];
    int i;
    bool got = false;
    if (s_motion != NULL) {
        s_motion(&ain[0], &ain[1], &ain[2], &gin[0], &gin[1], &gin[2], s_motion_user);
        got = true;
    } else if (wiimote_imu_read(&ain[0], &ain[1], &ain[2], &gin[0], &gin[1], &gin[2])) {
        got = true;
    }
    if (!got || s_proto == NULL) {
        return;
    }
    for (i = 0; i < 3; i++) {
        aout[i] = mapped_axis(ain, s_map[i]);
        gout[i] = mapped_axis(gin, s_map[i]);
    }
    wiimote_lock();
    wiimote_proto_set_accel(s_proto, (uint16_t)milli_g_to_raw(aout[0]), (uint16_t)milli_g_to_raw(aout[1]),
                             (uint16_t)milli_g_to_raw(aout[2]));
    wiimote_proto_set_gyro(s_proto, gout[0], gout[1], gout[2]);
    wiimote_unlock();
}

static uint8_t led_count_for_battery(uint8_t level)
{
    if (level >= 0xc0) return 4;
    if (level >= 0x80) return 3;
    if (level >= 0x40) return 2;
    return 1;
}

static void wiimote_task(void *arg)
{
    (void)arg;
    while (true) {
        int64_t now = esp_timer_get_time();
        wiimote_hw_poll();
        sample_motion();
        if (s_syncing) {
            if (now >= s_sync_deadline_us) {
                s_syncing = false;
                wiimote_hw_set_leds(s_leds);
                /* The backend closes the window and calls wiimote_note_sync_ended(). */
                wiimote_bt_listen_only();
            } else if (!s_connected && now - s_blink_us >= 250000) {
                s_blink_us = now;
                s_blink_lit = !s_blink_lit;
                wiimote_hw_blink(led_count_for_battery(s_battery), s_blink_lit);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void wiimote_lock(void)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

void wiimote_unlock(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

wiimote_proto_t *wiimote_state(void)
{
    return s_proto;
}

bool wiimote_ready(void)
{
    return s_proto != NULL;
}

void wiimote_note_sync_ended(void)
{
    s_syncing = false;
    if (!s_sync_reported) {
        return;
    }
    s_sync_reported = false;
    if (s_cb.sync_changed) {
        s_cb.sync_changed(false, s_cb.user);
    }
}

void wiimote_set_connected(bool up)
{
    bool was = s_connected;
    s_connected = up;
    if (up && s_syncing) {
        wiimote_note_sync_ended();
    }
    wiimote_lock();
    if (s_proto != NULL) {
        s_proto->link_up = up;
        if (!up) {
            s_proto->q_count = 0;
            s_proto->q_head = 0;
            s_proto->read.active = false;
            s_proto->reporting = false;
            s_proto->reporting_suspended = false;
        }
    }
    wiimote_unlock();
    if (was == up) {
        return;
    }
    if (!up) {
        wiimote_hw_set_leds(0);
        wiimote_hw_set_rumble(false);
        s_leds = 0;
    }
    if (up && s_cb.connected) {
        s_cb.connected(s_cb.user);
    }
    if (!up && s_cb.disconnected) {
        s_cb.disconnected(s_cb.user);
    }
}

bool wiimote_connected_flag(void)
{
    return s_connected;
}

uint8_t wiimote_current_battery(void)
{
    uint8_t level = s_battery;
    if (s_cb.battery) {
        s_cb.battery(&level, s_cb.user);
        s_battery = level;
    }
    return level;
}

void wiimote_note_leds(uint8_t mask)
{
    s_leds = mask;
    if (!s_syncing || s_connected) {
        wiimote_hw_set_leds(mask);
    }
    if (s_cb.leds) {
        s_cb.leds(mask, s_cb.user);
    }
}

void wiimote_note_rumble(bool on)
{
    wiimote_hw_set_rumble(on);
    if (s_cb.rumble) {
        s_cb.rumble(on, s_cb.user);
    }
}

void wiimote_pins_init(wiimote_pins_t *pins)
{
    int i;
    if (pins == NULL) {
        return;
    }
    memset(pins, 0, sizeof(*pins));
    for (i = 0; i < WIIMOTE_BUTTON_COUNT; i++) {
        pins->button_gpio[i] = -1;
    }
    pins->buttons_active_low = true;
    for (i = 0; i < WIIMOTE_AXIS_COUNT; i++) {
        pins->axis[i].gpio = -1;
        pins->axis[i].min = 0;
        pins->axis[i].max = 4095;
        pins->axis[i].center = 2048;
        pins->axis[i].deadzone = 200;
    }
    pins->rumble_gpio = -1;
    for (i = 0; i < 4; i++) {
        pins->led_gpio[i] = -1;
    }
}

esp_err_t wiimote_init(const wiimote_config_t *config)
{
    esp_err_t err;
    uint8_t battery;
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_proto != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_proto = (wiimote_proto_t *)calloc(1, sizeof(*s_proto));
    if (s_proto == NULL) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    battery = config->battery ? config->battery : 0xc0;
    s_battery = battery;
    if (config->callbacks) {
        s_cb = *config->callbacks;
    }
    wiimote_proto_init(s_proto, config->extension, battery);
    wiimote_hw_init(config->pins);
    wiimote_bt_set_external_stack(config->external_stack);
    err = wiimote_bt_start();
    if (err != ESP_OK) {
        /* Nothing runs yet: release the state so wiimote_init() can retry. */
        ESP_LOGE(TAG, "Bluetooth start failed: %s", esp_err_to_name(err));
        free(s_proto);
        s_proto = NULL;
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }
    if (xTaskCreate(wiimote_task, "wiimote", 6144, NULL, 5, &s_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Wiimote stack ready");
    return ESP_OK;
}

esp_err_t wiimote_sync(void)
{
    esp_err_t err;
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_blink_us = esp_timer_get_time();
    s_blink_lit = true;
    s_sync_deadline_us = s_blink_us + WIIMOTE_SYNC_WINDOW_MS * 1000LL;
    s_syncing = true;
    wiimote_hw_blink(led_count_for_battery(s_battery), true);
    err = wiimote_bt_sync();
    if (err != ESP_OK) {
        s_syncing = false;
        return err;
    }
    if (!s_sync_reported) {
        s_sync_reported = true;
        if (s_cb.sync_changed) {
            s_cb.sync_changed(true, s_cb.user);
        }
    }
    return ESP_OK;
}

bool wiimote_is_syncing(void)
{
    return s_syncing;
}

esp_err_t wiimote_connect(void)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_connected) {
        return ESP_OK;
    }
    return wiimote_bt_connect();
}

esp_err_t wiimote_disconnect(void)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return wiimote_bt_disconnect();
}

bool wiimote_has_paired_host(void)
{
    return s_proto != NULL && wiimote_bt_has_host();
}

void wiimote_send_now(void)
{
    if (s_proto != NULL) {
        wiimote_bt_send_now();
    }
}

bool wiimote_is_connecting(void)
{
    return s_proto != NULL && wiimote_bt_is_connecting();
}

bool wiimote_link_is_master(void)
{
    return s_proto != NULL && wiimote_bt_link_is_master();
}

uint8_t wiimote_last_disconnect_reason(void)
{
    return s_proto != NULL ? wiimote_bt_last_disconnect_reason() : 0;
}

esp_err_t wiimote_forget_host(void)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return wiimote_bt_forget_host();
}

esp_err_t wiimote_stop(void)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_syncing = false;
    wiimote_hw_set_leds(0);
    wiimote_hw_set_rumble(false);
    /* The backend reports the end of any open sync window. */
    return wiimote_bt_stop();
}

esp_err_t wiimote_set_extension(wiimote_extension_t extension)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_extension(s_proto, extension);
    wiimote_unlock();
    if (s_cb.extension_changed) {
        s_cb.extension_changed(extension, s_cb.user);
    }
    return ESP_OK;
}

esp_err_t wiimote_set_button(wiimote_button_t button, bool pressed)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (button >= WIIMOTE_BUTTON_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    wiimote_lock();
    wiimote_proto_set_button(s_proto, button, pressed);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_axis(wiimote_axis_t axis, float value)
{
    int raw;
    if (s_proto == NULL || axis >= WIIMOTE_AXIS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (axis) {
    case WIIMOTE_AXIS_NUNCHUK_X:
    case WIIMOTE_AXIS_NUNCHUK_Y:
        raw = stick_from_unit(value, 0, 255, 128);
        break;
    case WIIMOTE_AXIS_CLASSIC_LX:
    case WIIMOTE_AXIS_CLASSIC_LY:
        raw = stick_from_unit(value, 0, 63, 32);
        break;
    case WIIMOTE_AXIS_CLASSIC_RX:
    case WIIMOTE_AXIS_CLASSIC_RY:
        raw = stick_from_unit(value, 0, 31, 16);
        break;
    case WIIMOTE_AXIS_CLASSIC_LT:
    case WIIMOTE_AXIS_CLASSIC_RT:
        if (value < 0.f) value = 0.f;
        if (value > 1.f) value = 1.f;
        raw = (int)(value * 31.f + 0.5f);
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    wiimote_lock();
    wiimote_proto_set_axis_raw(s_proto, axis, raw);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_nunchuk_stick(uint8_t x, uint8_t y)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_axis_raw(s_proto, WIIMOTE_AXIS_NUNCHUK_X, x);
    wiimote_proto_set_axis_raw(s_proto, WIIMOTE_AXIS_NUNCHUK_Y, y);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_accel(int16_t x, int16_t y, int16_t z)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_accel(s_proto, (uint16_t)x, (uint16_t)y, (uint16_t)z);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_nunchuk_accel(int16_t x, int16_t y, int16_t z)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_nunchuk_accel(s_proto, (uint16_t)x, (uint16_t)y, (uint16_t)z);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_gyro(float pitch_dps, float roll_dps, float yaw_dps)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_gyro(s_proto, (int16_t)pitch_dps, (int16_t)roll_dps, (int16_t)yaw_dps);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_ir_dot(int index, int x, int y, int size)
{
    if (s_proto == NULL || index < 0 || index > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    wiimote_lock();
    wiimote_proto_set_ir_dot(s_proto, index, x, y, size);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_ir_raw(const uint8_t *data, size_t len)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wiimote_lock();
    wiimote_proto_set_ir_raw(s_proto, data, len);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_battery(uint8_t level)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_battery = level;
    wiimote_lock();
    wiimote_proto_set_battery(s_proto, level);
    wiimote_unlock();
    return ESP_OK;
}

esp_err_t wiimote_set_motion_callback(wiimote_motion_cb_t cb, void *user)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_motion = cb;
    s_motion_user = user;
    return ESP_OK;
}

esp_err_t wiimote_set_imu_map(const int8_t map[3])
{
    if (map == NULL) {
        s_map[0] = 1;
        s_map[1] = 2;
        s_map[2] = 3;
        return ESP_OK;
    }
    s_map[0] = map[0];
    s_map[1] = map[1];
    s_map[2] = map[2];
    return ESP_OK;
}

static esp_err_t open_imu(wiimote_imu_kind_t kind, int sda, int scl, uint8_t addr)
{
    if (s_proto == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return wiimote_imu_open(kind, sda, scl, addr);
}

esp_err_t wiimote_use_mpu6050(int sda, int scl, uint8_t addr)
{
    return open_imu(WIIMOTE_IMU_MPU6050, sda, scl, addr);
}

esp_err_t wiimote_use_mpu6500(int sda, int scl, uint8_t addr)
{
    return open_imu(WIIMOTE_IMU_MPU6500, sda, scl, addr);
}

esp_err_t wiimote_use_lsm6ds3(int sda, int scl, uint8_t addr)
{
    return open_imu(WIIMOTE_IMU_LSM6, sda, scl, addr);
}

bool wiimote_is_connected(void)
{
    return s_connected;
}

uint8_t wiimote_led_mask(void)
{
    return s_leds;
}
