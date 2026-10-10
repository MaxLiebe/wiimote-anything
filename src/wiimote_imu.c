/* SPDX-License-Identifier: MIT */
#include "wiimote_imu.h"

#include "wiimote_target.h"

#include <stddef.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "wiimote";

static wiimote_imu_kind_t s_kind = WIIMOTE_IMU_NONE;
static uint8_t s_addr;

static esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_NUM_0, addr, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_write_read_device(I2C_NUM_0, addr, &reg, 1, dst, len, pdMS_TO_TICKS(50));
}

static esp_err_t bus_init(int sda, int scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return err;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool who_is(uint8_t addr, uint8_t reg, uint8_t a, uint8_t b)
{
    uint8_t who = 0;
    if (i2c_read(addr, reg, &who, 1) != ESP_OK) {
        return false;
    }
    return who == a || who == b;
}

static esp_err_t probe(wiimote_imu_kind_t kind, uint8_t addr)
{
    if (kind == WIIMOTE_IMU_LSM6) {
        if (!who_is(addr, 0x0f, 0x69, 0x6c)) {
            return ESP_ERR_NOT_FOUND;
        }
        /* IF_INC + BDU, ±2 g at 416 Hz, ±2000 dps. */
        if (i2c_write(addr, 0x12, 0x44) != ESP_OK ||
            i2c_write(addr, 0x10, 0x60) != ESP_OK ||
            i2c_write(addr, 0x11, 0x6c) != ESP_OK) {
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    {
        uint8_t expect_a = (kind == WIIMOTE_IMU_MPU6050) ? 0x68 : 0x70;
        uint8_t expect_b = (kind == WIIMOTE_IMU_MPU6050) ? 0x68 : 0x71;
        if (!who_is(addr, 0x75, expect_a, expect_b)) {
            return ESP_ERR_NOT_FOUND;
        }
        if (i2c_write(addr, 0x6b, 0x00) != ESP_OK ||
            i2c_write(addr, 0x1c, 0x00) != ESP_OK ||
            i2c_write(addr, 0x1b, 0x18) != ESP_OK) {
            return ESP_FAIL;
        }
        return ESP_OK;
    }
}

esp_err_t wiimote_imu_open(wiimote_imu_kind_t kind, int sda, int scl, uint8_t addr)
{
    uint8_t first = addr;
    uint8_t second = 0;
    esp_err_t err = bus_init(sda, scl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }
    if (addr == 0) {
        first = (kind == WIIMOTE_IMU_LSM6) ? 0x6a : 0x68;
        second = (kind == WIIMOTE_IMU_LSM6) ? 0x6b : 0x69;
    }
    err = probe(kind, first);
    if (err != ESP_OK && second != 0) {
        err = probe(kind, second);
        if (err == ESP_OK) {
            first = second;
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IMU not found; motion reports stay at idle");
        s_kind = WIIMOTE_IMU_NONE;
        return err;
    }
    s_addr = first;
    s_kind = kind;
    ESP_LOGI(TAG, "IMU ready at 0x%02x", s_addr);
    return ESP_OK;
}

bool wiimote_imu_read(int16_t *ax_mg, int16_t *ay_mg, int16_t *az_mg,
                      int16_t *gx_dps, int16_t *gy_dps, int16_t *gz_dps)
{
    uint8_t raw[14];
    int16_t ax, ay, az, gx, gy, gz;
    if (s_kind == WIIMOTE_IMU_NONE) {
        return false;
    }
    if (s_kind == WIIMOTE_IMU_LSM6) {
        if (i2c_read(s_addr, 0x22, raw, 12) != ESP_OK) {
            return false;
        }
        gx = le16(raw + 0);
        gy = le16(raw + 2);
        gz = le16(raw + 4);
        ax = le16(raw + 6);
        ay = le16(raw + 8);
        az = le16(raw + 10);
        *ax_mg = (int16_t)((ax * 61) / 1000);
        *ay_mg = (int16_t)((ay * 61) / 1000);
        *az_mg = (int16_t)((az * 61) / 1000);
        *gx_dps = (int16_t)((gx * 70) / 1000);
        *gy_dps = (int16_t)((gy * 70) / 1000);
        *gz_dps = (int16_t)((gz * 70) / 1000);
        return true;
    }
    if (i2c_read(s_addr, 0x3b, raw, 14) != ESP_OK) {
        return false;
    }
    ax = (int16_t)((raw[0] << 8) | raw[1]);
    ay = (int16_t)((raw[2] << 8) | raw[3]);
    az = (int16_t)((raw[4] << 8) | raw[5]);
    gx = (int16_t)((raw[8] << 8) | raw[9]);
    gy = (int16_t)((raw[10] << 8) | raw[11]);
    gz = (int16_t)((raw[12] << 8) | raw[13]);
    *ax_mg = (int16_t)((ax * 1000) / 16384);
    *ay_mg = (int16_t)((ay * 1000) / 16384);
    *az_mg = (int16_t)((az * 1000) / 16384);
    *gx_dps = (int16_t)((gx * 10) / 164);
    *gy_dps = (int16_t)((gy * 10) / 164);
    *gz_dps = (int16_t)((gz * 10) / 164);
    return true;
}
