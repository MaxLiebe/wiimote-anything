/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_IMU_H
#define WIIMOTE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIIMOTE_IMU_NONE = 0,
    WIIMOTE_IMU_MPU6050,
    WIIMOTE_IMU_MPU6500,
    WIIMOTE_IMU_LSM6
} wiimote_imu_kind_t;

esp_err_t wiimote_imu_open(wiimote_imu_kind_t kind, int sda, int scl, uint8_t addr);
bool wiimote_imu_read(int16_t *ax_mg, int16_t *ay_mg, int16_t *az_mg,
                      int16_t *gx_dps, int16_t *gy_dps, int16_t *gz_dps);

#ifdef __cplusplus
}
#endif

#endif
