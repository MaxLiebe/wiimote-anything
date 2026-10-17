/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_EXT_H
#define WIIMOTE_EXT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wiimote_crypto.h"
#include "../include/wiimote_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    wiimote_extension_t type;
    uint8_t a4[256];
    uint8_t a6[256];
    uint8_t ir[256];
    uint8_t speaker[256];
    bool mp_present;
    bool mp_active;
    bool mp_passthrough;
    bool a4_visible;
    wiimote_crypto_t crypto;
    bool crypto_armed; /* register 0xF0 == 0xAA */
    bool crypto_dirty; /* key registers changed since the pads were derived */
    uint8_t nunchuk_x;
    uint8_t nunchuk_y;
    uint16_t nunchuk_ax;
    uint16_t nunchuk_ay;
    uint16_t nunchuk_az;
    bool nunchuk_c;
    bool nunchuk_z;
    uint8_t classic_lx;
    uint8_t classic_ly;
    uint8_t classic_rx;
    uint8_t classic_ry;
    uint8_t classic_lt;
    uint8_t classic_rt;
    uint32_t classic_pressed;
    int16_t pitch_dps;
    int16_t roll_dps;
    int16_t yaw_dps;
    uint8_t mp_phase;
} wiimote_ext_t;

typedef struct {
    int status_count;
    uint8_t plugged[2];
} wiimote_ext_result_t;

void wiimote_ext_init(wiimote_ext_t *ext, wiimote_extension_t type);
bool wiimote_ext_connected_bit(const wiimote_ext_t *ext);

/** 0 on success. error is 0, 7, or 8. result collects plug/unplug status edges. */
int wiimote_ext_read(wiimote_ext_t *ext, uint32_t addr, uint8_t *dst, size_t len, uint8_t *error);
int wiimote_ext_write(wiimote_ext_t *ext, uint32_t addr, const uint8_t *src, size_t len,
                      uint8_t *error, wiimote_ext_result_t *result);

/** Copy len bytes of the current extension stream (already encrypted when required). */
void wiimote_ext_stream(wiimote_ext_t *ext, uint8_t *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif
