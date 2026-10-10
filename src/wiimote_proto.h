/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_PROTO_H
#define WIIMOTE_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wiimote_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIIMOTE_EEPROM_SIZE 0x1700
#define WIIMOTE_PKT_MAX 23

typedef struct {
    uint8_t eeprom[WIIMOTE_EEPROM_SIZE];
    wiimote_ext_t ext;
    uint32_t core_pressed;
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint8_t battery;
    uint8_t leds; /* bit 0 = LED 1 */
    bool rumble;
    bool speaker;
    bool ir_clock;
    bool ir_logic;
    bool reporting;
    bool continuous;
    bool reporting_suspended;
    bool link_up;
    uint8_t mode;
    uint8_t ir_phase;
    struct {
        int x;
        int y;
        int size;
        bool visible;
    } ir_dot[4];
    uint8_t ir_raw[36];
    uint8_t ir_raw_len;
    struct {
        bool active;
        uint32_t addr;
        uint16_t left;
        bool is_register;
        uint8_t error;
    } read;
    struct {
        uint8_t len;
        uint8_t data[WIIMOTE_PKT_MAX];
    } queue[12];
    int q_head;
    int q_count;
    uint32_t last_report_ms;
    bool have_report_time;
    uint8_t last_payload[21];
    uint8_t last_payload_len;
    bool have_last_payload;
} wiimote_proto_t;

void wiimote_proto_init(wiimote_proto_t *proto, wiimote_extension_t extension, uint8_t battery);
void wiimote_proto_set_button(wiimote_proto_t *proto, wiimote_button_t button, bool pressed);
void wiimote_proto_set_axis_raw(wiimote_proto_t *proto, wiimote_axis_t axis, int value);
void wiimote_proto_set_accel(wiimote_proto_t *proto, uint16_t x, uint16_t y, uint16_t z);
void wiimote_proto_set_nunchuk_accel(wiimote_proto_t *proto, uint16_t x, uint16_t y, uint16_t z);
void wiimote_proto_set_gyro(wiimote_proto_t *proto, int16_t pitch, int16_t roll, int16_t yaw);
void wiimote_proto_set_ir_dot(wiimote_proto_t *proto, int index, int x, int y, int size);
void wiimote_proto_set_ir_raw(wiimote_proto_t *proto, const uint8_t *data, size_t len);
void wiimote_proto_set_battery(wiimote_proto_t *proto, uint8_t level);
void wiimote_proto_set_extension(wiimote_proto_t *proto, wiimote_extension_t extension);

/** bytes begin with the report id (no 0xA2). */
void wiimote_proto_handle_output(wiimote_proto_t *proto, const uint8_t *bytes, size_t len);

/**
 * Next input packet including the 0xA1 header, or 0 when nothing is due.
 * now_ms is a free-running millisecond clock.
 */
int wiimote_proto_poll(wiimote_proto_t *proto, uint8_t *out, size_t cap, uint32_t now_ms);

/** Build one data report for mode, including 0xA1. Returns length, or 0. */
int wiimote_proto_build(wiimote_proto_t *proto, uint8_t mode, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif
