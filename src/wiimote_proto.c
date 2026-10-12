/* SPDX-License-Identifier: MIT */
#include "wiimote_proto.h"

#include <string.h>

static bool core_down(const wiimote_proto_t *proto, wiimote_button_t button)
{
    return (proto->core_pressed & (1u << button)) != 0;
}

static bool classic_button(wiimote_button_t button)
{
    return button >= WIIMOTE_BUTTON_CLASSIC_A && button <= WIIMOTE_BUTTON_CLASSIC_RIGHT;
}

static void queue_packet(wiimote_proto_t *proto, const uint8_t *data, uint8_t len)
{
    int slot;
    if (len > WIIMOTE_PKT_MAX || proto->q_count >= (int)(sizeof(proto->queue) / sizeof(proto->queue[0]))) {
        return;
    }
    slot = (proto->q_head + proto->q_count) % (int)(sizeof(proto->queue) / sizeof(proto->queue[0]));
    memcpy(proto->queue[slot].data, data, len);
    proto->queue[slot].len = len;
    proto->q_count++;
}

static int dequeue_packet(wiimote_proto_t *proto, uint8_t *out, size_t cap)
{
    int n;
    if (proto->q_count == 0) {
        return 0;
    }
    n = proto->queue[proto->q_head].len;
    if ((size_t)n > cap) {
        return 0;
    }
    memcpy(out, proto->queue[proto->q_head].data, (size_t)n);
    proto->q_head = (proto->q_head + 1) % (int)(sizeof(proto->queue) / sizeof(proto->queue[0]));
    proto->q_count--;
    return n;
}

static void pack_buttons(const wiimote_proto_t *proto, uint8_t bb[2], uint8_t mode, bool accel_lsb)
{
    bb[0] = 0;
    bb[1] = 0;
    if (core_down(proto, WIIMOTE_BUTTON_LEFT)) bb[0] |= 0x01;
    if (core_down(proto, WIIMOTE_BUTTON_RIGHT)) bb[0] |= 0x02;
    if (core_down(proto, WIIMOTE_BUTTON_DOWN)) bb[0] |= 0x04;
    if (core_down(proto, WIIMOTE_BUTTON_UP)) bb[0] |= 0x08;
    if (core_down(proto, WIIMOTE_BUTTON_PLUS)) bb[0] |= 0x10;
    if (core_down(proto, WIIMOTE_BUTTON_TWO)) bb[1] |= 0x01;
    if (core_down(proto, WIIMOTE_BUTTON_ONE)) bb[1] |= 0x02;
    if (core_down(proto, WIIMOTE_BUTTON_B)) bb[1] |= 0x04;
    if (core_down(proto, WIIMOTE_BUTTON_A)) bb[1] |= 0x08;
    if (core_down(proto, WIIMOTE_BUTTON_MINUS)) bb[1] |= 0x10;
    if (core_down(proto, WIIMOTE_BUTTON_HOME)) bb[1] |= 0x80;
    if (mode == 0x3e) {
        bb[0] |= (uint8_t)(((proto->accel_z >> 4) & 0x03) << 5);
        bb[1] |= (uint8_t)(((proto->accel_z >> 6) & 0x03) << 5);
    } else if (mode == 0x3f) {
        bb[0] |= (uint8_t)((proto->accel_z & 0x03) << 5);
        bb[1] |= (uint8_t)(((proto->accel_z >> 2) & 0x03) << 5);
    } else if (accel_lsb) {
        bb[0] |= (uint8_t)((proto->accel_x & 0x03) << 5);
        bb[1] |= (uint8_t)(((proto->accel_y >> 1) & 0x01) << 5);
        bb[1] |= (uint8_t)(((proto->accel_z >> 1) & 0x01) << 6);
    }
}

static void pack_ir_pair_basic(const wiimote_proto_t *proto, int i0, int i1, uint8_t *out)
{
    const int idx[2] = {i0, i1};
    int x[2], y[2];
    int i;
    for (i = 0; i < 2; i++) {
        if (!proto->ir_dot[idx[i]].visible) {
            x[i] = 0x3ff;
            y[i] = 0x3ff;
        } else {
            x[i] = proto->ir_dot[idx[i]].x;
            y[i] = proto->ir_dot[idx[i]].y;
        }
    }
    out[0] = (uint8_t)x[0];
    out[1] = (uint8_t)y[0];
    out[2] = (uint8_t)(((y[0] >> 8) & 3) << 6 | ((x[0] >> 8) & 3) << 4 |
                       ((y[1] >> 8) & 3) << 2 | ((x[1] >> 8) & 3));
    out[3] = (uint8_t)x[1];
    out[4] = (uint8_t)y[1];
}

static void pack_ir_object(const wiimote_proto_t *proto, int index, uint8_t *out, bool full)
{
    int x, y, size;
    if (!proto->ir_dot[index].visible) {
        memset(out, 0xff, full ? 9 : 3);
        return;
    }
    x = proto->ir_dot[index].x;
    y = proto->ir_dot[index].y;
    size = proto->ir_dot[index].size;
    if (size < 0) size = 0;
    if (size > 15) size = 15;
    out[0] = (uint8_t)x;
    out[1] = (uint8_t)y;
    out[2] = (uint8_t)((((y >> 8) & 3) << 6) | (((x >> 8) & 3) << 4) | (size & 0x0f));
    if (!full) {
        return;
    }
    /* Bounding box is synthesized from the dot so full-mode games still see an object. */
    out[3] = (uint8_t)((x >> 3) & 0x7f);
    out[4] = (uint8_t)((y >> 3) & 0x7f);
    out[5] = (uint8_t)(((x >> 3) + size) & 0x7f);
    out[6] = (uint8_t)(((y >> 3) + size) & 0x7f);
    out[7] = 0;
    out[8] = (uint8_t)(size * 16);
}

static void fill_ir(const wiimote_proto_t *proto, uint8_t mode, uint8_t *dst)
{
    if (proto->ir_raw_len > 0) {
        size_t n = 10;
        if (mode == 0x33) n = 12;
        if (mode == 0x3e || mode == 0x3f) n = 18;
        if (n > proto->ir_raw_len) n = proto->ir_raw_len;
        memcpy(dst, proto->ir_raw, n);
        return;
    }
    if (mode == 0x33) {
        int i;
        for (i = 0; i < 4; i++) {
            pack_ir_object(proto, i, dst + i * 3, false);
        }
    } else if (mode == 0x36 || mode == 0x37) {
        pack_ir_pair_basic(proto, 0, 1, dst);
        pack_ir_pair_basic(proto, 2, 3, dst + 5);
    } else if (mode == 0x3e) {
        pack_ir_object(proto, 0, dst, true);
        pack_ir_object(proto, 1, dst + 9, true);
    } else if (mode == 0x3f) {
        pack_ir_object(proto, 2, dst, true);
        pack_ir_object(proto, 3, dst + 9, true);
    }
}

static int payload_size(uint8_t mode)
{
    switch (mode) {
    case 0x20: return 6;
    case 0x21: return 21;
    case 0x22: return 4;
    case 0x30: return 2;
    case 0x31: return 5;
    case 0x32: return 10;
    case 0x33: return 17;
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x3d:
    case 0x3e:
    case 0x3f: return 21;
    default: return -1;
    }
}

static bool mode_has_accel(uint8_t mode)
{
    return mode == 0x31 || mode == 0x33 || mode == 0x35 || mode == 0x37 || mode == 0x3e || mode == 0x3f;
}

int wiimote_proto_build(wiimote_proto_t *proto, uint8_t mode, uint8_t *out, size_t cap)
{
    int size = payload_size(mode);
    uint8_t *payload;
    bool accel = mode_has_accel(mode);
    if (size < 0 || cap < (size_t)size + 2) {
        return 0;
    }
    memset(out, 0, (size_t)size + 2);
    out[0] = 0xa1;
    out[1] = mode;
    payload = out + 2;
    if (mode != 0x3d) {
        pack_buttons(proto, payload, mode, accel && mode != 0x3e && mode != 0x3f);
    }
    if (mode == 0x31 || mode == 0x33 || mode == 0x35 || mode == 0x37) {
        payload[2] = (uint8_t)(proto->accel_x >> 2);
        payload[3] = (uint8_t)(proto->accel_y >> 2);
        payload[4] = (uint8_t)(proto->accel_z >> 2);
    } else if (mode == 0x3e) {
        payload[2] = (uint8_t)(proto->accel_x >> 2);
    } else if (mode == 0x3f) {
        payload[2] = (uint8_t)(proto->accel_y >> 2);
    }
    if (mode == 0x33) {
        fill_ir(proto, mode, payload + 5);
    } else if (mode == 0x36) {
        fill_ir(proto, mode, payload + 2);
    } else if (mode == 0x37) {
        fill_ir(proto, mode, payload + 5);
    } else if (mode == 0x3e || mode == 0x3f) {
        fill_ir(proto, mode, payload + 3);
    }
    if (mode == 0x32) {
        wiimote_ext_stream(&proto->ext, payload + 2, 8);
    } else if (mode == 0x34) {
        wiimote_ext_stream(&proto->ext, payload + 2, 19);
    } else if (mode == 0x35) {
        wiimote_ext_stream(&proto->ext, payload + 5, 16);
    } else if (mode == 0x36) {
        wiimote_ext_stream(&proto->ext, payload + 12, 9);
    } else if (mode == 0x37) {
        wiimote_ext_stream(&proto->ext, payload + 15, 6);
    } else if (mode == 0x3d) {
        wiimote_ext_stream(&proto->ext, payload, 21);
    }
    return size + 2;
}

static void queue_status(wiimote_proto_t *proto, bool from_extension)
{
    uint8_t pkt[8];
    uint8_t flags = 0;
    pkt[0] = 0xa1;
    pkt[1] = 0x20;
    pack_buttons(proto, pkt + 2, 0x20, false);
    if (proto->battery < 0x20) flags |= 0x01;
    if (wiimote_ext_connected_bit(&proto->ext)) flags |= 0x02;
    if (proto->speaker) flags |= 0x04;
    if (proto->ir_clock && proto->ir_logic) flags |= 0x08;
    flags |= (uint8_t)((proto->leds & 0x0f) << 4);
    pkt[4] = flags;
    pkt[5] = 0;
    pkt[6] = 0;
    pkt[7] = proto->battery;
    queue_packet(proto, pkt, 8);
    if (from_extension) {
        proto->reporting = false;
    }
}

static void queue_ack(wiimote_proto_t *proto, uint8_t report, uint8_t error)
{
    uint8_t pkt[6];
    pkt[0] = 0xa1;
    pkt[1] = 0x22;
    pack_buttons(proto, pkt + 2, 0x22, false);
    pkt[4] = report;
    pkt[5] = error;
    queue_packet(proto, pkt, 6);
}

static void start_read(wiimote_proto_t *proto, uint32_t addr, uint16_t len, bool is_register)
{
    proto->read.active = len > 0;
    proto->read.addr = addr;
    proto->read.left = len;
    proto->read.is_register = is_register;
    proto->read.error = 0;
    proto->reporting_suspended = proto->read.active;
}

static int emit_read(wiimote_proto_t *proto, uint8_t *out, size_t cap)
{
    uint8_t payload[16];
    uint8_t error = 0;
    uint16_t chunk;
    uint16_t addr16;
    if (!proto->read.active) {
        return 0;
    }
    if (cap < 23) {
        return 0;
    }
    memset(payload, 0, sizeof(payload));
    memset(out, 0, 23);
    chunk = proto->read.left > 16 ? 16 : proto->read.left;
    if (proto->read.error) {
        error = proto->read.error;
        chunk = 0;
    } else if (proto->read.is_register) {
        if (wiimote_ext_read(&proto->ext, proto->read.addr, payload, chunk, &error) != 0) {
            chunk = 0;
        }
    } else {
        uint32_t off = proto->read.addr % 0x10000u;
        if (off >= WIIMOTE_EEPROM_SIZE || off + chunk > WIIMOTE_EEPROM_SIZE) {
            error = 8;
            chunk = 0;
        } else {
            memcpy(payload, proto->eeprom + off, chunk);
        }
    }
    out[0] = 0xa1;
    out[1] = 0x21;
    pack_buttons(proto, out + 2, 0x21, false);
    out[4] = (uint8_t)((chunk ? ((chunk - 1) << 4) : 0) | (error & 0x0f));
    addr16 = (uint16_t)(proto->read.addr & 0xffffu);
    out[5] = (uint8_t)(addr16 >> 8);
    out[6] = (uint8_t)addr16;
    if (error == 0) {
        memcpy(out + 7, payload, chunk);
    }
    if (error || proto->read.left <= chunk) {
        proto->read.active = false;
        proto->read.left = 0;
        proto->reporting_suspended = false;
    } else {
        proto->read.left = (uint16_t)(proto->read.left - chunk);
        proto->read.addr += chunk;
    }
    return 23;
}

static void handle_write(wiimote_proto_t *proto, const uint8_t *p, size_t len)
{
    uint32_t addr;
    uint8_t size;
    uint8_t error = 0;
    bool is_reg;
    bool both;
    if (len < 6) {
        return;
    }
    both = (p[0] & 0x04) && (p[0] & 0x08);
    is_reg = (p[0] & 0x0c) != 0;
    addr = ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    size = p[4];
    if (size > 16) {
        size = 16;
    }
    if (len < (size_t)(5 + size)) {
        size = (uint8_t)(len - 5);
    }
    if (both) {
        error = 8;
    } else if (!is_reg) {
        uint32_t off = addr % 0x10000u;
        if (size > 0 && (off >= WIIMOTE_EEPROM_SIZE || off + size > WIIMOTE_EEPROM_SIZE)) {
            error = 8;
        } else if (size > 0) {
            memcpy(proto->eeprom + off, p + 5, size);
        }
    } else {
        wiimote_ext_result_t note;
        int i;
        wiimote_ext_write(&proto->ext, addr, p + 5, size, &error, &note);
        queue_ack(proto, 0x16, error);
        for (i = 0; i < note.status_count; i++) {
            queue_status(proto, true);
        }
        return;
    }
    queue_ack(proto, 0x16, error);
}

static void handle_read(wiimote_proto_t *proto, const uint8_t *p, size_t len, bool want_ack)
{
    uint32_t addr;
    uint16_t size;
    bool is_reg;
    bool both;
    uint8_t error = 0;
    if (len < 6) {
        return;
    }
    both = (p[0] & 0x04) && (p[0] & 0x08);
    is_reg = (p[0] & 0x0c) != 0;
    addr = ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    size = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
    if (both) {
        error = 8;
    } else if (!is_reg) {
        uint32_t off = addr % 0x10000u;
        if (size > 0 && off >= WIIMOTE_EEPROM_SIZE) {
            error = 8;
        }
    }
    if (want_ack) {
        queue_ack(proto, 0x17, error);
    }
    if (error) {
        start_read(proto, addr, 1, is_reg);
        proto->read.error = error;
        return;
    }
    start_read(proto, addr, size, is_reg);
}

static void apply_rumble(wiimote_proto_t *proto, uint8_t first)
{
    proto->rumble = (first & 0x01) != 0;
}

void wiimote_proto_handle_output(wiimote_proto_t *proto, const uint8_t *bytes, size_t len)
{
    uint8_t id;
    const uint8_t *p;
    bool want_ack;
    if (len < 1) {
        return;
    }
    id = bytes[0];
    p = bytes + 1;
    len -= 1;
    if (len > 0) {
        apply_rumble(proto, p[0]);
    }
    want_ack = len > 0 && (p[0] & 0x02) != 0;
    switch (id) {
    case 0x10:
        break;
    case 0x11:
        if (len >= 1) {
            proto->leds = (uint8_t)((p[0] >> 4) & 0x0f);
        }
        break;
    case 0x12:
        if (len >= 2) {
            uint8_t mode = p[1];
            proto->continuous = (p[0] & 0x04) != 0;
            if (payload_size(mode) < 0 || mode < 0x30) {
                if (want_ack) queue_ack(proto, 0x12, 5);
            } else {
                proto->mode = mode;
                proto->reporting = true;
                proto->have_last_payload = false;
                if (want_ack) queue_ack(proto, 0x12, 0);
            }
        }
        break;
    case 0x13:
        if (len >= 1) proto->ir_clock = (p[0] & 0x04) != 0;
        break;
    case 0x14:
        if (len >= 1) proto->speaker = (p[0] & 0x04) != 0;
        break;
    case 0x15:
        queue_status(proto, false);
        break;
    case 0x16:
        handle_write(proto, p, len);
        return;
    case 0x17:
        handle_read(proto, p, len, want_ack);
        return;
    case 0x18:
    case 0x19:
        break;
    case 0x1a:
        if (len >= 1) proto->ir_logic = (p[0] & 0x04) != 0;
        break;
    default:
        if (want_ack) queue_ack(proto, id, 3);
        return;
    }
    if (want_ack && id != 0x12) {
        queue_ack(proto, id, 0);
    }
}

static void seed_eeprom(uint8_t *eeprom)
{
    static const uint8_t ir[11] = {
        0xa1, 0xaa, 0x8b, 0x99, 0xae, 0x9e, 0x78, 0x30, 0xa7, 0x74, 0xd3
    };
    static const uint8_t accel[10] = {
        0x80, 0x80, 0x80, 0x00, 0x9a, 0x9a, 0x9a, 0x2a, 0x40, 0x0d
    };
    memset(eeprom, 0, WIIMOTE_EEPROM_SIZE);
    memcpy(eeprom + 0x00, ir, sizeof(ir));
    memcpy(eeprom + 0x0b, ir, sizeof(ir));
    memcpy(eeprom + 0x16, accel, sizeof(accel));
    memcpy(eeprom + 0x20, accel, sizeof(accel));
}

void wiimote_proto_init(wiimote_proto_t *proto, wiimote_extension_t extension, uint8_t battery)
{
    memset(proto, 0, sizeof(*proto));
    seed_eeprom(proto->eeprom);
    wiimote_ext_init(&proto->ext, extension);
    proto->battery = battery;
    proto->accel_x = 0x200;
    proto->accel_y = 0x200;
    proto->accel_z = 0x26a;
    proto->mode = 0x30;
    proto->reporting = false;
}

void wiimote_proto_set_button(wiimote_proto_t *proto, wiimote_button_t button, bool pressed)
{
    bool changed = false;
    if (button >= WIIMOTE_BUTTON_COUNT) {
        return;
    }
    if (button == WIIMOTE_BUTTON_NUNCHUK_C) {
        changed = (proto->ext.nunchuk_c != pressed);
        proto->ext.nunchuk_c = pressed;
    } else if (button == WIIMOTE_BUTTON_NUNCHUK_Z) {
        changed = (proto->ext.nunchuk_z != pressed);
        proto->ext.nunchuk_z = pressed;
    } else if (classic_button(button)) {
        uint32_t prev = proto->ext.classic_pressed;
        if (pressed) proto->ext.classic_pressed |= (1u << button);
        else proto->ext.classic_pressed &= ~(1u << button);
        changed = (proto->ext.classic_pressed != prev);
    } else {
        uint32_t prev = proto->core_pressed;
        if (pressed) proto->core_pressed |= (1u << button);
        else proto->core_pressed &= ~(1u << button);
        changed = (proto->core_pressed != prev);
    }
    if (changed) {
        proto->have_last_payload = false;
    }
}

static uint8_t clamp_u8(int value, int lo, int hi)
{
    if (value < lo) return (uint8_t)lo;
    if (value > hi) return (uint8_t)hi;
    return (uint8_t)value;
}

void wiimote_proto_set_axis_raw(wiimote_proto_t *proto, wiimote_axis_t axis, int value)
{
    uint8_t clamped;
    bool changed = false;
    switch (axis) {
    case WIIMOTE_AXIS_NUNCHUK_X:
        clamped = clamp_u8(value, 0, 255);
        changed = (proto->ext.nunchuk_x != clamped);
        proto->ext.nunchuk_x = clamped;
        break;
    case WIIMOTE_AXIS_NUNCHUK_Y:
        clamped = clamp_u8(value, 0, 255);
        changed = (proto->ext.nunchuk_y != clamped);
        proto->ext.nunchuk_y = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_LX:
        clamped = clamp_u8(value, 0, 63);
        changed = (proto->ext.classic_lx != clamped);
        proto->ext.classic_lx = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_LY:
        clamped = clamp_u8(value, 0, 63);
        changed = (proto->ext.classic_ly != clamped);
        proto->ext.classic_ly = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_RX:
        clamped = clamp_u8(value, 0, 31);
        changed = (proto->ext.classic_rx != clamped);
        proto->ext.classic_rx = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_RY:
        clamped = clamp_u8(value, 0, 31);
        changed = (proto->ext.classic_ry != clamped);
        proto->ext.classic_ry = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_LT:
        clamped = clamp_u8(value, 0, 31);
        changed = (proto->ext.classic_lt != clamped);
        proto->ext.classic_lt = clamped;
        break;
    case WIIMOTE_AXIS_CLASSIC_RT:
        clamped = clamp_u8(value, 0, 31);
        changed = (proto->ext.classic_rt != clamped);
        proto->ext.classic_rt = clamped;
        break;
    default:
        break;
    }
    if (changed) {
        proto->have_last_payload = false;
    }
}

void wiimote_proto_set_accel(wiimote_proto_t *proto, uint16_t x, uint16_t y, uint16_t z)
{
    uint16_t cx = x > 1023 ? 1023 : x;
    uint16_t cy = y > 1023 ? 1023 : y;
    uint16_t cz = z > 1023 ? 1023 : z;
    if (proto->accel_x != cx || proto->accel_y != cy || proto->accel_z != cz) {
        proto->accel_x = cx;
        proto->accel_y = cy;
        proto->accel_z = cz;
        proto->have_last_payload = false;
    }
}

void wiimote_proto_set_nunchuk_accel(wiimote_proto_t *proto, uint16_t x, uint16_t y, uint16_t z)
{
    uint16_t cx = x > 1023 ? 1023 : x;
    uint16_t cy = y > 1023 ? 1023 : y;
    uint16_t cz = z > 1023 ? 1023 : z;
    if (proto->ext.nunchuk_ax != cx || proto->ext.nunchuk_ay != cy || proto->ext.nunchuk_az != cz) {
        proto->ext.nunchuk_ax = cx;
        proto->ext.nunchuk_ay = cy;
        proto->ext.nunchuk_az = cz;
        proto->have_last_payload = false;
    }
}

void wiimote_proto_set_gyro(wiimote_proto_t *proto, int16_t pitch, int16_t roll, int16_t yaw)
{
    if (proto->ext.pitch_dps != pitch || proto->ext.roll_dps != roll || proto->ext.yaw_dps != yaw) {
        proto->ext.pitch_dps = pitch;
        proto->ext.roll_dps = roll;
        proto->ext.yaw_dps = yaw;
        proto->have_last_payload = false;
    }
}

void wiimote_proto_set_ir_dot(wiimote_proto_t *proto, int index, int x, int y, int size)
{
    if (index < 0 || index > 3) {
        return;
    }
    proto->ir_raw_len = 0;
    proto->ir_dot[index].visible = size >= 0;
    proto->ir_dot[index].x = x < 0 ? 0 : (x > 1023 ? 1023 : x);
    proto->ir_dot[index].y = y < 0 ? 0 : (y > 767 ? 767 : y);
    proto->ir_dot[index].size = size < 0 ? 0 : (size > 15 ? 15 : size);
    proto->have_last_payload = false;
}

void wiimote_proto_set_ir_raw(wiimote_proto_t *proto, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        proto->ir_raw_len = 0;
        return;
    }
    if (len > sizeof(proto->ir_raw)) {
        len = sizeof(proto->ir_raw);
    }
    memcpy(proto->ir_raw, data, len);
    proto->ir_raw_len = (uint8_t)len;
    proto->have_last_payload = false;
}

void wiimote_proto_set_battery(wiimote_proto_t *proto, uint8_t level)
{
    proto->battery = level;
}

void wiimote_proto_set_extension(wiimote_proto_t *proto, wiimote_extension_t extension)
{
    bool was = wiimote_ext_connected_bit(&proto->ext);
    uint8_t nx = proto->ext.nunchuk_x;
    uint8_t ny = proto->ext.nunchuk_y;
    uint16_t ax = proto->ext.nunchuk_ax, ay = proto->ext.nunchuk_ay, az = proto->ext.nunchuk_az;
    bool c = proto->ext.nunchuk_c, z = proto->ext.nunchuk_z;
    uint32_t classic = proto->ext.classic_pressed;
    uint8_t lx = proto->ext.classic_lx, ly = proto->ext.classic_ly;
    uint8_t rx = proto->ext.classic_rx, ry = proto->ext.classic_ry;
    uint8_t lt = proto->ext.classic_lt, rt = proto->ext.classic_rt;
    int16_t pitch = proto->ext.pitch_dps, roll = proto->ext.roll_dps, yaw = proto->ext.yaw_dps;
    wiimote_ext_init(&proto->ext, extension);
    proto->ext.nunchuk_x = nx;
    proto->ext.nunchuk_y = ny;
    proto->ext.nunchuk_ax = ax;
    proto->ext.nunchuk_ay = ay;
    proto->ext.nunchuk_az = az;
    proto->ext.nunchuk_c = c;
    proto->ext.nunchuk_z = z;
    proto->ext.classic_pressed = classic;
    proto->ext.classic_lx = lx;
    proto->ext.classic_ly = ly;
    proto->ext.classic_rx = rx;
    proto->ext.classic_ry = ry;
    proto->ext.classic_lt = lt;
    proto->ext.classic_rt = rt;
    proto->ext.pitch_dps = pitch;
    proto->ext.roll_dps = roll;
    proto->ext.yaw_dps = yaw;
    if (proto->link_up && was != wiimote_ext_connected_bit(&proto->ext)) {
        queue_status(proto, true);
    }
}

static uint32_t report_period_ms(const wiimote_proto_t *proto)
{
    if (proto->ext.mp_active && proto->ext.mp_passthrough) {
        return 5;
    }
    return 10;
}

static uint8_t effective_mode(const wiimote_proto_t *proto)
{
    if (proto->mode == 0x3e || proto->mode == 0x3f) {
        return (proto->ir_phase & 1) ? 0x3f : 0x3e;
    }
    return proto->mode;
}

int wiimote_proto_poll(wiimote_proto_t *proto, uint8_t *out, size_t cap, uint32_t now_ms)
{
    int n = dequeue_packet(proto, out, cap);
    uint8_t built[WIIMOTE_PKT_MAX];
    int built_len;
    uint8_t mode;
    if (n > 0) {
        return n;
    }
    if (proto->read.active) {
        return emit_read(proto, out, cap);
    }
    if (!proto->reporting || proto->reporting_suspended) {
        return 0;
    }
    if (proto->have_report_time && (uint32_t)(now_ms - proto->last_report_ms) < report_period_ms(proto)) {
        return 0;
    }
    mode = effective_mode(proto);
    built_len = wiimote_proto_build(proto, mode, built, sizeof(built));
    if (built_len <= 0) {
        return 0;
    }
    proto->last_report_ms = now_ms;
    proto->have_report_time = true;
    if (!proto->continuous && proto->have_last_payload &&
        proto->last_payload_len == (uint8_t)(built_len - 2) &&
        memcmp(proto->last_payload, built + 2, (size_t)(built_len - 2)) == 0 &&
        mode != 0x3e && mode != 0x3f) {
        return 0;
    }
    if ((size_t)built_len > cap) {
        return 0;
    }
    memcpy(out, built, (size_t)built_len);
    memcpy(proto->last_payload, built + 2, (size_t)(built_len - 2));
    proto->last_payload_len = (uint8_t)(built_len - 2);
    proto->have_last_payload = true;
    if (proto->mode == 0x3e || proto->mode == 0x3f) {
        proto->ir_phase++;
    }
    return built_len;
}
