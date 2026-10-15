/* SPDX-License-Identifier: MIT */
#include "wiimote_ext.h"

#include <string.h>

static void refresh_plaintext(wiimote_ext_t *ext);

static const uint8_t k_id_nunchuk[6] = {0x00, 0x00, 0xa4, 0x20, 0x00, 0x00};
static const uint8_t k_id_classic[6] = {0x00, 0x00, 0xa4, 0x20, 0x01, 0x01};
static const uint8_t k_id_classic_pro[6] = {0x01, 0x00, 0xa4, 0x20, 0x01, 0x01};
static const uint8_t k_id_mp_solo[6] = {0x00, 0x00, 0xa4, 0x20, 0x04, 0x05};
static const uint8_t k_id_mp_nunchuk[6] = {0x00, 0x00, 0xa4, 0x20, 0x05, 0x05};
static const uint8_t k_id_mp_inactive[6] = {0x00, 0x00, 0xa6, 0x20, 0x00, 0x05};
static const uint8_t k_id_mp_off_solo[6] = {0x00, 0x00, 0xa6, 0x20, 0x04, 0x05};
static const uint8_t k_id_mp_off_nunchuk[6] = {0x00, 0x00, 0xa6, 0x20, 0x05, 0x05};

static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xffffffffu;
    size_t i;
    for (i = 0; i < len; i++) {
        int bit;
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return ~crc;
}

static void put_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void seal_ext_cal(uint8_t *block)
{
    size_t i;
    unsigned sum = 0x55;
    for (i = 0; i < 14; i++) {
        sum += block[i];
    }
    block[14] = (uint8_t)sum;
    block[15] = (uint8_t)(sum + 0x55);
}

static void write_nunchuk_cal(uint8_t *reg)
{
    /* 0 g at 512, +1 g at 0x26A, stick max/min/center. Duplicate lives at 0x30. */
    static const uint8_t body[14] = {
        0x80, 0x80, 0x80, 0x00, 0x9a, 0x9a, 0x9a, 0x2a,
        0xe0, 0x1e, 0x80, 0xe0, 0x1e, 0x80
    };
    memcpy(reg + 0x20, body, sizeof(body));
    seal_ext_cal(reg + 0x20);
    memcpy(reg + 0x30, reg + 0x20, 16);
}

static void write_classic_cal(uint8_t *reg)
{
    /* Stick max / min / center in 8-bit scale (6-bit LX/LY << 2, 5-bit RX/RY << 3),
     * trigger rest values, then the 2-byte checksum (sum+0x55, sum+0xaa).
     * The full 0x00..0xff span matches what wiimote_set_axis() produces
     * (0..63 / 0..31), so games scale a full deflection to full scale. */
    static const uint8_t body[14] = {
        0xff, 0x00, 0x80, /* Left Stick X: max, min, center */
        0xff, 0x00, 0x80, /* Left Stick Y: max, min, center */
        0xff, 0x00, 0x80, /* Right Stick X: max, min, center */
        0xff, 0x00, 0x80, /* Right Stick Y: max, min, center */
        0x00, 0x00        /* Left/Right Trigger rest */
    };
    memcpy(reg + 0x20, body, sizeof(body));
    seal_ext_cal(reg + 0x20);
    memcpy(reg + 0x30, reg + 0x20, 16);
}

static void write_mp_cal(uint8_t *reg)
{
    uint8_t block[32];
    uint32_t crc;
    int i;
    memset(block, 0, sizeof(block));
    for (i = 0; i < 6; i++) {
        put_be16(block + (size_t)i * 2, 0x7dfc); /* 8063 << 2, so zero>>2 matches the live rest code */
    }
    put_be16(block + 12, 0xb810);
    put_be16(block + 14, 0xb810);
    put_be16(block + 16, 0xb810);
    put_be16(block + 18, 0x8ac4);
    put_be16(block + 20, 0x8ac4);
    put_be16(block + 22, 0x8ac4);
    put_be16(block + 24, 45); /* degrees/6 ≈ 270 dps slow full-scale reference */
    crc = crc32_ieee(block, 32);
    block[28] = (uint8_t)(crc >> 24);
    block[29] = (uint8_t)(crc >> 16);
    block[30] = (uint8_t)(crc >> 8);
    block[31] = (uint8_t)crc;
    memcpy(reg + 0x20, block, sizeof(block));
}

static void clear_crypto(wiimote_ext_t *ext)
{
    memset(&ext->crypto, 0, sizeof(ext->crypto));
    ext->crypto_armed = false;
    ext->crypto_dirty = false;
}

/* Like a real extension (and Dolphin): reads are encrypted while register
 * 0xF0 holds 0xAA, with pads derived from whatever is in 0x40..0x4F at that
 * moment. The console may write the key and 0xAA in either order. */
static void refresh_crypto(wiimote_ext_t *ext)
{
    if (!ext->crypto_armed) {
        ext->crypto.active = false;
        return;
    }
    if (ext->crypto_dirty || !ext->crypto.active) {
        wiimote_crypto_load_key(&ext->crypto, ext->a4 + 0x40);
        ext->crypto_dirty = false;
    }
}

static bool nunchuk_plugged(const wiimote_ext_t *ext)
{
    return ext->type == WIIMOTE_EXT_NUNCHUK || ext->type == WIIMOTE_EXT_MOTION_PLUS_NUNCHUK;
}

static bool classic_plugged(const wiimote_ext_t *ext)
{
    return ext->type == WIIMOTE_EXT_CLASSIC || ext->type == WIIMOTE_EXT_CLASSIC_PRO;
}

static void apply_visibility(wiimote_ext_t *ext)
{
    if (ext->mp_active) {
        ext->a4_visible = true;
        memcpy(ext->a4 + 0xfa, ext->mp_passthrough ? k_id_mp_nunchuk : k_id_mp_solo, 6);
        write_mp_cal(ext->a4);
        clear_crypto(ext);
        return;
    }
    if (nunchuk_plugged(ext)) {
        ext->a4_visible = true;
        memcpy(ext->a4 + 0xfa, k_id_nunchuk, 6);
        write_nunchuk_cal(ext->a4);
        return;
    }
    if (classic_plugged(ext)) {
        ext->a4_visible = true;
        memcpy(ext->a4 + 0xfa,
               ext->type == WIIMOTE_EXT_CLASSIC_PRO ? k_id_classic_pro : k_id_classic, 6);
        write_classic_cal(ext->a4);
        return;
    }
    ext->a4_visible = false;
}

bool wiimote_ext_connected_bit(const wiimote_ext_t *ext)
{
    if (ext->mp_active) {
        return true;
    }
    return ext->a4_visible;
}

void wiimote_ext_init(wiimote_ext_t *ext, wiimote_extension_t type)
{
    memset(ext, 0, sizeof(*ext));
    ext->type = type;
    ext->mp_present = (type == WIIMOTE_EXT_MOTION_PLUS || type == WIIMOTE_EXT_MOTION_PLUS_NUNCHUK);
    ext->nunchuk_x = 128;
    ext->nunchuk_y = 128;
    ext->nunchuk_ax = 0x200;
    ext->nunchuk_ay = 0x200;
    ext->nunchuk_az = 0x26a;
    ext->classic_lx = 32;
    ext->classic_ly = 32;
    ext->classic_rx = 16;
    ext->classic_ry = 16;
    if (ext->mp_present) {
        memcpy(ext->a6 + 0xfa, k_id_mp_inactive, 6);
        ext->a6[0xf7] = 0x0e;
    }
    apply_visibility(ext);
}

static void note_status(wiimote_ext_result_t *result, bool plugged)
{
    if (result == NULL || result->status_count >= 2) {
        return;
    }
    result->plugged[result->status_count++] = plugged ? 1 : 0;
}

static void activate_mp(wiimote_ext_t *ext, bool passthrough, wiimote_ext_result_t *result)
{
    bool was = wiimote_ext_connected_bit(ext);
    ext->mp_active = true;
    ext->mp_passthrough = passthrough && nunchuk_plugged(ext);
    /* A game that asks for classic-in-MP (0x07) is answered as solo MotionPlus. */
    if (passthrough && !nunchuk_plugged(ext)) {
        ext->mp_passthrough = false;
    }
    apply_visibility(ext);
    if (!was && wiimote_ext_connected_bit(ext)) {
        note_status(result, true);
    }
}

static void deactivate_mp(wiimote_ext_t *ext, wiimote_ext_result_t *result)
{
    bool was_pt = ext->mp_passthrough;
    ext->mp_active = false;
    ext->mp_passthrough = false;
    if (ext->mp_present) {
        memcpy(ext->a6 + 0xfa, was_pt ? k_id_mp_off_nunchuk : k_id_mp_off_solo, 6);
    }
    clear_crypto(ext);
    apply_visibility(ext);
    note_status(result, false);
    if (wiimote_ext_connected_bit(ext)) {
        note_status(result, true);
    }
}

static void maybe_finish_key(wiimote_ext_t *ext, uint32_t addr, size_t len)
{
    uint32_t end = addr + (uint32_t)len - 1;
    if (addr <= 0x4f && end >= 0x40) {
        ext->crypto_dirty = true;
    }
}

static void on_a4_byte(wiimote_ext_t *ext, uint32_t addr, uint8_t value, size_t write_len,
                       wiimote_ext_result_t *result)
{
    if (addr == 0xf0 && value == 0x55 && ext->mp_active) {
        deactivate_mp(ext, result);
        return;
    }
    (void)write_len;
    if (addr == 0xf0) {
        if (value == 0xaa) {
            ext->crypto_armed = true;
            ext->crypto_dirty = true;
        } else {
            clear_crypto(ext); /* 0x55 (the "new way" init) and anything else */
        }
    }
}

static int register_space(uint32_t addr, uint8_t **base, bool *present, wiimote_ext_t *ext)
{
    uint32_t page = addr & 0xffff00u;
    if (page == 0xa40000u) {
        *base = ext->a4;
        *present = ext->a4_visible;
        return 0;
    }
    if (page == 0xa60000u) {
        *base = ext->a6;
        *present = ext->mp_present && !ext->mp_active;
        return 1;
    }
    if (page == 0xb00000u) {
        *base = ext->ir;
        *present = true;
        return 2;
    }
    if (page == 0xa20000u) {
        *base = ext->speaker;
        *present = true;
        return 3;
    }
    return -1;
}

int wiimote_ext_read(wiimote_ext_t *ext, uint32_t addr, uint8_t *dst, size_t len, uint8_t *error)
{
    uint8_t *base = NULL;
    bool present = false;
    uint32_t off;
    size_t i;
    int space = register_space(addr, &base, &present, ext);
    *error = 0;
    if (len == 0) {
        return 0;
    }
    off = addr & 0xffu;
    if (space < 0 || off + len > 0x100) {
        *error = 8;
        return -1;
    }
    if (!present) {
        *error = 7;
        return -1;
    }
    if (space == 0) {
        refresh_crypto(ext);
    }
    if (space == 0 && !ext->mp_active) {
        uint8_t saved = ext->mp_phase;
        refresh_plaintext(ext);
        ext->mp_phase = saved;
    }
    for (i = 0; i < len; i++) {
        uint8_t b = base[off + i];
        uint32_t at = off + (uint32_t)i;
        if (space == 0 && ext->crypto.active) {
            b = wiimote_crypto_encrypt_byte(&ext->crypto, b, (uint8_t)at);
        }
        dst[i] = b;
    }
    return 0;
}

int wiimote_ext_write(wiimote_ext_t *ext, uint32_t addr, const uint8_t *src, size_t len,
                      uint8_t *error, wiimote_ext_result_t *result)
{
    uint8_t *base = NULL;
    bool present = false;
    uint32_t off;
    size_t i;
    int space;
    if (result) {
        result->status_count = 0;
    }
    *error = 0;
    if (len == 0) {
        return 0;
    }
    space = register_space(addr, &base, &present, ext);
    off = addr & 0xffu;
    if (space < 0 || off + len > 0x100) {
        *error = 8;
        return -1;
    }
    if (!present) {
        *error = 7;
        return -1;
    }
    for (i = 0; i < len; i++) {
        base[off + i] = src[i];
        if (space == 0) {
            on_a4_byte(ext, off + (uint32_t)i, src[i], len, result);
        }
    }
    if (space == 0) {
        maybe_finish_key(ext, off, len);
    }
    if (space == 1 && off <= 0xfe && off + len > 0xfe) {
        uint8_t cmd = ext->a6[0xfe];
        if (ext->a6[0xf0] == 0x55 && (cmd == 0x04 || cmd == 0x05 || cmd == 0x07)) {
            bool want_pt = (cmd == 0x05);
            activate_mp(ext, want_pt, result);
        }
    }
    if (space == 1 && off <= 0xf0 && off + len > 0xf0 && ext->a6[0xf0] == 0x55) {
        ext->a6[0xf7] = 0x0e;
    }
    return 0;
}

static void pack_nunchuk(const wiimote_ext_t *ext, uint8_t out[6], bool passthrough)
{
    uint16_t ax = ext->nunchuk_ax;
    uint16_t ay = ext->nunchuk_ay;
    uint16_t az = ext->nunchuk_az;
    out[0] = ext->nunchuk_x;
    out[1] = ext->nunchuk_y;
    out[2] = (uint8_t)(ax >> 2);
    out[3] = (uint8_t)(ay >> 2);
    out[4] = (uint8_t)(az >> 2);
    if (!passthrough) {
        out[5] = (uint8_t)(((az & 3) << 6) | ((ay & 3) << 4) | ((ax & 3) << 2));
        if (!ext->nunchuk_c) {
            out[5] |= 0x02;
        }
        if (!ext->nunchuk_z) {
            out[5] |= 0x01;
        }
        return;
    }
    /* Passthrough keeps byte 5 bit 1 clear so the host can tell it from gyro frames. */
    out[5] = (uint8_t)(((az & 3) << 6) | ((ay & 1) << 5) | ((ax & 1) << 4));
    if (!ext->nunchuk_c) {
        out[5] |= 0x08;
    }
    if (!ext->nunchuk_z) {
        out[5] |= 0x04;
    }
}

static bool classic_down(const wiimote_ext_t *ext, wiimote_button_t button)
{
    return (ext->classic_pressed & (1u << button)) != 0;
}

static void pack_classic(const wiimote_ext_t *ext, uint8_t out[6])
{
    uint8_t lx = ext->classic_lx & 0x3f;
    uint8_t ly = ext->classic_ly & 0x3f;
    uint8_t rx = ext->classic_rx & 0x1f;
    uint8_t ry = ext->classic_ry & 0x1f;
    uint8_t lt = ext->classic_lt & 0x1f;
    uint8_t rt = ext->classic_rt & 0x1f;
    /* Data format 1 (wiibrew "Classic Controller", Linux hid-wiimote, Dolphin):
     *   0: RX<4:3> LX<5:0>
     *   1: RX<2:1> LY<5:0>
     *   2: RX<0>   LT<4:3> RY<4:0>
     *   3: LT<2:0> RT<4:0>                                                  */
    out[0] = (uint8_t)((((rx >> 3) & 0x03) << 6) | lx);
    out[1] = (uint8_t)((((rx >> 1) & 0x03) << 6) | ly);
    out[2] = (uint8_t)(((rx & 0x01) << 7) | (((lt >> 3) & 0x03) << 5) | ry);
    out[3] = (uint8_t)(((lt & 0x07) << 5) | rt);
    out[4] = 0xff;
    out[5] = 0xff;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_RIGHT)) out[4] &= (uint8_t)~0x80;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_DOWN)) out[4] &= (uint8_t)~0x40;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_L)) out[4] &= (uint8_t)~0x20;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_MINUS)) out[4] &= (uint8_t)~0x10;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_HOME)) out[4] &= (uint8_t)~0x08;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_PLUS)) out[4] &= (uint8_t)~0x04;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_R)) out[4] &= (uint8_t)~0x02;
    out[4] |= 0x01;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_ZL)) out[5] &= (uint8_t)~0x80;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_B)) out[5] &= (uint8_t)~0x40;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_Y)) out[5] &= (uint8_t)~0x20;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_A)) out[5] &= (uint8_t)~0x10;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_X)) out[5] &= (uint8_t)~0x08;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_ZR)) out[5] &= (uint8_t)~0x04;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_LEFT)) out[5] &= (uint8_t)~0x02;
    if (classic_down(ext, WIIMOTE_BUTTON_CLASSIC_UP)) out[5] &= (uint8_t)~0x01;
}

static void encode_gyro(int dps, uint8_t *lo, uint8_t *hi, int *slow)
{
    int units;
    int value;
    if (dps > 440 || dps < -440) {
        *slow = 0;
        units = (int)((dps * 151448L + (dps >= 0 ? 25000 : -25000)) / 50000);
    } else {
        *slow = 1;
        units = (int)((dps * 13768L + (dps >= 0 ? 500 : -500)) / 1000);
    }
    value = 0x1f7f + units;
    if (value < 0) {
        value = 0;
    }
    if (value > 0x3fff) {
        value = 0x3fff;
    }
    *lo = (uint8_t)value;
    *hi = (uint8_t)((value >> 8) & 0x3f);
}

static void pack_motionplus(const wiimote_ext_t *ext, uint8_t out[6])
{
    uint8_t yaw_lo, roll_lo, pitch_lo, yaw_hi, roll_hi, pitch_hi;
    int yaw_slow, roll_slow, pitch_slow;
    encode_gyro(ext->yaw_dps, &yaw_lo, &yaw_hi, &yaw_slow);
    encode_gyro(ext->roll_dps, &roll_lo, &roll_hi, &roll_slow);
    encode_gyro(ext->pitch_dps, &pitch_lo, &pitch_hi, &pitch_slow);
    out[0] = yaw_lo;
    out[1] = roll_lo;
    out[2] = pitch_lo;
    out[3] = (uint8_t)((yaw_hi << 2) | (yaw_slow ? 0x02 : 0) | (pitch_slow ? 0x01 : 0));
    out[4] = (uint8_t)((roll_hi << 2) | (roll_slow ? 0x02 : 0));
    if (ext->mp_passthrough) {
        out[4] |= 0x01;
    }
    out[5] = (uint8_t)((pitch_hi << 2) | 0x02);
}

static void refresh_plaintext(wiimote_ext_t *ext)
{
    uint8_t frame[6];
    memset(frame, 0, sizeof(frame));
    if (ext->mp_active) {
        bool nunchuk_frame = ext->mp_passthrough && (ext->mp_phase & 1);
        if (nunchuk_frame) {
            pack_nunchuk(ext, frame, true);
        } else {
            pack_motionplus(ext, frame);
        }
        ext->mp_phase++;
    } else if (nunchuk_plugged(ext)) {
        pack_nunchuk(ext, frame, false);
    } else if (classic_plugged(ext)) {
        pack_classic(ext, frame);
    }
    memcpy(ext->a4, frame, 6);
    memcpy(ext->a4 + 8, frame, 6);
}

void wiimote_ext_stream(wiimote_ext_t *ext, uint8_t *dst, size_t len)
{
    size_t i;
    refresh_plaintext(ext);
    refresh_crypto(ext);
    for (i = 0; i < len; i++) {
        uint8_t at = (uint8_t)(i & 0xff);
        uint8_t b = ext->a4[at];
        if (ext->crypto.active && !ext->mp_active) {
            b = wiimote_crypto_encrypt_byte(&ext->crypto, b, at);
        }
        dst[i] = b;
    }
}
