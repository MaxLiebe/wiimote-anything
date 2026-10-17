/* SPDX-License-Identifier: MIT */
#include <stdio.h>
#include <string.h>

#include "wiimote_crypto.h"
#include "wiimote_proto.h"

static int g_fails;

static void check(int cond, const char *what, int line)
{
    if (!cond) {
        printf("FAIL %s:%d %s\n", "test_proto.c", line, what);
        g_fails++;
    }
}

#define CHECK(c) check((c), #c, __LINE__)

static void feed(wiimote_proto_t *p, const uint8_t *bytes, size_t n)
{
    wiimote_proto_handle_output(p, bytes, n);
}

static int take(wiimote_proto_t *p, uint8_t *out, uint32_t ms)
{
    return wiimote_proto_poll(p, out, 32, ms);
}

static void test_buttons_and_accel(void)
{
    wiimote_proto_t p;
    uint8_t pkt[32];
    int n;
    wiimote_proto_init(&p, WIIMOTE_EXT_NONE, 0xc0);
    wiimote_proto_set_button(&p, WIIMOTE_BUTTON_A, true);
    n = wiimote_proto_build(&p, 0x30, pkt, sizeof(pkt));
    CHECK(n == 4);
    CHECK(pkt[0] == 0xa1 && pkt[1] == 0x30);
    CHECK(pkt[2] == 0x00 && pkt[3] == 0x08);

    n = wiimote_proto_build(&p, 0x31, pkt, sizeof(pkt));
    CHECK(n == 7);
    CHECK(pkt[2] == 0x00);
    CHECK((pkt[3] & 0x48) == 0x48);
    CHECK(pkt[4] == 0x80 && pkt[5] == 0x80 && pkt[6] == 0x9a);
}

static void test_nunchuk_and_classic(void)
{
    wiimote_proto_t p;
    uint8_t pkt[32];
    wiimote_proto_init(&p, WIIMOTE_EXT_NUNCHUK, 0xc0);
    wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
    CHECK(pkt[4] == 128 && pkt[5] == 128);
    CHECK((pkt[9] & 0x03) == 0x03);

    wiimote_proto_set_button(&p, WIIMOTE_BUTTON_NUNCHUK_Z, true);
    wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
    CHECK((pkt[9] & 0x01) == 0);

    wiimote_proto_init(&p, WIIMOTE_EXT_CLASSIC, 0xc0);
    wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
    /* Rest: LX = LY = 32, RX = RY = 16, triggers 0. */
    CHECK(pkt[4] == 0xa0 && pkt[5] == 0x20 && pkt[6] == 0x10 && pkt[7] == 0x00);
    CHECK(pkt[8] == 0xff);
    CHECK(pkt[9] == 0xff);
    wiimote_proto_set_button(&p, WIIMOTE_BUTTON_CLASSIC_A, true);
    wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
    CHECK(pkt[9] == (uint8_t)~0x10);
    CHECK((pkt[8] & 0x01) == 0x01);

    {
        const uint8_t read_cal[] = {0x17, 0x04, 0xa4, 0x00, 0x20, 0x00, 0x10};
        feed(&p, read_cal, sizeof(read_cal));
        CHECK(take(&p, pkt, 0) == 23);
        CHECK(pkt[1] == 0x21);
        /* LX max=0xff, min=0x00, center=0x80; LY the same */
        CHECK(pkt[7] == 0xff && pkt[8] == 0x00 && pkt[9] == 0x80);
        CHECK(pkt[10] == 0xff && pkt[11] == 0x00 && pkt[12] == 0x80);
        /* 2-byte checksum: block[15] == (uint8_t)(block[14] + 0x55) */
        CHECK(pkt[22] == (uint8_t)(pkt[21] + 0x55));
    }
}

/* Decode exactly like Linux hid-wiimote (wiimod_classic_in_ext). */
static void decode_classic(const uint8_t *ext, int *lx, int *ly, int *rx, int *ry, int *lt, int *rt)
{
    *lx = ext[0] & 0x3f;
    *ly = ext[1] & 0x3f;
    *rx = ((ext[0] >> 3) & 0x18) | ((ext[1] >> 5) & 0x06) | ((ext[2] >> 7) & 0x01);
    *ry = ext[2] & 0x1f;
    *rt = ext[3] & 0x1f;
    *lt = ((ext[2] >> 2) & 0x18) | ((ext[3] >> 5) & 0x07);
}

static void test_classic_axes_roundtrip(void)
{
    static const int vals6[] = {0, 1, 17, 32, 46, 62, 63};
    static const int vals5[] = {0, 1, 9, 16, 22, 30, 31};
    wiimote_proto_t p;
    uint8_t pkt[32];
    size_t i;
    wiimote_proto_init(&p, WIIMOTE_EXT_CLASSIC, 0xc0);
    for (i = 0; i < sizeof(vals6) / sizeof(vals6[0]); i++) {
        int lx, ly, rx, ry, lt, rt;
        size_t j = (i + 3) % (sizeof(vals5) / sizeof(vals5[0]));
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_LX, vals6[i]);
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_LY, vals6[(i + 1) % 7]);
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_RX, vals5[i]);
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_RY, vals5[j]);
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_LT, vals5[(i + 5) % 7]);
        wiimote_proto_set_axis_raw(&p, WIIMOTE_AXIS_CLASSIC_RT, vals5[(i + 2) % 7]);
        wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
        decode_classic(pkt + 4, &lx, &ly, &rx, &ry, &lt, &rt);
        CHECK(lx == vals6[i]);
        CHECK(ly == vals6[(i + 1) % 7]);
        CHECK(rx == vals5[i]);
        CHECK(ry == vals5[j]);
        CHECK(lt == vals5[(i + 5) % 7]);
        CHECK(rt == vals5[(i + 2) % 7]);
        /* Buttons untouched by axis bits: all released. */
        CHECK(pkt[8] == 0xff && pkt[9] == 0xff);
    }
}

static void test_extension_init_and_missing(void)
{
    wiimote_proto_t p;
    uint8_t pkt[32];
    const uint8_t read_id[] = {0x17, 0x04, 0xa4, 0x00, 0xfa, 0x00, 0x06};
    const uint8_t init1[] = {0x16, 0x04, 0xa4, 0x00, 0xf0, 0x01, 0x55};
    const uint8_t init2[] = {0x16, 0x04, 0xa4, 0x00, 0xfb, 0x01, 0x00};
    wiimote_proto_init(&p, WIIMOTE_EXT_NONE, 0xc0);
    feed(&p, read_id, sizeof(read_id));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK(pkt[1] == 0x21);
    CHECK((pkt[4] & 0x0f) == 7);

    wiimote_proto_init(&p, WIIMOTE_EXT_NUNCHUK, 0xc0);
    feed(&p, init1, sizeof(init1));
    feed(&p, init2, sizeof(init2));
    feed(&p, read_id, sizeof(read_id));
    while (take(&p, pkt, 0) > 0 && pkt[1] != 0x21) {
    }
    CHECK(pkt[1] == 0x21);
    CHECK((pkt[4] & 0x0f) == 0);
    CHECK(pkt[7] == 0x00 && pkt[8] == 0x00 && pkt[9] == 0xa4 && pkt[10] == 0x20);
    CHECK(pkt[11] == 0x00 && pkt[12] == 0x00);
}

static void test_motionplus(void)
{
    wiimote_proto_t p;
    uint8_t pkt[32];
    uint8_t a[32];
    uint8_t b[32];
    const uint8_t read_fe[] = {0x17, 0x04, 0xa6, 0x00, 0xfe, 0x00, 0x02};
    const uint8_t read_a4[] = {0x17, 0x04, 0xa4, 0x00, 0xfa, 0x00, 0x06};
    const uint8_t arm[] = {0x16, 0x04, 0xa6, 0x00, 0xf0, 0x01, 0x55};
    const uint8_t act[] = {0x16, 0x04, 0xa6, 0x00, 0xfe, 0x01, 0x04};
    const uint8_t act_pt[] = {0x16, 0x04, 0xa6, 0x00, 0xfe, 0x01, 0x05};
    const uint8_t read_cal[] = {0x17, 0x04, 0xa4, 0x00, 0x20, 0x00, 0x02};

    wiimote_proto_init(&p, WIIMOTE_EXT_MOTION_PLUS, 0xc0);
    feed(&p, read_fe, sizeof(read_fe));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK((pkt[4] & 0x0f) == 0);
    CHECK(pkt[7] == 0x00 && pkt[8] == 0x05);
    feed(&p, read_a4, sizeof(read_a4));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK((pkt[4] & 0x0f) == 7);

    feed(&p, arm, sizeof(arm));
    feed(&p, act, sizeof(act));
    while (take(&p, pkt, 0) > 0) {
        if (pkt[1] == 0x20) {
            CHECK((pkt[4] & 0x02) != 0);
        }
    }
    feed(&p, read_a4, sizeof(read_a4));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK(pkt[7] == 0x00 && pkt[9] == 0xa4 && pkt[11] == 0x04 && pkt[12] == 0x05);

    wiimote_proto_build(&p, 0x32, a, sizeof(a));
    CHECK(a[4] == 0x7f && a[5] == 0x7f && a[6] == 0x7f);
    CHECK((a[7] & 0x03) == 0x03);
    CHECK((a[9] & 0x02) == 0x02);

    wiimote_proto_set_gyro(&p, 0, 0, 1000);
    wiimote_proto_build(&p, 0x32, a, sizeof(a));
    CHECK((a[7] & 0x02) == 0);

    feed(&p, read_cal, sizeof(read_cal));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK(pkt[7] == 0x7d && pkt[8] == 0xfc);

    wiimote_proto_init(&p, WIIMOTE_EXT_MOTION_PLUS_NUNCHUK, 0xc0);
    feed(&p, arm, sizeof(arm));
    feed(&p, act_pt, sizeof(act_pt));
    while (take(&p, pkt, 0) > 0) {
    }
    wiimote_proto_build(&p, 0x37, a, sizeof(a));
    wiimote_proto_build(&p, 0x37, b, sizeof(b));
    CHECK(((a[22] ^ b[22]) & 0x02) == 0x02);
}

static void test_crypto(void)
{
    wiimote_crypto_t crypto;
    uint8_t key[16];
    uint8_t plain[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t enc[8];
    int i;
    wiimote_proto_t p;
    uint8_t pkt[32];
    const uint8_t oldkey[] = {0x16, 0x04, 0xa4, 0x00, 0x40, 0x01, 0x00};
    const uint8_t read0[] = {0x17, 0x04, 0xa4, 0x00, 0x00, 0x00, 0x01};

    memset(key, 0, sizeof(key));
    for (i = 0; i < 10; i++) {
        key[i] = (uint8_t)(i + 1);
    }
    wiimote_crypto_fill_secret(key, 3);
    CHECK(wiimote_crypto_load_key(&crypto, key) == 3);
    for (i = 0; i < 8; i++) {
        enc[i] = wiimote_crypto_encrypt_byte(&crypto, plain[i], (uint8_t)i);
        CHECK(wiimote_crypto_decrypt_byte(&crypto, enc[i], (uint8_t)i) == plain[i]);
    }

    memset(key, 0, sizeof(key));
    CHECK(wiimote_crypto_load_key(&crypto, key) < 0);
    CHECK(crypto.ft[0] == 0x97 && crypto.sb[3] == 0x97);

    wiimote_proto_init(&p, WIIMOTE_EXT_NUNCHUK, 0xc0);
    feed(&p, oldkey, sizeof(oldkey));
    while (take(&p, pkt, 0) > 0) {
    }
    feed(&p, read0, sizeof(read0));
    CHECK(take(&p, pkt, 0) == 23);
    /* Without 0xAA in register 0xF0 the bus stays plaintext (as in Dolphin). */
    CHECK(pkt[7] == 0x80);
}

/* Vector computed with Dolphin's KeyGen1stParty (Encryption.cpp): the key a
 * Wii writes to 0x40..0x4F for table index 4, and what an idle Classic
 * Controller then reads back, encrypted. */
static void test_crypto_dolphin_vector(int key_first)
{
    static const uint8_t regs[16] = {0x13, 0x57, 0x9b, 0xdf, 0x24, 0x68, 0xac, 0xe0,
                                     0x31, 0x75, 0x8c, 0xf8, 0x7a, 0x4a, 0xe9, 0x6f};
    static const uint8_t want_data[6] = {0x5c, 0xad, 0x12, 0x7f, 0x8b, 0x7b};
    static const uint8_t want_id[6] = {0xe2, 0x7f, 0x26, 0x5a, 0x8d, 0xcb};
    const uint8_t enable[] = {0x16, 0x04, 0xa4, 0x00, 0xf0, 0x01, 0xaa};
    const uint8_t read_id[] = {0x17, 0x04, 0xa4, 0x00, 0xfa, 0x00, 0x06};
    uint8_t w[22];
    uint8_t pkt[32];
    wiimote_proto_t p;
    int chunk;
    wiimote_crypto_t crypto;

    CHECK(wiimote_crypto_load_key(&crypto, regs) == 4);

    wiimote_proto_init(&p, WIIMOTE_EXT_CLASSIC, 0xc0);
    if (!key_first) {
        feed(&p, enable, sizeof(enable));
    }
    /* The SDK writes the key in 6 + 6 + 4 byte chunks. */
    for (chunk = 0; chunk < 3; chunk++) {
        int off = chunk * 6;
        int n = chunk == 2 ? 4 : 6;
        memset(w, 0, sizeof(w));
        w[0] = 0x16;
        w[1] = 0x04;
        w[2] = 0xa4;
        w[3] = 0x00;
        w[4] = (uint8_t)(0x40 + off);
        w[5] = (uint8_t)n;
        memcpy(w + 6, regs + off, (size_t)n);
        feed(&p, w, sizeof(w));
    }
    if (key_first) {
        feed(&p, enable, sizeof(enable));
    }
    while (take(&p, pkt, 0) > 0) {
    }
    feed(&p, read_id, sizeof(read_id));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK(memcmp(pkt + 7, want_id, 6) == 0);
    wiimote_proto_build(&p, 0x32, pkt, sizeof(pkt));
    CHECK(memcmp(pkt + 4, want_data, 6) == 0);
}

static void test_eeprom_status_rumble_ir_mode(void)
{
    wiimote_proto_t p;
    uint8_t pkt[32];
    const uint8_t read_cal[] = {0x17, 0x00, 0x00, 0x00, 0x16, 0x00, 0x0a};
    const uint8_t status[] = {0x15, 0x00};
    const uint8_t leds[] = {0x11, 0x11};
    const uint8_t mode[] = {0x12, 0x04, 0x33};
    const uint8_t ir_on[] = {0x13, 0x04};
    const uint8_t ir_on2[] = {0x1a, 0x04};
    int i;

    wiimote_proto_init(&p, WIIMOTE_EXT_NUNCHUK, 0xc0);
    feed(&p, read_cal, sizeof(read_cal));
    CHECK(take(&p, pkt, 0) == 23);
    CHECK(pkt[1] == 0x21);
    CHECK(pkt[4] == 0x90);
    CHECK(pkt[7] == 0x80 && pkt[10] == 0x00 && pkt[14] == 0x2a && pkt[15] == 0x40 && pkt[16] == 0x0d);

    feed(&p, status, sizeof(status));
    CHECK(take(&p, pkt, 0) == 8);
    CHECK(pkt[1] == 0x20);
    CHECK((pkt[4] & 0x02) != 0);
    CHECK(pkt[7] == 0xc0);

    feed(&p, leds, sizeof(leds));
    CHECK(p.rumble);
    CHECK(p.leds == 0x01);

    feed(&p, mode, sizeof(mode));
    CHECK(p.continuous);
    CHECK(p.reporting);
    CHECK(p.mode == 0x33);

    wiimote_proto_set_ir_dot(&p, 0, 512, 384, 2);
    for (i = 1; i < 4; i++) {
        wiimote_proto_set_ir_dot(&p, i, 0, 0, -1);
    }
    wiimote_proto_build(&p, 0x36, pkt, sizeof(pkt));
    CHECK(pkt[4] == 0x00);
    CHECK(pkt[5] == 0x80);
    CHECK(pkt[6] == 0x6f);
    CHECK(pkt[7] == 0xff && pkt[8] == 0xff);

    feed(&p, ir_on, sizeof(ir_on));
    feed(&p, ir_on2, sizeof(ir_on2));
    feed(&p, status, sizeof(status));
    CHECK(take(&p, pkt, 0) == 8);
    CHECK((pkt[4] & 0x08) != 0);
    CHECK((pkt[4] & 0x01) == 0);
}

int main(void)
{
    test_buttons_and_accel();
    test_nunchuk_and_classic();
    test_classic_axes_roundtrip();
    test_extension_init_and_missing();
    test_motionplus();
    test_crypto();
    test_crypto_dolphin_vector(0);
    test_crypto_dolphin_vector(1);
    test_eeprom_status_rumble_ir_mode();
    if (g_fails) {
        printf("%d checks failed\n", g_fails);
        return 1;
    }
    printf("protocol tests passed\n");
    return 0;
}
