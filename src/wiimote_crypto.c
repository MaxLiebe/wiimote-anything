/* SPDX-License-Identifier: MIT */
/*
 * Extension-bus cipher used by the Nunchuk and Classic Controller.
 *
 * The 256-byte tables are the published substitution boxes required for a
 * game to accept the peripheral. The key schedule below is an independent
 * implementation of that published transform: an 8-bit rotate, a table
 * index search, then the add/xor pads. It is not Nintendo source.
 */
#include "wiimote_crypto.h"

#include <string.h>

#include "wiimote_crypto_tables.inc"

static const uint8_t k_answer[7][6] = {
    {0xa8, 0x77, 0xa6, 0xe0, 0xf7, 0x43},
    {0x5a, 0x35, 0x85, 0xe2, 0x72, 0x97},
    {0x8f, 0xb7, 0x1a, 0x62, 0x87, 0x38},
    {0x0d, 0x67, 0xc7, 0xbe, 0x4f, 0x3e},
    {0x20, 0x76, 0x37, 0x8f, 0x68, 0xb7},
    {0xa9, 0x26, 0x3f, 0x2b, 0x10, 0xe3},
    {0x30, 0x7e, 0x90, 0x0e, 0x85, 0x0a},
};

/* 8-bit rotate. Shift counts coming out of the key table are not limited to 0..7. */
static uint8_t rot_right(uint8_t value, uint8_t places)
{
    places = (uint8_t)(places & 7);
    if (places == 0) {
        return value;
    }
    return (uint8_t)((value >> places) | (value << (8 - places)));
}

static void secret_for_index(const uint8_t random10[10], int index, uint8_t secret[6])
{
    uint8_t mixed[10];
    int i;
    for (i = 0; i < 10; i++) {
        mixed[i] = k_keygen[random10[i]];
    }
    secret[0] = (uint8_t)((rot_right((uint8_t)(k_answer[index][0] ^ mixed[5]), mixed[2]) - mixed[9]) ^ mixed[4]);
    secret[1] = (uint8_t)((rot_right((uint8_t)(k_answer[index][1] ^ mixed[1]), mixed[0]) - mixed[5]) ^ mixed[7]);
    secret[2] = (uint8_t)((rot_right((uint8_t)(k_answer[index][2] ^ mixed[6]), mixed[8]) - mixed[2]) ^ mixed[0]);
    secret[3] = (uint8_t)((rot_right((uint8_t)(k_answer[index][3] ^ mixed[4]), mixed[7]) - mixed[3]) ^ mixed[2]);
    secret[4] = (uint8_t)((rot_right((uint8_t)(k_answer[index][4] ^ mixed[1]), mixed[6]) - mixed[3]) ^ mixed[4]);
    secret[5] = (uint8_t)((rot_right((uint8_t)(k_answer[index][5] ^ mixed[7]), mixed[8]) - mixed[5]) ^ mixed[9]);
}

/* The console writes the key to registers 0x40..0x4f. The random half is
 * 0x40..0x49 and the secret half 0x4a..0x4f, each stored in reverse byte
 * order (Dolphin's KeyGen::GenerateFromExtensionKeyData does the same). */
static void split_key(const uint8_t regs[16], uint8_t random10[10], uint8_t secret[6])
{
    int i;
    for (i = 0; i < 10; i++) {
        random10[i] = regs[9 - i];
    }
    for (i = 0; i < 6; i++) {
        secret[i] = regs[15 - i];
    }
}

void wiimote_crypto_fill_secret(uint8_t key[16], int index)
{
    uint8_t random10[10];
    uint8_t secret[6];
    int i;
    if (index < 0 || index > 6) {
        return;
    }
    for (i = 0; i < 10; i++) {
        random10[i] = key[9 - i];
    }
    secret_for_index(random10, index, secret);
    for (i = 0; i < 6; i++) {
        key[15 - i] = secret[i];
    }
}

static int matching_index(const uint8_t random10[10], const uint8_t secret[6])
{
    int index;
    for (index = 0; index < 7; index++) {
        uint8_t candidate[6];
        secret_for_index(random10, index, candidate);
        if (memcmp(candidate, secret, 6) == 0) {
            return index;
        }
    }
    return -1;
}

static void pads_from_tables(uint8_t ft[8], uint8_t sb[8], const uint8_t random10[10],
                             const uint8_t secret[6], const uint8_t *box_a, const uint8_t *box_b)
{
    ft[0] = (uint8_t)(box_a[secret[4]] ^ box_b[random10[3]]);
    ft[1] = (uint8_t)(box_a[secret[2]] ^ box_b[random10[5]]);
    ft[2] = (uint8_t)(box_a[secret[5]] ^ box_b[random10[7]]);
    ft[3] = (uint8_t)(box_a[secret[0]] ^ box_b[random10[2]]);
    ft[4] = (uint8_t)(box_a[secret[1]] ^ box_b[random10[4]]);
    ft[5] = (uint8_t)(box_a[secret[3]] ^ box_b[random10[9]]);
    ft[6] = (uint8_t)(box_a[random10[0]] ^ box_b[random10[6]]);
    ft[7] = (uint8_t)(box_a[random10[1]] ^ box_b[random10[8]]);

    sb[0] = (uint8_t)(box_a[secret[0]] ^ box_b[random10[1]]);
    sb[1] = (uint8_t)(box_a[secret[5]] ^ box_b[random10[4]]);
    sb[2] = (uint8_t)(box_a[secret[3]] ^ box_b[random10[0]]);
    sb[3] = (uint8_t)(box_a[secret[2]] ^ box_b[random10[9]]);
    sb[4] = (uint8_t)(box_a[secret[4]] ^ box_b[random10[7]]);
    sb[5] = (uint8_t)(box_a[secret[1]] ^ box_b[random10[8]]);
    sb[6] = (uint8_t)(box_a[random10[3]] ^ box_b[random10[5]]);
    sb[7] = (uint8_t)(box_a[random10[2]] ^ box_b[random10[6]]);
}

void wiimote_crypto_load_constant(wiimote_crypto_t *crypto, uint8_t pad)
{
    memset(crypto->ft, pad, sizeof(crypto->ft));
    memset(crypto->sb, pad, sizeof(crypto->sb));
    crypto->active = true;
}

int wiimote_crypto_load_key(wiimote_crypto_t *crypto, const uint8_t key[16])
{
    uint8_t random10[10];
    uint8_t secret[6];
    int index;
    split_key(key, random10, secret);
    index = matching_index(random10, secret);
    if (index < 0) {
        /* Retail software always matches; some homebrew writes zeros. */
        wiimote_crypto_load_constant(crypto, 0x97);
        return -1;
    }
    pads_from_tables(crypto->ft, crypto->sb, random10, secret, k_sboxes[index], k_sboxes[index + 1]);
    crypto->active = true;
    return index;
}

uint8_t wiimote_crypto_encrypt_byte(const wiimote_crypto_t *crypto, uint8_t plain, uint8_t addr)
{
    uint8_t slot = (uint8_t)(addr & 7);
    return (uint8_t)((plain - crypto->ft[slot]) ^ crypto->sb[slot]);
}

uint8_t wiimote_crypto_decrypt_byte(const wiimote_crypto_t *crypto, uint8_t enc, uint8_t addr)
{
    uint8_t slot = (uint8_t)(addr & 7);
    return (uint8_t)((enc ^ crypto->sb[slot]) + crypto->ft[slot]);
}
