/* SPDX-License-Identifier: MIT */
#ifndef WIIMOTE_CRYPTO_H
#define WIIMOTE_CRYPTO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t ft[8];
    uint8_t sb[8];
    bool active;
} wiimote_crypto_t;

/** Pads of 0x97, the published constant used before a 16-byte key is loaded. */
void wiimote_crypto_load_constant(wiimote_crypto_t *crypto, uint8_t pad);

/**
 * Load the 16 bytes the console wrote to extension registers 0x40..0x4f
 * (10 random bytes, then 6 secret bytes, each half in reverse byte order).
 * Returns the matching table index 0..6, or -1 when the secret does not
 * match and the constant 0x97 pads are used instead.
 */
int wiimote_crypto_load_key(wiimote_crypto_t *crypto, const uint8_t key[16]);

/** Fill registers 10..15 with the secret matching key[0..9] for a table index (register layout). */
void wiimote_crypto_fill_secret(uint8_t key[16], int index);

uint8_t wiimote_crypto_encrypt_byte(const wiimote_crypto_t *crypto, uint8_t plain, uint8_t addr);
uint8_t wiimote_crypto_decrypt_byte(const wiimote_crypto_t *crypto, uint8_t enc, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif
