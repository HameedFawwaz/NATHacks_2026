#include "core/crypto/link_signing.h"

#include <string.h>

#define MAVLINK_HELPER static inline
#include "mavlink_sha256.h"

bool core_signing_key_from_passphrase(const char *passphrase, uint8_t key_out[CORE_SIGNING_KEY_LEN]) {
    if (!passphrase || !key_out || !passphrase[0]) {
        return false;
    }
    size_t len = strlen(passphrase);
    if (len > UINT64_MAX / 8) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)passphrase[i];
        if (c < 0x21 || c > 0x7e) return false;
    }

    mavlink_sha256_ctx ctx;
    mavlink_sha256_init(&ctx);
    /* The vendored byte counter is only 32 bits. Its block offset remains
     * valid on wrap; track the full bit length separately for finalisation. */
    for (size_t offset = 0; offset < len;) {
        uint32_t chunk = (uint32_t)((len - offset > 4096) ? 4096 : len - offset);
        mavlink_sha256_update(&ctx, passphrase + offset, chunk);
        offset += chunk;
    }

    /* Standard SHA-256 padding, followed by the 64-bit big-endian bit count.
     * Unlike final_48(), serialize all eight state words (not just six bytes). */
    uint8_t padding[128] = {0x80};
    size_t pad_len = 64 - ((len % 64 + 8) % 64);
    uint64_t bits = (uint64_t)len * 8;
    for (size_t i = 0; i < 8; i++) {
        padding[pad_len + i] = (uint8_t)(bits >> (56 - 8 * i));
    }
    mavlink_sha256_update(&ctx, padding, (uint32_t)(pad_len + 8));
    for (size_t i = 0; i < CORE_SIGNING_KEY_LEN; i++) {
        key_out[i] = (uint8_t)(ctx.counter[i / 4] >> (24 - 8 * (i % 4)));
    }
    volatile uint8_t *p = (volatile uint8_t *)&ctx;
    for (size_t i = 0; i < sizeof(ctx); i++) p[i] = 0;
    return true;
}
