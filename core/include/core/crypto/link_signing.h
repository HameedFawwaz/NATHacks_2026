#ifndef CORE_CRYPTO_LINK_SIGNING_H
#define CORE_CRYPTO_LINK_SIGNING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Key handling for MAVLink 2 message signing on the operator<->drone link.
 *
 * Signing authenticates every frame with an HMAC-style SHA-256 signature over
 * (key, header, payload, link id, timestamp). Frames with a missing, wrong,
 * or replayed signature are dropped by the connection layer. It does NOT
 * encrypt: telemetry remains readable by an eavesdropper, but cannot be
 * forged or replayed.
 *
 * Keys are 32 bytes. On disk they are stored as 64 hex characters (plus an
 * optional trailing newline) in a file that should be mode 0600.
 */

#define CORE_SIGNING_KEY_LEN 32

/* Parses 64 hex chars (case-insensitive) into key_out. Returns false on bad length/characters. */
bool core_signing_key_from_hex(const char *hex, uint8_t key_out[CORE_SIGNING_KEY_LEN]);

/* Writes key_in as lowercase hex + NUL into out (needs >= 2*CORE_SIGNING_KEY_LEN + 1 bytes). */
void core_signing_key_to_hex(const uint8_t key_in[CORE_SIGNING_KEY_LEN], char *out);

/* Loads a key from a hex file. Warns (but still loads) if the file is group/world accessible. */
bool core_signing_key_load_file(const char *path, uint8_t key_out[CORE_SIGNING_KEY_LEN]);

/* Fills key_out from the OS CSPRNG (/dev/urandom). */
bool core_signing_key_generate(uint8_t key_out[CORE_SIGNING_KEY_LEN]);

/* Generates a fresh key and writes it to path with mode 0600. Refuses to overwrite an existing file. */
bool core_signing_key_generate_file(const char *path);

/*
 * Convenience: loads the key named by the CORE_SIGNING_KEY_FILE environment
 * variable. Returns false if the variable is unset or the file is unusable.
 */
bool core_signing_key_load_from_env(uint8_t key_out[CORE_SIGNING_KEY_LEN]);

#endif /* CORE_CRYPTO_LINK_SIGNING_H */
