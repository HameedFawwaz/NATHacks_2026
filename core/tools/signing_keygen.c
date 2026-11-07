/*
 * Generates a fresh 256-bit MAVLink signing key and writes it to a file with
 * mode 0600. Usage: signing_keygen <path> [--print-hex]
 *
 * --print-hex also prints the key to stdout, which is what you paste into
 * MAVProxy's / ArduPilot's side of the setup (see core/README.md).
 */
#include <stdio.h>
#include <string.h>

#include "core/crypto/link_signing.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <key-file> [--print-hex]\n", argv[0]);
        return 2;
    }
    if (!core_signing_key_generate_file(argv[1])) {
        return 1;
    }
    printf("wrote new signing key to %s (mode 0600)\n", argv[1]);

    if (argc > 2 && strcmp(argv[2], "--print-hex") == 0) {
        uint8_t key[CORE_SIGNING_KEY_LEN];
        char hex[2 * CORE_SIGNING_KEY_LEN + 1];
        if (!core_signing_key_load_file(argv[1], key)) {
            return 1;
        }
        core_signing_key_to_hex(key, hex);
        printf("%s\n", hex);
    }
    return 0;
}
