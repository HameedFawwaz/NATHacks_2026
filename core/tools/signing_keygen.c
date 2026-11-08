/*
 * Generates a fresh 256-bit MAVLink signing key and writes it to a file with
 * mode 0600. Usage: signing_keygen <path> [--from-passphrase] [--print-hex]
 *
 * --from-passphrase reads one ASCII line from stdin, deriving the same key
 * as MAVProxy's signing setup. --print-hex prints raw key material for our
 * own peers; MAVProxy does not accept a raw hex key (see core/README.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/crypto/link_signing.h"

int main(int argc, char **argv) {
    bool from_passphrase = false, print_hex = false;
    if (argc < 2) {
        fprintf(stderr, "usage: %s <key-file> [--from-passphrase] [--print-hex]\n", argv[0]);
        return 2;
    }
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--from-passphrase") == 0) from_passphrase = true;
        else if (strcmp(argv[i], "--print-hex") == 0) print_hex = true;
        else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return 2;
        }
    }
    uint8_t key[CORE_SIGNING_KEY_LEN] = {0};
    bool ok;
    if (from_passphrase) {
        char *line = NULL;
        size_t capacity = 0;
        ssize_t len = getline(&line, &capacity, stdin);
        if (len > 0 && line[len - 1] == '\n') line[--len] = '\0';
        if (len > 0 && line[len - 1] == '\r') line[--len] = '\0';
        ok = len > 0 && !ferror(stdin) && (size_t)len == strlen(line) &&
             core_signing_key_from_passphrase(line, key);
        if (!ok) fprintf(stderr, "expected a nonempty printable ASCII passphrase (0x21..0x7e, no whitespace or controls) on stdin\n");
        volatile char *p = line;
        for (size_t i = 0; i < capacity; i++) p[i] = 0;
        free(line);
    } else {
        ok = core_signing_key_generate(key);
    }
    if (ok) ok = core_signing_key_write_file(argv[1], key);
    volatile uint8_t *k = key;
    for (size_t i = 0; i < sizeof(key); i++) k[i] = 0;
    if (!ok) {
        return 1;
    }
    printf("wrote new signing key to %s (mode 0600)\n", argv[1]);

    if (print_hex) {
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
