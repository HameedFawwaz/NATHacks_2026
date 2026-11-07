#include "core/crypto/link_signing.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "core/common/log.h"

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool core_signing_key_from_hex(const char *hex, uint8_t key_out[CORE_SIGNING_KEY_LEN]) {
    if (!hex || !key_out) {
        return false;
    }
    if (strlen(hex) != 2 * CORE_SIGNING_KEY_LEN) {
        return false;
    }
    uint8_t tmp[CORE_SIGNING_KEY_LEN];
    for (size_t i = 0; i < CORE_SIGNING_KEY_LEN; i++) {
        int hi = hex_val(hex[2 * i]);
        int lo = hex_val(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            return false;
        }
        tmp[i] = (uint8_t)((hi << 4) | lo);
    }
    memcpy(key_out, tmp, sizeof(tmp));
    return true;
}

void core_signing_key_to_hex(const uint8_t key_in[CORE_SIGNING_KEY_LEN], char *out) {
    static const char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < CORE_SIGNING_KEY_LEN; i++) {
        out[2 * i] = digits[key_in[i] >> 4];
        out[2 * i + 1] = digits[key_in[i] & 0x0f];
    }
    out[2 * CORE_SIGNING_KEY_LEN] = '\0';
}

bool core_signing_key_load_file(const char *path, uint8_t key_out[CORE_SIGNING_KEY_LEN]) {
    if (!path || !key_out) {
        return false;
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        CORE_LOG_ERROR("cannot open signing key file %s: %s", path, strerror(errno));
        return false;
    }

    struct stat st;
    if (fstat(fileno(f), &st) == 0 && (st.st_mode & (S_IRWXG | S_IRWXO)) != 0) {
        CORE_LOG_WARN("signing key file %s is accessible by group/others; run chmod 600", path);
    }

    char buf[2 * CORE_SIGNING_KEY_LEN + 8];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';

    /* Strip trailing whitespace/newlines. */
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' ' || buf[n - 1] == '\t')) {
        buf[--n] = '\0';
    }

    bool ok = core_signing_key_from_hex(buf, key_out);
    if (!ok) {
        CORE_LOG_ERROR("signing key file %s must contain exactly %d hex characters", path,
                       2 * CORE_SIGNING_KEY_LEN);
    }
    /* Don't leave key material lying around on the stack. */
    volatile char *p = buf;
    for (size_t i = 0; i < sizeof(buf); i++) p[i] = 0;
    return ok;
}

bool core_signing_key_generate(uint8_t key_out[CORE_SIGNING_KEY_LEN]) {
    if (!key_out) {
        return false;
    }
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd < 0) {
        CORE_LOG_ERROR("cannot open /dev/urandom: %s", strerror(errno));
        return false;
    }
    size_t got = 0;
    while (got < CORE_SIGNING_KEY_LEN) {
        ssize_t r = read(fd, key_out + got, CORE_SIGNING_KEY_LEN - got);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) continue;
            close(fd);
            CORE_LOG_ERROR("read from /dev/urandom failed");
            return false;
        }
        got += (size_t)r;
    }
    close(fd);
    return true;
}

bool core_signing_key_generate_file(const char *path) {
    if (!path) {
        return false;
    }
    uint8_t key[CORE_SIGNING_KEY_LEN];
    if (!core_signing_key_generate(key)) {
        return false;
    }

    /* O_EXCL: never silently replace a key that's already provisioned. */
    int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) {
        CORE_LOG_ERROR("cannot create key file %s: %s", path, strerror(errno));
        return false;
    }

    char hex[2 * CORE_SIGNING_KEY_LEN + 2];
    core_signing_key_to_hex(key, hex);
    hex[2 * CORE_SIGNING_KEY_LEN] = '\n';
    ssize_t w = write(fd, hex, sizeof(hex) - 1);
    bool ok = (w == (ssize_t)(sizeof(hex) - 1));
    close(fd);
    if (!ok) {
        CORE_LOG_ERROR("failed writing key file %s", path);
        unlink(path);
    }
    volatile uint8_t *k = key;
    for (size_t i = 0; i < sizeof(key); i++) k[i] = 0;
    return ok;
}

core_signing_env_result_t core_signing_key_load_from_env(uint8_t key_out[CORE_SIGNING_KEY_LEN]) {
    const char *path = getenv("CORE_SIGNING_KEY_FILE");
    if (!path) {
        return CORE_SIGNING_ENV_UNSET;
    }
    return core_signing_key_load_file(path, key_out) ? CORE_SIGNING_ENV_LOADED : CORE_SIGNING_ENV_ERROR;
}
