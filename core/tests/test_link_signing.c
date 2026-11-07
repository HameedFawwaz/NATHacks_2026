/*
 * Tests MAVLink 2 signing enforcement on the connection layer over loopback
 * UDP. This file plays both the legitimate operator/vehicle (signs with the
 * right key) and attackers (no signature, wrong key, replayed frames), so like
 * test_mavlink_loopback.c it may touch the vendored MAVLink headers directly.
 */
#include <arpa/inet.h>
#include <assert.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "common/mavlink.h"

#include "core/crypto/link_signing.h"
#include "core/mavlink/mavlink.h"

/* Peer channels index this test file's private vendored channel array,
 * separate from core_lib's shared channel array. */
#define PEER_TX_CHAN 0
#define PEER_RX_CHAN 15

static const uint8_t KEY_A[32] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                                  17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
static const uint8_t KEY_B[32] = {0xAA, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0xBB};

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static uint64_t signing_ts_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t usec = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    return (usec - 1420070400ULL * 1000000ULL) / 10ULL;
}

typedef struct {
    int fd;
    struct sockaddr_in conn_addr;
    mavlink_signing_t tx_signing;
    mavlink_signing_t rx_signing;
    mavlink_signing_streams_t rx_streams;
} peer_t;

static void peer_init(peer_t *p, uint16_t conn_port, const uint8_t *tx_key /* NULL = unsigned */,
                      const uint8_t *rx_key /* NULL = no rx verification */) {
    memset(p, 0, sizeof(*p));
    p->fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(p->fd >= 0);
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    assert(bind(p->fd, (struct sockaddr *)&a, sizeof(a)) == 0);

    p->conn_addr = a;
    p->conn_addr.sin_port = htons(conn_port);

    mavlink_status_t *tx = mavlink_get_channel_status(PEER_TX_CHAN);
    tx->signing = NULL;
    if (tx_key) {
        memcpy(p->tx_signing.secret_key, tx_key, 32);
        p->tx_signing.link_id = 7;
        p->tx_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
        p->tx_signing.timestamp = signing_ts_now();
        tx->signing = &p->tx_signing;
        tx->signing_streams = NULL;
    }

    mavlink_status_t *rx = mavlink_get_channel_status(PEER_RX_CHAN);
    rx->signing = NULL;
    rx->parse_state = MAVLINK_PARSE_STATE_IDLE;
    if (rx_key) {
        memcpy(p->rx_signing.secret_key, rx_key, 32);
        p->rx_signing.link_id = 0;
        p->rx_signing.flags = 0;
        p->rx_signing.timestamp = signing_ts_now();
        p->rx_signing.accept_unsigned_callback = NULL; /* reject unsigned */
        rx->signing = &p->rx_signing;
        rx->signing_streams = &p->rx_streams;
    }
}

static void peer_close(peer_t *p) {
    mavlink_get_channel_status(PEER_TX_CHAN)->signing = NULL;
    mavlink_get_channel_status(PEER_RX_CHAN)->signing = NULL;
    close(p->fd);
}

/* Builds a heartbeat on PEER_TX_CHAN (signed iff tx_key was given) into buf. Returns its length. */
static uint16_t build_heartbeat(uint8_t *buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, PEER_TX_CHAN, &msg, MAV_TYPE_QUADROTOR,
                                    MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

static void send_bytes(peer_t *p, const uint8_t *buf, uint16_t len) {
    ssize_t s = sendto(p->fd, buf, len, 0, (struct sockaddr *)&p->conn_addr, sizeof(p->conn_addr));
    assert(s == (ssize_t)len);
}

static void pump(core_mavlink_conn_t *conn, int iters) {
    for (int i = 0; i < iters; i++) {
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
}

static core_mavlink_conn_t *open_conn(uint16_t port, const uint8_t *key) {
    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "127.0.0.1", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = port;
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 1000;
    cfg.heartbeat_timeout_ms = 60000;
    if (key) {
        cfg.signing_enabled = true;
        memcpy(cfg.signing_key, key, 32);
        cfg.signing_link_id = 0;
    }
    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    assert(conn != NULL);
    return conn;
}

static core_link_security_t sec_of(core_mavlink_conn_t *conn) {
    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    return t.link_security;
}

static void test_signed_accepted(void) {
    core_mavlink_conn_t *conn = open_conn(14561, KEY_A);
    peer_t p;
    peer_init(&p, 14561, KEY_A, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 50);

    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.heartbeat.valid && "correctly signed frame must be accepted");
    assert(t.link_security.signing_enabled);
    assert(t.link_security.signed_accepted == 1);
    assert(t.link_security.rejected_unsigned == 0);
    assert(t.link_security.rejected_bad_sig == 0);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: signed frame accepted\n");
}

static void test_unsigned_rejected(void) {
    core_mavlink_conn_t *conn = open_conn(14562, KEY_A);
    peer_t p;
    peer_init(&p, 14562, NULL, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 50);

    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(!t.heartbeat.valid && "unsigned frame must be rejected when signing is on");
    assert(t.link_security.rejected_unsigned == 1);
    assert(t.link_security.signed_accepted == 0);
    assert(t.link_security.last_reject_ms != 0);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: unsigned frame rejected\n");
}

static void test_wrong_key_rejected(void) {
    core_mavlink_conn_t *conn = open_conn(14563, KEY_A);
    peer_t p;
    peer_init(&p, 14563, KEY_B, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 50);

    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(!t.heartbeat.valid && "frame signed with the wrong key must be rejected");
    assert(t.link_security.rejected_bad_sig == 1);
    assert(t.link_security.signed_accepted == 0);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: wrong-key frame rejected\n");
}

static void test_tampered_rejected(void) {
    core_mavlink_conn_t *conn = open_conn(14564, KEY_A);
    peer_t p;
    peer_init(&p, 14564, KEY_A, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    buf[10] ^= 0x01; /* flip a payload bit */
    /* Fix up nothing: either the CRC or the signature must fail. */
    send_bytes(&p, buf, len);
    pump(conn, 50);

    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(!t.heartbeat.valid && "tampered frame must not be accepted");
    assert(t.link_security.signed_accepted == 0);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: tampered frame rejected\n");
}

static void test_replay_rejected(void) {
    core_mavlink_conn_t *conn = open_conn(14565, KEY_A);
    peer_t p;
    peer_init(&p, 14565, KEY_A, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 30);
    assert(sec_of(conn).signed_accepted == 1);

    /* An attacker who recorded the frame re-sends it verbatim. */
    send_bytes(&p, buf, len);
    pump(conn, 30);
    core_link_security_t s = sec_of(conn);
    assert(s.signed_accepted == 1 && "replayed frame must not be accepted again");
    assert(s.rejected_replay == 1);

    /* A fresh frame from the real sender still works afterwards. */
    len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 30);
    assert(sec_of(conn).signed_accepted == 2);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: replayed frame rejected, fresh frame still accepted\n");
}

static void test_signing_off_unchanged(void) {
    core_mavlink_conn_t *conn = open_conn(14566, NULL);
    peer_t p;
    peer_init(&p, 14566, NULL, NULL);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 50);

    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.heartbeat.valid && "with signing off, behavior must be unchanged");
    assert(!t.link_security.signing_enabled);
    assert(t.link_security.rejected_unsigned == 0);

    peer_close(&p);
    core_mavlink_conn_close(conn);
    printf("  ok: signing disabled leaves behavior unchanged\n");
}

static bool recv_and_verify(peer_t *p, uint8_t expected_msgid) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    ssize_t n = -1;
    for (int i = 0; i < 1000 && n < 0; i++) {
        n = recv(p->fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) usleep(1000);
    }
    assert(n > 0 && "timed out waiting for outbound frame");
    mavlink_message_t m;
    mavlink_status_t st;
    for (ssize_t i = 0; i < n; i++) {
        if (mavlink_parse_char(PEER_RX_CHAN, buf[i], &m, &st)) {
            return m.msgid == expected_msgid;
        }
    }
    return false;
}

static void assert_frame(peer_t *p, uint8_t msgid, bool signed_frame, uint8_t seq) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    ssize_t n = -1;
    for (int i = 0; i < 1000 && n < 0; i++) {
        n = recv(p->fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) usleep(1000);
    }
    assert(n >= 12);
    assert(buf[0] == 0xFD);
    assert((buf[2] & 0x01) == (signed_frame ? 1 : 0));
    assert(buf[4] == seq);
    assert(buf[7] == msgid && buf[8] == 0 && buf[9] == 0);
}

static void test_connection_isolation(bool signed_first) {
    core_mavlink_conn_t *first = open_conn(14571, signed_first ? KEY_A : NULL);
    core_mavlink_conn_t *second = open_conn(14572, signed_first ? NULL : KEY_A);
    core_mavlink_conn_t *signed_conn = signed_first ? first : second;
    core_mavlink_conn_t *plain_conn = signed_first ? second : first;
    peer_t ps, pp;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    peer_init(&ps, signed_first ? 14571 : 14572, KEY_A, NULL);
    send_bytes(&ps, buf, build_heartbeat(buf));
    pump(signed_conn, 10);
    peer_init(&pp, signed_first ? 14572 : 14571, NULL, NULL);
    send_bytes(&pp, buf, build_heartbeat(buf));
    pump(plain_conn, 10);

    assert(core_mavlink_send_heartbeat(plain_conn));
    assert_frame(&pp, MAVLINK_MSG_ID_HEARTBEAT, false, 0);
    assert(core_mavlink_send_heartbeat(signed_conn));
    assert_frame(&ps, MAVLINK_MSG_ID_HEARTBEAT, true, 0);
    assert(core_mavlink_send_statustext(plain_conn, CORE_MAV_SEVERITY_INFO, "plain hello"));
    assert_frame(&pp, MAVLINK_MSG_ID_STATUSTEXT, false, 1);
    assert(core_mavlink_send_statustext(signed_conn, CORE_MAV_SEVERITY_INFO, "signed hello"));
    assert_frame(&ps, MAVLINK_MSG_ID_STATUSTEXT, true, 1);

    peer_close(&ps);
    peer_close(&pp);
    core_mavlink_conn_close(first);
    core_mavlink_conn_close(second);
    printf("  ok: connection signing isolation (signed first: %d)\n", signed_first);
}

static void test_outbound_signed(void) {
    core_mavlink_conn_t *conn = open_conn(14567, KEY_A);
    peer_t p;
    /* Receiver verifies with KEY_A. First it needs to teach conn its address with a signed heartbeat. */
    peer_init(&p, 14567, KEY_A, KEY_A);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&p, buf, len);
    pump(conn, 30);

    assert(core_mavlink_send_heartbeat(conn));
    assert(recv_and_verify(&p, MAVLINK_MSG_ID_HEARTBEAT) && "outbound frame must carry a valid signature");
    assert(core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_INFO, "signed hello"));
    assert(recv_and_verify(&p, MAVLINK_MSG_ID_STATUSTEXT));
    peer_close(&p);
    core_mavlink_conn_close(conn);

    /* A receiver with a different key must reject the connection's outbound frames.
     * (Fresh connection: the first one has learned the now-closed peer's address.) */
    conn = open_conn(14570, KEY_A);
    peer_t q;
    peer_init(&q, 14570, KEY_A, KEY_B);
    len = build_heartbeat(buf);
    send_bytes(&q, buf, len);
    pump(conn, 30);
    assert(core_mavlink_send_heartbeat(conn));
    assert(!recv_and_verify(&q, MAVLINK_MSG_ID_HEARTBEAT) && "wrong-key receiver must not accept outbound frames");
    peer_close(&q);
    core_mavlink_conn_close(conn);
    printf("  ok: outbound frames are signed (and fail under a different key)\n");
}

static void test_latency(void) {
    core_mavlink_conn_t *signed_conn = open_conn(14568, KEY_A);
    core_mavlink_conn_t *plain_conn = open_conn(14569, NULL);
    peer_t ps, pp;
    peer_init(&ps, 14568, KEY_A, NULL);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = build_heartbeat(buf);
    send_bytes(&ps, buf, len);
    pump(signed_conn, 30);
    peer_init(&pp, 14569, NULL, NULL);
    len = build_heartbeat(buf);
    send_bytes(&pp, buf, len);
    pump(plain_conn, 30);

    const int N = 2000;
    uint64_t t0 = now_ns();
    for (int i = 0; i < N; i++) assert(core_mavlink_send_named_value_float(plain_conn, "X", 1.0f));
    uint64_t plain_ns = now_ns() - t0;
    t0 = now_ns();
    for (int i = 0; i < N; i++) assert(core_mavlink_send_named_value_float(signed_conn, "X", 1.0f));
    uint64_t signed_ns = now_ns() - t0;

    double extra_us = ((double)signed_ns - (double)plain_ns) / N / 1000.0;
    printf("  latency: plain %.2f us/msg, signed %.2f us/msg (+%.2f us)\n", (double)plain_ns / N / 1000.0,
           (double)signed_ns / N / 1000.0, extra_us);
    assert(extra_us < 500.0 && "signing overhead should be negligible");

    peer_close(&ps);
    peer_close(&pp);
    core_mavlink_conn_close(signed_conn);
    core_mavlink_conn_close(plain_conn);
}

static void test_key_helpers(void) {
    uint8_t k[32], k2[32];
    char hex[65];
    assert(core_signing_key_generate(k));
    core_signing_key_to_hex(k, hex);
    assert(strlen(hex) == 64);
    assert(core_signing_key_from_hex(hex, k2));
    assert(memcmp(k, k2, 32) == 0);
    assert(!core_signing_key_from_hex("deadbeef", k2));
    assert(!core_signing_key_from_hex("zz00000000000000000000000000000000000000000000000000000000000000", k2));

    char path[] = "/tmp/core_signing_test_keyXXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0);
    close(fd);
    unlink(path);
    assert(core_signing_key_generate_file(path));
    assert(!core_signing_key_generate_file(path) && "must not overwrite an existing key");
    assert(core_signing_key_load_file(path, k2));
    unlink(path);
    printf("  ok: key helpers\n");
}

static void test_key_environment(void) {
    const char *old = getenv("CORE_SIGNING_KEY_FILE");
    char *saved = old ? strdup(old) : NULL;
    assert(!old || saved);
    uint8_t key[CORE_SIGNING_KEY_LEN];
    assert(unsetenv("CORE_SIGNING_KEY_FILE") == 0);
    assert(core_signing_key_load_from_env(key) == CORE_SIGNING_ENV_UNSET);
    assert(setenv("CORE_SIGNING_KEY_FILE", "", 1) == 0);
    assert(core_signing_key_load_from_env(key) == CORE_SIGNING_ENV_ERROR);

    char path[] = "/tmp/core_signing_envXXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0);
    assert(write(fd, "bad key\n", 8) == 8);
    assert(close(fd) == 0);
    assert(setenv("CORE_SIGNING_KEY_FILE", path, 1) == 0);
    assert(core_signing_key_load_from_env(key) == CORE_SIGNING_ENV_ERROR);
    assert(unlink(path) == 0);
    assert(core_signing_key_load_from_env(key) == CORE_SIGNING_ENV_ERROR);
    assert(core_signing_key_generate_file(path));
    assert(core_signing_key_load_from_env(key) == CORE_SIGNING_ENV_LOADED);
    uint8_t expected[CORE_SIGNING_KEY_LEN];
    assert(core_signing_key_load_file(path, expected));
    assert(memcmp(key, expected, sizeof(key)) == 0);
    assert(unlink(path) == 0);
    if (saved) {
        assert(setenv("CORE_SIGNING_KEY_FILE", saved, 1) == 0);
        free(saved);
    } else {
        assert(unsetenv("CORE_SIGNING_KEY_FILE") == 0);
    }
    printf("  ok: environment distinguishes unset, loaded, and unusable keys\n");
}

static void test_passphrase_keys(void) {
    uint8_t key[CORE_SIGNING_KEY_LEN], loaded[CORE_SIGNING_KEY_LEN];
    char hex[2 * CORE_SIGNING_KEY_LEN + 1];
    assert(!core_signing_key_from_passphrase(NULL, key));
    assert(!core_signing_key_from_passphrase("abc", NULL));
    assert(!core_signing_key_from_passphrase("", key));
    assert(!core_signing_key_from_passphrase("non-ascii\x80", key));
    assert(!core_signing_key_from_passphrase("correct horse", key));
    assert(!core_signing_key_from_passphrase("tab\there", key));
    assert(!core_signing_key_from_passphrase("control\x01", key));
    assert(!core_signing_key_from_passphrase("delete\x7f", key));
    assert(!core_signing_key_from_passphrase("newline\n", key));
    assert(core_signing_key_from_passphrase("!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~", key));
    assert(core_signing_key_from_passphrase("abc", key));
    core_signing_key_to_hex(key, hex);
    assert(strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);

    /* Verified independently with shasum -a 256 before adding these vectors. */
    const struct { size_t len; const char *digest; } vectors[] = {
        {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
        {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
        {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
        {119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
        {1000000, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"}
    };
    char *input = malloc(1000001);
    assert(input);
    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
        memset(input, 'a', vectors[i].len);
        input[vectors[i].len] = '\0';
        assert(core_signing_key_from_passphrase(input, key));
        core_signing_key_to_hex(key, hex);
        assert(strcmp(hex, vectors[i].digest) == 0);
    }
    free(input);

    char path[] = "/tmp/core_signing_passphraseXXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0);
    assert(close(fd) == 0);
    assert(unlink(path) == 0);
    assert(core_signing_key_write_file(path, key));
    assert(!core_signing_key_write_file(path, KEY_B));
    struct stat st;
    assert(stat(path, &st) == 0);
    assert((st.st_mode & 0777) == 0600);
    assert(core_signing_key_load_file(path, loaded));
    assert(memcmp(key, loaded, sizeof(key)) == 0);
    assert(unlink(path) == 0);
    printf("  ok: MAVProxy passphrase SHA-256 vectors and exclusive key writes\n");
}

int main(void) {
    printf("test_link_signing:\n");
    test_connection_isolation(true);
    test_connection_isolation(false);
    test_key_helpers();
    test_key_environment();
    test_passphrase_keys();
    test_signing_off_unchanged();
    test_signed_accepted();
    test_unsigned_rejected();
    test_wrong_key_rejected();
    test_tampered_rejected();
    test_replay_rejected();
    test_outbound_signed();
    test_latency();
    printf("test_link_signing: all assertions passed\n");
    return 0;
}
