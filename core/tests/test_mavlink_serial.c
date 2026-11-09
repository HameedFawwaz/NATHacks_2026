/*
 * Byte-stream transports: the caller-supplied stream backend (what the ESP32
 * build uses over its UART) and the POSIX tty backend, exercised over a real
 * pseudo-terminal. No hardware needed. Like test_link_signing.c this plays the
 * far end with the vendored MAVLink headers directly.
 *
 * Covers frames split across reads (including inside the 13-byte signature),
 * one whole frame per outbound write, signed-link rejection counting, and a
 * hangup dropping the connection instead of spinning.
 */
#define _XOPEN_SOURCE 600
#define _DARWIN_C_SOURCE
#define _DEFAULT_SOURCE

#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common/mavlink.h"

#include "core/mavlink/mavlink.h"

#define PEER_TX_CHAN 0
#define PEER_RX_CHAN 14

static const uint8_t KEY[32] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 11, 12, 13, 14, 15, 16,
                                17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

static uint64_t signing_ts_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t usec = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    return (usec - 1420070400ULL * 1000000ULL) / 10ULL;
}

/* --- in-memory stream: what the test feeds core, and what core wrote --- */

typedef struct {
    uint8_t in[4096];
    size_t in_len, in_pos;
    size_t chunk;      /* bytes handed over per read call, to split frames */
    bool fail_reads;

    uint8_t out[4096];
    size_t out_len;
    int writes;
    size_t write_lens[64];
} mem_stream_t;

static int mem_read(void *ctx, uint8_t *buf, size_t len) {
    mem_stream_t *m = ctx;
    if (m->fail_reads) {
        return -1;
    }
    size_t avail = m->in_len - m->in_pos;
    size_t n = avail < m->chunk ? avail : m->chunk;
    if (n > len) n = len;
    memcpy(buf, m->in + m->in_pos, n);
    m->in_pos += n;
    return (int)n;
}

static bool mem_write(void *ctx, const uint8_t *buf, size_t len) {
    mem_stream_t *m = ctx;
    assert(m->out_len + len <= sizeof(m->out));
    memcpy(m->out + m->out_len, buf, len);
    m->out_len += len;
    if (m->writes < 64) m->write_lens[m->writes] = len;
    m->writes++;
    return true;
}

static void mem_push(mem_stream_t *m, const uint8_t *buf, size_t len) {
    assert(m->in_len + len <= sizeof(m->in));
    memcpy(m->in + m->in_len, buf, len);
    m->in_len += len;
}

static core_mavlink_conn_t *open_stream(mem_stream_t *m, const uint8_t *key) {
    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_STREAM;
    cfg.transport.stream.read = mem_read;
    cfg.transport.stream.write = mem_write;
    cfg.transport.stream.ctx = m;
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 0;
    cfg.heartbeat_timeout_ms = 60000;
    if (key) {
        cfg.signing_enabled = true;
        memcpy(cfg.signing_key, key, 32);
    }
    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    assert(conn != NULL);
    return conn;
}

/* Peer-side signing on this file's private vendored channel array. */
static mavlink_signing_t g_tx_signing;
static void peer_sign_with(const uint8_t *key) {
    mavlink_status_t *tx = mavlink_get_channel_status(PEER_TX_CHAN);
    tx->signing = NULL;
    if (key) {
        memset(&g_tx_signing, 0, sizeof(g_tx_signing));
        memcpy(g_tx_signing.secret_key, key, 32);
        g_tx_signing.link_id = 5;
        g_tx_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
        g_tx_signing.timestamp = signing_ts_now();
        tx->signing = &g_tx_signing;
    }
}

static uint16_t build_heartbeat(uint8_t *buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, PEER_TX_CHAN, &msg, MAV_TYPE_QUADROTOR,
                                    MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED, 5, MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

static uint16_t build_gps(uint8_t *buf, int32_t lat) {
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, PEER_TX_CHAN, &msg, 123456, 3, lat, -1223974560,
                                      10000, 150, 150, 0, 0, 10, 0, 0, 0, 0, 0, 0);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

/* Counts whole, valid frames in buf with a fresh parser channel. */
static int count_frames(const uint8_t *buf, size_t len, uint32_t want_msgid) {
    mavlink_status_t *st = mavlink_get_channel_status(PEER_RX_CHAN);
    memset(st, 0, sizeof(*st));
    int n = 0;
    for (size_t i = 0; i < len; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(PEER_RX_CHAN, buf[i], &msg, &status) && msg.msgid == want_msgid) {
            n++;
        }
    }
    return n;
}

static void pump(core_mavlink_conn_t *conn, int iters) {
    for (int i = 0; i < iters; i++) {
        core_mavlink_conn_poll(conn);
    }
}

static void test_stream_reassembles_split_frames(void) {
    for (size_t chunk = 1; chunk <= 9; chunk++) {
        mem_stream_t m;
        memset(&m, 0, sizeof(m));
        m.chunk = chunk;
        peer_sign_with(NULL);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint8_t noise[] = {0x00, 0xFD, 0x42, 0x13}; /* e.g. the ESP32 ROM boot banner */
        mem_push(&m, noise, sizeof(noise));
        mem_push(&m, buf, build_heartbeat(buf));
        mem_push(&m, buf, build_gps(buf, 473977420));

        core_mavlink_conn_t *conn = open_stream(&m, NULL);
        /* poll reads at most 64 chunks per call, so frames span calls */
        pump(conn, 200);
        core_telemetry_t t;
        core_mavlink_conn_get_telemetry(conn, &t);
        assert(t.heartbeat.valid && t.heartbeat.custom_mode == 5);
        assert(t.gps_raw.valid && t.gps_raw.lat == 473977420);
        assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED);

        /* Outbound: exactly one whole frame per write call. */
        assert(core_mavlink_send_heartbeat(conn));
        assert(core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_INFO, "split test"));
        assert(m.writes == 2);
        assert(count_frames(m.out, m.write_lens[0], MAVLINK_MSG_ID_HEARTBEAT) == 1);
        assert(count_frames(m.out + m.write_lens[0], m.write_lens[1], MAVLINK_MSG_ID_STATUSTEXT) == 1);
        core_mavlink_conn_close(conn);
    }
    printf("  stream: frames split across 1..9-byte reads decode; one frame per write\n");
}

static void test_stream_signed(void) {
    /* Signed frames split inside their signature block must still verify once. */
    for (size_t chunk = 1; chunk <= 13; chunk += 3) {
        mem_stream_t m;
        memset(&m, 0, sizeof(m));
        m.chunk = chunk;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        peer_sign_with(KEY);
        uint16_t hb_len = build_heartbeat(buf);
        uint8_t hb[MAVLINK_MAX_PACKET_LEN];
        memcpy(hb, buf, hb_len);
        mem_push(&m, hb, hb_len);
        mem_push(&m, buf, build_gps(buf, 111));
        mem_push(&m, hb, hb_len); /* replay */
        peer_sign_with(NULL);
        mem_push(&m, buf, build_gps(buf, 222)); /* unsigned injection */

        core_mavlink_conn_t *conn = open_stream(&m, KEY);
        pump(conn, 300);
        core_telemetry_t t;
        core_mavlink_conn_get_telemetry(conn, &t);
        assert(t.heartbeat.valid);
        assert(t.gps_raw.lat == 111);
        assert(t.link_security.signed_accepted == 2);
        assert(t.link_security.rejected_replay == 1);
        assert(t.link_security.rejected_unsigned == 1);
        assert(t.link_security.rejected_bad_sig == 0);

        /* Our own output is signed. */
        assert(core_mavlink_send_heartbeat(conn));
        assert(m.writes == 1 && m.write_lens[0] == 12 + 9 + MAVLINK_SIGNATURE_BLOCK_LEN);
        assert(m.out[2] & MAVLINK_IFLAG_SIGNED);
        core_mavlink_conn_close(conn);
    }
    printf("  stream: signed frames verified once across signature splits; replay/unsigned rejected\n");
}

static void test_stream_failure_drops_connection(void) {
    mem_stream_t m;
    memset(&m, 0, sizeof(m));
    m.chunk = 64;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    peer_sign_with(NULL);
    mem_push(&m, buf, build_heartbeat(buf));
    core_mavlink_conn_t *conn = open_stream(&m, NULL);
    pump(conn, 2);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED);
    m.fail_reads = true;
    pump(conn, 1);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_DISCONNECTED);
    assert(!core_mavlink_send_heartbeat(conn)); /* nothing written while down */
    assert(m.writes == 0);
    m.fail_reads = false;
    pump(conn, 1); /* reconnect_interval_ms = 0: reopens on the next poll */
    assert(core_mavlink_conn_get_state(conn) != CORE_MAVLINK_CONN_DISCONNECTED);
    core_mavlink_conn_close(conn);

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_STREAM; /* no callbacks */
    assert(core_mavlink_conn_open(&cfg) == NULL);
    printf("  stream: read failure drops the link and it recovers; missing callbacks refused\n");
}

/* --- POSIX tty backend over a pseudo-terminal --- */

static int open_pty(char *slave, size_t slave_len) {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
        return -1;
    }
    const char *name = ptsname(master);
    assert(name && strlen(name) < slave_len);
    strcpy(slave, name);
    return master;
}

static core_mavlink_conn_t *open_serial(const char *device, uint32_t baud) {
    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_SERIAL;
    strncpy(cfg.transport.serial.device, device, sizeof(cfg.transport.serial.device) - 1);
    cfg.transport.serial.baud_rate = baud;
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 100000; /* don't reopen during the hangup check */
    cfg.heartbeat_timeout_ms = 60000;
    return core_mavlink_conn_open(&cfg);
}

static void test_posix_serial_over_pty(void) {
    char slave[128];
    int master = open_pty(slave, sizeof(slave));
    if (master < 0) {
        printf("  serial: SKIPPED, no pseudo-terminals available\n");
        return;
    }
    assert(open_serial(slave, 12345) == NULL);              /* unsupported baud */
    assert(open_serial("/dev/does-not-exist-core", 115200) == NULL);

    core_mavlink_conn_t *conn = open_serial(slave, 115200);
    assert(conn != NULL);
    assert(open_serial(slave, 115200) == NULL); /* second opener refused */

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    peer_sign_with(NULL);
    uint16_t len = build_heartbeat(buf);
    /* Write in 3-byte pieces with polls in between, as a UART delivers them. */
    for (uint16_t off = 0; off < len; off += 3) {
        uint16_t n = (uint16_t)(len - off < 3 ? len - off : 3);
        assert(write(master, buf + off, n) == n);
        usleep(2000);
        core_mavlink_conn_poll(conn);
    }
    for (int i = 0; i < 50; i++) {
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.heartbeat.valid && t.heartbeat.custom_mode == 5);

    /* Raw mode: bytes like 0x0A/0x0D inside our frames must arrive untouched. */
    assert(core_mavlink_send_named_value_float(conn, "TRUST_NAV", 0.5f));
    assert(core_mavlink_send_heartbeat(conn));
    uint8_t rx[512];
    size_t got = 0;
    for (int i = 0; i < 100 && got < 12 + 18 + 12 + 9; i++) {
        struct pollfd p = {.fd = master, .events = POLLIN};
        if (poll(&p, 1, 10) > 0) {
            ssize_t n = read(master, rx + got, sizeof(rx) - got);
            if (n > 0) got += (size_t)n;
        }
    }
    assert(count_frames(rx, got, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) == 1);
    assert(count_frames(rx, got, MAVLINK_MSG_ID_HEARTBEAT) == 1);

    /* Hangup: the board's cable is pulled. */
    close(master);
    for (int i = 0; i < 20 && core_mavlink_conn_get_state(conn) != CORE_MAVLINK_CONN_DISCONNECTED; i++) {
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_DISCONNECTED);
    core_mavlink_conn_close(conn);
    printf("  serial: pty fragments decode, raw output intact, second opener refused, hangup drops link\n");
}

int main(void) {
    printf("test_mavlink_serial\n");
    test_stream_reassembles_split_frames();
    test_stream_signed();
    test_stream_failure_drops_connection();
    test_posix_serial_over_pty();
    printf("all serial transport tests passed\n");
    return 0;
}
