#include "core/mavlink/mavlink_connection.h"
#include "mavlink_internal.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/common/log.h"
#include "core/common/time.h"

/* MAVLink's per-channel parse buffers are indexed by chan, so each connection
 * gets its own channel to avoid cross-talk if more than one is ever open.
 * Channel 0 is pack-only scratch for vendored *_pack() helpers and must never
 * have a connection's signing state attached. */
static mavlink_channel_t allocate_channel(void) {
    static unsigned next_channel = 1;
    mavlink_channel_t chan = (mavlink_channel_t)next_channel;
    next_channel = next_channel % (MAVLINK_COMM_NUM_BUFFERS - 1) + 1;
    return chan;
}

static bool open_udp_socket(core_mavlink_conn_t *conn) {
    const core_mavlink_udp_config_t *udp_cfg = &conn->cfg.transport.udp;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        CORE_LOG_ERROR("socket() failed: %s", strerror(errno));
        return false;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        CORE_LOG_ERROR("fcntl(O_NONBLOCK) failed: %s", strerror(errno));
        close(fd);
        return false;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(udp_cfg->bind_port);
    const char *addr_str = (udp_cfg->bind_addr[0] != '\0') ? udp_cfg->bind_addr : "0.0.0.0";
    if (inet_pton(AF_INET, addr_str, &bind_addr.sin_addr) != 1) {
        CORE_LOG_ERROR("invalid bind address: %s", addr_str);
        close(fd);
        return false;
    }

    if (bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        CORE_LOG_ERROR("bind() failed on %s:%u: %s", addr_str, udp_cfg->bind_port, strerror(errno));
        close(fd);
        return false;
    }

    conn->sock_fd = fd;
    conn->peer_known = false;
    memset(&conn->peer_addr, 0, sizeof(conn->peer_addr));

    /* If a remote peer was configured explicitly, trust it; otherwise we'll
     * learn the peer address from the first inbound packet (standard SITL
     * pattern, since UDP has no notion of "connected" on its own). */
    if (udp_cfg->remote_port != 0) {
        struct sockaddr_in peer;
        memset(&peer, 0, sizeof(peer));
        peer.sin_family = AF_INET;
        peer.sin_port = htons(udp_cfg->remote_port);
        const char *remote_str = (udp_cfg->remote_addr[0] != '\0') ? udp_cfg->remote_addr : "127.0.0.1";
        if (inet_pton(AF_INET, remote_str, &peer.sin_addr) == 1) {
            conn->peer_addr = peer;
            conn->peer_known = true;
        }
    }

    return true;
}

core_mavlink_conn_t *core_mavlink_conn_open(const core_mavlink_conn_config_t *cfg) {
    if (!cfg) {
        return NULL;
    }

    if (cfg->type == CORE_MAVLINK_TRANSPORT_SERIAL) {
        CORE_LOG_ERROR("serial transport not yet implemented");
        return NULL;
    }

    if (cfg->type != CORE_MAVLINK_TRANSPORT_UDP) {
        CORE_LOG_ERROR("unknown mavlink transport type %d", (int)cfg->type);
        return NULL;
    }

    core_mavlink_conn_t *conn = calloc(1, sizeof(*conn));
    if (!conn) {
        CORE_LOG_ERROR("out of memory opening mavlink connection");
        return NULL;
    }

    conn->cfg = *cfg;
    conn->sock_fd = -1;
    conn->state = CORE_MAVLINK_CONN_DISCONNECTED;
    conn->chan = allocate_channel();

    if (!open_udp_socket(conn)) {
        free(conn);
        return NULL;
    }

    core_mavlink_internal_signing_attach(conn);

    conn->state = CORE_MAVLINK_CONN_CONNECTING;
    conn->last_reconnect_attempt_ms = core_now_ms();
    return conn;
}

void core_mavlink_conn_close(core_mavlink_conn_t *conn) {
    if (!conn) {
        return;
    }
    if (conn->sock_fd >= 0) {
        close(conn->sock_fd);
    }
    core_mavlink_internal_signing_detach(conn);
    free(conn);
}

core_mavlink_conn_state_t core_mavlink_conn_get_state(const core_mavlink_conn_t *conn) {
    if (!conn) {
        return CORE_MAVLINK_CONN_DISCONNECTED;
    }
    return conn->state;
}

static void drop_connection(core_mavlink_conn_t *conn) {
    if (conn->sock_fd >= 0) {
        close(conn->sock_fd);
        conn->sock_fd = -1;
    }
    conn->state = CORE_MAVLINK_CONN_DISCONNECTED;
    conn->peer_known = false;
}

int core_mavlink_conn_poll(core_mavlink_conn_t *conn) {
    if (!conn) {
        return -1;
    }

    uint64_t now = core_now_ms();

    if (conn->state == CORE_MAVLINK_CONN_DISCONNECTED) {
        if (now - conn->last_reconnect_attempt_ms >= conn->cfg.reconnect_interval_ms) {
            conn->last_reconnect_attempt_ms = now;
            if (open_udp_socket(conn)) {
                conn->state = CORE_MAVLINK_CONN_CONNECTING;
            } else {
                CORE_LOG_WARN("mavlink reconnect attempt failed");
            }
        }
        return 0;
    }

    int decoded = 0;
    uint8_t buf[512];

    /* Bounded so one very chatty peer can't starve the caller's main loop. */
    for (int reads = 0; reads < 64; reads++) {
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        ssize_t n = recvfrom(conn->sock_fd, buf, sizeof(buf), MSG_DONTWAIT,
                              (struct sockaddr *)&from_addr, &from_len);
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                CORE_LOG_WARN("mavlink recvfrom error: %s", strerror(errno));
            }
            break;
        }

        if (!conn->peer_known) {
            conn->peer_addr = from_addr;
            conn->peer_known = true;
        }

        for (ssize_t i = 0; i < n; i++) {
            mavlink_message_t msg;
            /* Use the framing API, not parse_char's error recovery: the latter
             * reuses a rejected frame's last byte as STX if it is 0xFD. A
             * replayed signature can legitimately end in 0xFD, seeding a
             * phantom frame that consumes the next genuine heartbeat. */
            uint8_t framing = mavlink_frame_char(conn->chan, buf[i], &msg, &conn->parse_status);
            bool got_msg = framing == MAVLINK_FRAMING_OK;
            core_mavlink_internal_signing_account(conn, got_msg);
            if (got_msg) {
                decoded++;
                conn->last_rx_ms = now;
                if (conn->state == CORE_MAVLINK_CONN_CONNECTING) {
                    conn->state = CORE_MAVLINK_CONN_CONNECTED;
                    CORE_LOG_INFO("mavlink connection established");
                }
                core_mavlink_internal_decode_into_telemetry(conn, &msg);
            }
        }

        if (conn->parse_status.packet_rx_drop_count != conn->last_seen_packet_rx_drop_count) {
            CORE_LOG_WARN("mavlink packet drop count now %u", conn->parse_status.packet_rx_drop_count);
            conn->last_seen_packet_rx_drop_count = conn->parse_status.packet_rx_drop_count;
        }
    }

    /* Re-read the clock: a heartbeat decoded above can be stamped later than the
     * `now` taken on entry, which would underflow this unsigned subtraction and
     * look like a timeout. */
    now = core_now_ms();
    if (conn->state == CORE_MAVLINK_CONN_CONNECTED &&
        conn->last_heartbeat_rx_ms != 0 &&
        now - conn->last_heartbeat_rx_ms > conn->cfg.heartbeat_timeout_ms) {
        CORE_LOG_WARN("mavlink heartbeat timeout, dropping connection");
        drop_connection(conn);
        conn->last_reconnect_attempt_ms = now;
    }

    return decoded;
}

void core_mavlink_conn_get_telemetry(const core_mavlink_conn_t *conn, core_telemetry_t *out) {
    if (!conn || !out) {
        return;
    }
    *out = conn->telemetry;
}
