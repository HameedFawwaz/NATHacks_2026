#include "core/mavlink/mavlink_connection.h"
#include "mavlink_internal.h"

#include <stdlib.h>
#include <string.h>

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

static const core_mavlink_transport_ops_t *ops_for(core_mavlink_transport_type_t type) {
    switch (type) {
#if CORE_MAVLINK_POSIX_TRANSPORTS
    case CORE_MAVLINK_TRANSPORT_UDP:
        return &core_mavlink_transport_udp_ops;
    case CORE_MAVLINK_TRANSPORT_SERIAL:
        return &core_mavlink_transport_serial_ops;
#endif
    case CORE_MAVLINK_TRANSPORT_STREAM:
        return &core_mavlink_transport_stream_ops;
    default:
        return NULL;
    }
}

core_mavlink_conn_t *core_mavlink_conn_open(const core_mavlink_conn_config_t *cfg) {
    if (!cfg) {
        return NULL;
    }

    const core_mavlink_transport_ops_t *ops = ops_for(cfg->type);
    if (!ops) {
        CORE_LOG_ERROR("mavlink transport type %d is not available in this build", (int)cfg->type);
        return NULL;
    }

    core_mavlink_conn_t *conn = calloc(1, sizeof(*conn));
    if (!conn) {
        CORE_LOG_ERROR("out of memory opening mavlink connection");
        return NULL;
    }

    conn->cfg = *cfg;
    conn->ops = ops;
    conn->state = CORE_MAVLINK_CONN_DISCONNECTED;
    conn->chan = allocate_channel();

    if (!ops->open(conn)) {
        if (ops->destroy) {
            ops->destroy(conn);
        }
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
    if (conn->state != CORE_MAVLINK_CONN_DISCONNECTED) {
        conn->ops->close(conn);
    }
    if (conn->ops->destroy) {
        conn->ops->destroy(conn);
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
    conn->ops->close(conn);
    conn->state = CORE_MAVLINK_CONN_DISCONNECTED;
}

int core_mavlink_conn_poll(core_mavlink_conn_t *conn) {
    if (!conn) {
        return -1;
    }

    uint64_t now = core_now_ms();

    if (conn->state == CORE_MAVLINK_CONN_DISCONNECTED) {
        if (now - conn->last_reconnect_attempt_ms >= conn->cfg.reconnect_interval_ms) {
            conn->last_reconnect_attempt_ms = now;
            if (conn->ops->open(conn)) {
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
        int n = conn->ops->read(conn, buf, sizeof(buf));
        if (n < 0) {
            CORE_LOG_WARN("mavlink %s link failed, dropping connection", conn->ops->name);
            drop_connection(conn);
            conn->last_reconnect_attempt_ms = core_now_ms();
            return decoded;
        }
        if (n == 0) {
            break;
        }

        /* A stream read can end mid-frame: the parser keeps its state on
         * conn->chan between reads, so the rest of the frame completes it on a
         * later call. */
        for (int i = 0; i < n; i++) {
            mavlink_message_t msg;
            if (conn->ops->rx_byte) {
                conn->ops->rx_byte(conn, buf[i], conn->parse_status.parse_state);
            }
            /* Use the framing API, not parse_char's error recovery: the latter
             * reuses a rejected frame's last byte as STX if it is 0xFD. A
             * replayed signature can legitimately end in 0xFD, seeding a
             * phantom frame that consumes the next genuine heartbeat. */
            uint8_t framing = mavlink_frame_char(conn->chan, buf[i], &msg, &conn->parse_status);
            bool got_msg = framing == MAVLINK_FRAMING_OK;
            core_mavlink_internal_signing_account(conn, got_msg);
            if (conn->parse_status.parse_state == MAVLINK_PARSE_STATE_IDLE && conn->ops->rx_frame_end) {
                conn->ops->rx_frame_end(conn, got_msg);
            }
            if (got_msg) {
                decoded++;
                conn->last_rx_ms = now;
                if (conn->state == CORE_MAVLINK_CONN_CONNECTING) {
                    conn->state = CORE_MAVLINK_CONN_CONNECTED;
                    /* A fresh window for the next heartbeat. Left at the one
                     * from before an outage, the timeout below would drop
                     * this connection again in the same poll unless the very
                     * first frame back happened to be a heartbeat. */
                    if (conn->last_heartbeat_rx_ms != 0) {
                        conn->last_heartbeat_rx_ms = now;
                    }
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
