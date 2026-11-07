#include "mavlink_internal.h"

#include <string.h>
#include <time.h>

#include "core/common/log.h"
#include "core/common/time.h"

/*
 * Per-channel unsigned-frame counter. The vendored parser invokes the
 * accept-unsigned callback with only the channel status (no user pointer), so
 * we key off the status address to find our connection's counter.
 */
static core_mavlink_conn_t *g_owner_by_chan[MAVLINK_COMM_NUM_BUFFERS];

static bool reject_unsigned_cb(const mavlink_status_t *status, uint32_t msgid) {
    (void)msgid;
    for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        core_mavlink_conn_t *conn = g_owner_by_chan[i];
        if (conn && status == mavlink_get_channel_status((uint8_t)i)) {
            /* The parser also calls this after a failed signature check, with
             * last_status already set to the specific failure. Only a genuinely
             * unsigned frame arrives here with last_status still NONE. */
            if (conn->signing.last_status == MAVLINK_SIGNING_STATUS_NONE) {
                conn->telemetry.link_security.rejected_unsigned++;
                conn->telemetry.link_security.last_reject_ms = core_now_ms();
            }
            break;
        }
    }
    return false; /* never accept unsigned frames while signing is on */
}

/* MAVLink signing timestamp: 10us units since 2015-01-01 GMT. */
static uint64_t signing_timestamp_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t usec = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    const uint64_t epoch_2015_usec = 1420070400ULL * 1000000ULL;
    return usec > epoch_2015_usec ? (usec - epoch_2015_usec) / 10ULL : 0;
}

void core_mavlink_internal_signing_attach(core_mavlink_conn_t *conn) {
    if (!conn->cfg.signing_enabled) {
        return;
    }
    memset(&conn->signing, 0, sizeof(conn->signing));
    memset(&conn->signing_streams, 0, sizeof(conn->signing_streams));
    memcpy(conn->signing.secret_key, conn->cfg.signing_key, sizeof(conn->signing.secret_key));
    conn->signing.link_id = conn->cfg.signing_link_id;
    conn->signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    conn->signing.timestamp = signing_timestamp_now();
    conn->signing.accept_unsigned_callback = reject_unsigned_cb;
    conn->signing.last_status = MAVLINK_SIGNING_STATUS_NONE;

    mavlink_status_t *status = mavlink_get_channel_status(conn->chan);
    status->signing = &conn->signing;
    status->signing_streams = &conn->signing_streams;
    g_owner_by_chan[conn->chan] = conn;

    conn->telemetry.link_security.signing_enabled = true;
    CORE_LOG_INFO("mavlink signing enabled (link id %u)", conn->cfg.signing_link_id);
}

void core_mavlink_internal_signing_detach(core_mavlink_conn_t *conn) {
    if (!conn->cfg.signing_enabled) {
        return;
    }
    mavlink_status_t *status = mavlink_get_channel_status(conn->chan);
    status->signing = NULL;
    status->signing_streams = NULL;
    g_owner_by_chan[conn->chan] = NULL;
    /* Don't leave the key sitting in freed memory. */
    volatile uint8_t *k = conn->signing.secret_key;
    for (size_t i = 0; i < sizeof(conn->signing.secret_key); i++) k[i] = 0;
}

void core_mavlink_internal_signing_account(core_mavlink_conn_t *conn, bool frame_accepted) {
    if (!conn->cfg.signing_enabled) {
        return;
    }
    core_link_security_t *sec = &conn->telemetry.link_security;
    switch (conn->signing.last_status) {
    case MAVLINK_SIGNING_STATUS_OK:
        sec->signed_accepted++;
        break;
    case MAVLINK_SIGNING_STATUS_REPLAY:
    case MAVLINK_SIGNING_STATUS_OLD_TIMESTAMP:
        sec->rejected_replay++;
        sec->last_reject_ms = core_now_ms();
        break;
    case MAVLINK_SIGNING_STATUS_BAD_SIGNATURE:
    case MAVLINK_SIGNING_STATUS_NO_STREAMS:
    case MAVLINK_SIGNING_STATUS_TOO_MANY_STREAMS:
        sec->rejected_bad_sig++;
        sec->last_reject_ms = core_now_ms();
        break;
    case MAVLINK_SIGNING_STATUS_NONE:
    default:
        (void)frame_accepted;
        break;
    }
    conn->signing.last_status = MAVLINK_SIGNING_STATUS_NONE;
}
