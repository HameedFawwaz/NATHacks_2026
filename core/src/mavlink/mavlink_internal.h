#ifndef CORE_SRC_MAVLINK_INTERNAL_H
#define CORE_SRC_MAVLINK_INTERNAL_H

/*
 * Private to core/src/mavlink (this directory's .c files only). Never
 * installed under include/ and never included outside this directory - this
 * is the only place vendored MAVLink types are allowed to appear.
 */

#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/mavlink.h"

#include "core/mavlink/mavlink_connection.h"
#include "core/mavlink/mavlink_telemetry.h"

struct core_mavlink_conn {
    core_mavlink_conn_config_t cfg;
    core_mavlink_conn_state_t state;

    int sock_fd; /* -1 when closed */
    bool peer_known;
    struct sockaddr_in peer_addr;

    mavlink_channel_t chan; /* dedicated parse channel, avoids cross-talk between connections */
    mavlink_status_t parse_status;
    uint32_t last_seen_packet_rx_drop_count;

    uint64_t last_rx_ms;
    uint64_t last_heartbeat_rx_ms;
    uint64_t last_reconnect_attempt_ms;

    core_telemetry_t telemetry;

    uint8_t vehicle_sysid;  /* from the autopilot's heartbeat, 0 = not seen yet */
    uint8_t vehicle_compid;
};

/* Packs msg's bytes and sends them to the known/learned peer. Returns false (and logs) on failure. */
bool core_mavlink_internal_send_raw(core_mavlink_conn_t *conn, const mavlink_message_t *msg);

/* Decodes msg into conn->telemetry, updating last_update_ms/valid as appropriate. */
void core_mavlink_internal_decode_into_telemetry(core_mavlink_conn_t *conn, const mavlink_message_t *msg);

#endif /* CORE_SRC_MAVLINK_INTERNAL_H */
