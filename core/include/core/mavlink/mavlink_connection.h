#ifndef CORE_MAVLINK_CONNECTION_H
#define CORE_MAVLINK_CONNECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "core/mavlink/mavlink_telemetry.h"

typedef enum {
    CORE_MAVLINK_TRANSPORT_UDP = 0,
    CORE_MAVLINK_TRANSPORT_SERIAL = 1
} core_mavlink_transport_type_t;

typedef struct {
    char bind_addr[64];    /* local address to listen on, e.g. "0.0.0.0" or "127.0.0.1" */
    uint16_t bind_port;
    char remote_addr[64];  /* known peer address, or "" to learn it from a valid inbound MAVLink frame */
    uint16_t remote_port;  /* 0 = learn peer address+port from the first valid frame (typical for SITL) */
} core_mavlink_udp_config_t;

typedef struct {
    char device[64];       /* e.g. "/dev/ttyUSB0" - not yet implemented */
    uint32_t baud_rate;
} core_mavlink_serial_config_t;

typedef struct {
    core_mavlink_transport_type_t type;
    union {
        core_mavlink_udp_config_t udp;
        core_mavlink_serial_config_t serial;
    } transport;

    uint8_t system_id;             /* our MAVLink system id */
    uint8_t component_id;          /* our MAVLink component id, e.g. MAV_COMP_ID_ONBOARD_COMPUTER (191) */
    uint32_t reconnect_interval_ms; /* minimum gap between reconnect attempts */
    uint32_t heartbeat_timeout_ms;  /* no inbound heartbeat within this window => link considered lost */

    /*
     * MAVLink 2 message signing (see core/crypto/link_signing.h). Off by default
     * (a zero-initialized config behaves as before). When enabled, every
     * outbound frame is signed and every inbound frame must carry a valid,
     * non-replayed signature from the same key - anything else is dropped and
     * counted in telemetry.link_security.
     */
    bool signing_enabled;
    uint8_t signing_key[32];   /* shared secret, CORE_SIGNING_KEY_LEN bytes */
    uint8_t signing_link_id;   /* identifies this link in the signature; keep unique per link */

    /* The operator's ground station, tracked in telemetry.gcs_heartbeat. 0 means
     * 255, ArduPilot's default SYSID_MYGCS. */
    uint8_t operator_gcs_sysid;
} core_mavlink_conn_config_t;

typedef enum {
    CORE_MAVLINK_CONN_DISCONNECTED = 0,
    CORE_MAVLINK_CONN_CONNECTING,
    CORE_MAVLINK_CONN_CONNECTED
} core_mavlink_conn_state_t;

typedef struct core_mavlink_conn core_mavlink_conn_t; /* opaque */

/* Allocates and opens a connection per cfg. Returns NULL on fatal misconfiguration. */
core_mavlink_conn_t *core_mavlink_conn_open(const core_mavlink_conn_config_t *cfg);

/* Closes the connection and frees it. Safe to call with NULL. */
void core_mavlink_conn_close(core_mavlink_conn_t *conn);

core_mavlink_conn_state_t core_mavlink_conn_get_state(const core_mavlink_conn_t *conn);

/*
 * Non-blocking. Drains any available inbound data, decodes messages into the
 * cached telemetry snapshot, and drives the reconnect/heartbeat-timeout state
 * machine. Intended to be called once per main-loop tick (e.g. every 20-50ms).
 * Returns the number of MAVLink messages decoded this call, or -1 on a fatal
 * error.
 */
int core_mavlink_conn_poll(core_mavlink_conn_t *conn);

/* Copies out the latest known telemetry snapshot. */
void core_mavlink_conn_get_telemetry(const core_mavlink_conn_t *conn, core_telemetry_t *out);

#endif /* CORE_MAVLINK_CONNECTION_H */
