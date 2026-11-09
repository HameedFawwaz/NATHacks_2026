#ifndef CORE_SRC_MAVLINK_INTERNAL_H
#define CORE_SRC_MAVLINK_INTERNAL_H

/*
 * Private to core/src/mavlink (this directory's .c files only). Never
 * installed under include/ and never included outside this directory - this
 * is the only place vendored MAVLink types are allowed to appear.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The vendored helpers keep per-channel state in a function-local `static`
 * array inside a `static inline` function, which gives every .c file its own
 * private copy. Signing state is attached to a channel in one file and read by
 * the parser/sender in others, so all files in this directory must share one
 * array. Supplying our own mavlink_get_channel_status() (defined in
 * mavlink_channels.c) makes the vendored one step aside.
 */
#define MAVLINK_GET_CHANNEL_STATUS
struct __mavlink_status;
struct __mavlink_status *mavlink_get_channel_status(uint8_t chan);

#include "common/mavlink.h"

#include "core/mavlink/mavlink_connection.h"
#include "core/mavlink/mavlink_telemetry.h"

/*
 * One backend per transport type. Backends keep their own state behind
 * conn->transport, so this header needs no socket or tty types and the
 * portable files build on targets without POSIX networking (ESP32).
 */
typedef struct core_mavlink_transport_ops {
    const char *name;
    /* Opens (or reopens after a drop) the link. false = try again later. */
    bool (*open)(core_mavlink_conn_t *conn);
    /* Closes the link but keeps whatever open needs to reopen it. */
    void (*close)(core_mavlink_conn_t *conn);
    /* Frees backend state; the link is already closed. May be NULL. */
    void (*destroy)(core_mavlink_conn_t *conn);
    /* Non-blocking read: >0 bytes, 0 = nothing now, -1 = link failed. */
    int (*read)(core_mavlink_conn_t *conn, uint8_t *buf, size_t len);
    /*
     * Parser hooks for datagram backends that learn their peer (may be NULL).
     * rx_byte runs before each received byte is framed, with the parser state
     * it is about to be framed in; rx_frame_end runs when the parser returns to
     * idle, with whether a whole frame (signature-verified when signing is on)
     * was accepted. A peer must only be learned in rx_frame_end, from a frame
     * whose every byte came from one sender, never from raw reads, so unsigned,
     * forged or spliced traffic can't redirect where commands go.
     */
    void (*rx_byte)(core_mavlink_conn_t *conn, uint8_t byte, uint8_t parse_state);
    void (*rx_frame_end)(core_mavlink_conn_t *conn, bool accepted);
    /* Whether there is anywhere to send yet (e.g. a learned UDP peer). */
    bool (*can_send)(const core_mavlink_conn_t *conn);
    /* Sends one complete frame. */
    bool (*write)(core_mavlink_conn_t *conn, const uint8_t *buf, size_t len);
} core_mavlink_transport_ops_t;

#ifndef CORE_MAVLINK_POSIX_TRANSPORTS
#define CORE_MAVLINK_POSIX_TRANSPORTS 1 /* ESP-IDF builds define it to 0 */
#endif

#if CORE_MAVLINK_POSIX_TRANSPORTS
extern const core_mavlink_transport_ops_t core_mavlink_transport_udp_ops;    /* mavlink_transport_udp_posix.c */
extern const core_mavlink_transport_ops_t core_mavlink_transport_serial_ops; /* mavlink_transport_serial_posix.c */
#endif
extern const core_mavlink_transport_ops_t core_mavlink_transport_stream_ops; /* mavlink_transport_stream.c */

struct core_mavlink_conn {
    core_mavlink_conn_config_t cfg;
    core_mavlink_conn_state_t state;

    const core_mavlink_transport_ops_t *ops;
    void *transport; /* backend-private state */

    mavlink_channel_t chan; /* dedicated parse channel, avoids cross-talk between connections */
    mavlink_status_t parse_status;
    uint32_t last_seen_packet_rx_drop_count;

    uint64_t last_rx_ms;
    uint64_t last_heartbeat_rx_ms;
    uint64_t last_reconnect_attempt_ms;

    mavlink_signing_t signing;               /* only wired into the channel when cfg.signing_enabled */
    mavlink_signing_streams_t signing_streams;

    core_telemetry_t telemetry;

    uint8_t vehicle_sysid;  /* from the autopilot's heartbeat, 0 = not seen yet */
    uint8_t vehicle_compid;
};

/* Enables/disables signing on conn's channel per conn->cfg. Called from conn_open and close. */
void core_mavlink_internal_signing_attach(core_mavlink_conn_t *conn);
void core_mavlink_internal_signing_detach(core_mavlink_conn_t *conn);

/* Classifies and counts any rejection recorded by the parser since the last call. Call after each parsed byte. */
void core_mavlink_internal_signing_account(core_mavlink_conn_t *conn, bool frame_accepted);

/* Packs msg's bytes and sends them through the transport. Returns false (and logs) on failure. */
bool core_mavlink_internal_send_raw(core_mavlink_conn_t *conn, const mavlink_message_t *msg);

/* Decodes msg into conn->telemetry, updating last_update_ms/valid as appropriate. */
void core_mavlink_internal_decode_into_telemetry(core_mavlink_conn_t *conn, const mavlink_message_t *msg);

#endif /* CORE_SRC_MAVLINK_INTERNAL_H */
