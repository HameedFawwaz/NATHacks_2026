#ifndef CORE_MAVLINK_TELEMETRY_H
#define CORE_MAVLINK_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Plain-C telemetry snapshots decoded from MAVLink messages. No vendored
 * MAVLink type appears anywhere in this file - consumers never need to touch
 * raw MAVLink to read telemetry.
 *
 * Each struct carries `valid` (has at least one message of this type ever
 * been received) and `last_update_ms` (core_now_ms() at last update), so
 * consumers can judge freshness themselves.
 */

typedef struct {
    uint8_t type;           /* MAV_TYPE */
    uint8_t autopilot;      /* MAV_AUTOPILOT */
    uint8_t base_mode;      /* MAV_MODE_FLAG bitmask */
    uint32_t custom_mode;
    uint8_t system_status;  /* MAV_STATE */
    uint64_t last_update_ms;
    bool valid;
} core_heartbeat_t;

typedef struct {
    int32_t lat;             /* degE7 */
    int32_t lon;              /* degE7 */
    int32_t alt_mm;
    int32_t relative_alt_mm;
    int16_t vx, vy, vz;       /* cm/s */
    uint16_t hdg_cdeg;        /* centidegrees, 0..35999, UINT16_MAX = unknown */
    uint64_t last_update_ms;
    bool valid;
} core_global_position_t;

typedef struct {
    uint8_t fix_type;    /* GPS_FIX_TYPE */
    int32_t lat;          /* degE7 */
    int32_t lon;           /* degE7 */
    int32_t alt_mm;
    uint16_t eph;         /* GPS HDOP, 100 = 1.0 */
    uint16_t epv;         /* GPS VDOP, 100 = 1.0 */
    uint16_t vel_cms;
    uint8_t satellites_visible;
    uint64_t last_update_ms;
    bool valid;
} core_gps_raw_t;

typedef struct {
    float roll, pitch, yaw;             /* rad */
    float rollspeed, pitchspeed, yawspeed; /* rad/s */
    uint64_t last_update_ms;
    bool valid;
} core_attitude_t;

typedef struct {
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint16_t noise;
    uint16_t remnoise;
    uint16_t rxerrors;
    uint16_t fixed;
    uint64_t last_update_ms;
    bool valid;
} core_radio_status_t;

typedef struct {
    core_heartbeat_t heartbeat;
    core_global_position_t global_position;
    core_gps_raw_t gps_raw;
    core_attitude_t attitude;
    core_radio_status_t radio_status;
} core_telemetry_t;

#endif /* CORE_MAVLINK_TELEMETRY_H */
