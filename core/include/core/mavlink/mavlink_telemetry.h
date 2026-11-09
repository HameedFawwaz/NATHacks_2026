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

/* Only the autopilot's heartbeat lands here; GCS and companion heartbeats are ignored. */
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
    uint16_t cog_cdeg;    /* course over ground, centidegrees, UINT16_MAX = unknown */
    uint8_t satellites_visible;
    uint64_t time_usec;   /* GPS receiver's own clock (usec since boot/epoch per receiver) - distinct from last_update_ms */
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
    float xacc_mps2, yacc_mps2, zacc_mps2; /* body-frame specific force, m/s^2 */
    float xgyro_rps, ygyro_rps, zgyro_rps; /* body-frame angular rate, rad/s */
    uint64_t last_update_ms;
    bool valid;
} core_imu_t; /* from SCALED_IMU2 */

typedef struct {
    float press_abs_hpa; /* absolute pressure, hPa */
    uint64_t last_update_ms;
    bool valid;
} core_baro_t; /* from SCALED_PRESSURE */

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

/*
 * Heartbeats from the operator's ground station, the one with
 * cfg.operator_gcs_sysid. The autopilot forwards them to every other link, so
 * the companion computer hears them even though it isn't on the operator's
 * link. count only increases, so a reader can tell how many arrived since it
 * last looked.
 */
typedef struct {
    uint8_t sysid;
    uint32_t count;
    uint64_t last_update_ms;
    bool valid;
} core_gcs_heartbeat_t;

/*
 * RC_CHANNELS.rssi from the autopilot: receiver or telemetry-radio signal
 * strength, depending on its RSSI_TYPE. ArduPilot forwards no RADIO_STATUS to
 * other links, so with RSSI_TYPE=5 this is the only view a companion computer
 * gets of the telemetry radio. 0..254, 255 = not configured.
 */
typedef struct {
    uint8_t rssi;
    uint64_t last_update_ms;
    bool valid;
} core_rc_status_t;

/*
 * The autopilot's HOME_POSITION: the origin the dashboard places core's
 * EST_N/EST_E estimate against. Only accepted from the autopilot identified
 * by its heartbeat. ArduCopter moves home when it arms, so consumers must
 * re-read it rather than latch the first value.
 */
typedef struct {
    int32_t lat;      /* degE7 */
    int32_t lon;      /* degE7 */
    int32_t alt_mm;   /* MSL */
    uint64_t last_update_ms;
    bool valid;
} core_home_position_t;

/* The most recent COMMAND_ACK; a newer ack for any command overwrites it. */
typedef struct {
    uint16_t command;  /* MAV_CMD */
    uint8_t result;    /* MAV_RESULT */
    uint64_t last_update_ms;
    bool valid;
} core_command_ack_t;

/*
 * Counters for MAVLink signing enforcement. All zero while signing is off.
 * A rising rejected_* count is direct evidence of someone injecting or
 * replaying frames on the link.
 */
typedef struct {
    bool signing_enabled;
    uint32_t signed_accepted;     /* frames with a valid signature */
    uint32_t rejected_unsigned;   /* frames with no signature at all */
    uint32_t rejected_bad_sig;    /* signature present but wrong key/tampered */
    uint32_t rejected_replay;     /* valid-looking signature but stale/repeated timestamp */
    uint64_t last_reject_ms;      /* core_now_ms() of most recent rejection, 0 = never */
} core_link_security_t;

typedef struct {
    core_heartbeat_t heartbeat;
    core_global_position_t global_position;
    core_gps_raw_t gps_raw;
    core_attitude_t attitude;
    core_imu_t imu;
    core_baro_t baro;
    core_radio_status_t radio_status;
    core_command_ack_t command_ack;
    core_gcs_heartbeat_t gcs_heartbeat;
    core_rc_status_t rc_status;

    core_link_security_t link_security;

    core_home_position_t home_position;
} core_telemetry_t;

#endif /* CORE_MAVLINK_TELEMETRY_H */
