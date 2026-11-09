#ifndef CORE_MAVLINK_SEND_H
#define CORE_MAVLINK_SEND_H

#include <stdbool.h>
#include <stdint.h>

#include "core/common/trust_score.h"
#include "core/mavlink/mavlink_connection.h"

/* ArduPilot-dialect command id, for matching against core_command_ack_t. */
#define CORE_MAV_CMD_SET_EKF_SOURCE_SET 42007

/* Mirrors MAV_RESULT, for reading core_command_ack_t. */
typedef enum {
    CORE_MAV_RESULT_ACCEPTED = 0,
    CORE_MAV_RESULT_TEMPORARILY_REJECTED = 1,
    CORE_MAV_RESULT_DENIED = 2,
    CORE_MAV_RESULT_UNSUPPORTED = 3,
    CORE_MAV_RESULT_FAILED = 4
} core_mav_result_t;

/* Mirrors MAV_SEVERITY_* without exposing the vendored enum. */
typedef enum {
    CORE_MAV_SEVERITY_EMERGENCY = 0,
    CORE_MAV_SEVERITY_ALERT = 1,
    CORE_MAV_SEVERITY_CRITICAL = 2,
    CORE_MAV_SEVERITY_ERROR = 3,
    CORE_MAV_SEVERITY_WARNING = 4,
    CORE_MAV_SEVERITY_NOTICE = 5,
    CORE_MAV_SEVERITY_INFO = 6,
    CORE_MAV_SEVERITY_DEBUG = 7
} core_mav_severity_t;

/* All return false (and log a warning) on failure - e.g. no known peer yet, or a write error. None of them block or crash. */

bool core_mavlink_send_heartbeat(core_mavlink_conn_t *conn);

/* text longer than MAVLink's STATUSTEXT limit (50 bytes) is truncated. */
bool core_mavlink_send_statustext(core_mavlink_conn_t *conn, core_mav_severity_t severity, const char *text);

/* name longer than MAVLink's NAMED_VALUE_FLOAT limit (10 bytes) is truncated. */
bool core_mavlink_send_named_value_float(core_mavlink_conn_t *conn, const char *name, float value);

/*
 * Publishes score as NAMED_VALUE_FLOAT messages: TRUST and TRUST_CRY always,
 * and TRUST_NAV, TRUST_LNK and TRUST_HW only while their producer is live (see
 * core_trust_component_live), so a missing detector reads as no data rather
 * than as fully trusted.
 */
bool core_mavlink_publish_trust_score(core_mavlink_conn_t *conn, const core_trust_score_t *score);

/* Publishes SIG_UNSGN, SIG_BADSG and SIG_REPLY only when signing is enabled. */
bool core_mavlink_publish_link_security(core_mavlink_conn_t *conn, const core_link_security_t *security);

/*
 * Commands to the autopilot. These also return false until a heartbeat has
 * identified the vehicle. true only means the command was sent: confirm the
 * mode from the heartbeat's custom_mode, and the source set from command_ack.
 */

/* MAV_CMD_DO_SET_MODE. custom_mode is firmware-specific, e.g. ArduCopter's LOITER = 5. */
bool core_mavlink_send_set_mode(core_mavlink_conn_t *conn, uint32_t custom_mode);

/* MAV_CMD_SET_EKF_SOURCE_SET: selects ArduPilot's EK3_SRC<source_set>_* parameters (1-3). */
bool core_mavlink_send_set_ekf_source_set(core_mavlink_conn_t *conn, uint8_t source_set);

/*
 * Requests the flight controller stream a given message (by its MAVLink
 * message ID, e.g. 116 for SCALED_IMU2) at a given interval. interval_us <= 0
 * requests the default rate; a positive value requests that exact interval.
 * Targets the vehicle identified from its heartbeat (see
 * core_mavlink_send_set_mode's doc above) - also returns false until that's
 * happened.
 *
 * Detection logic (nav/, link/, ...) needs IMU/baro data at a real sample
 * rate to do dead-reckoning accurately - SITL/MAVProxy's default forwarding
 * rate for SCALED_IMU2 is only a few Hz, far too coarse for that. Callers
 * should request at least 50Hz (interval_us=20000) for SCALED_IMU2.
 */
bool core_mavlink_request_message_interval(core_mavlink_conn_t *conn, uint32_t message_id, int32_t interval_us);

#endif /* CORE_MAVLINK_SEND_H */
