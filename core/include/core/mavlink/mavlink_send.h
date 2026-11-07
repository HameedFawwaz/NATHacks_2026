#ifndef CORE_MAVLINK_SEND_H
#define CORE_MAVLINK_SEND_H

#include <stdbool.h>

#include "core/common/trust_score.h"
#include "core/mavlink/mavlink_connection.h"

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

/* Publishes score as four NAMED_VALUE_FLOAT messages: TRUST, TRUST_NAV, TRUST_LNK, TRUST_CRY. */
bool core_mavlink_publish_trust_score(core_mavlink_conn_t *conn, const core_trust_score_t *score);

/*
 * Requests the flight controller stream a given message (by its MAVLink
 * message ID, e.g. 116 for SCALED_IMU2) at a given interval. interval_us <= 0
 * requests the default rate; a positive value requests that exact interval.
 * Targets system/component 1,1 (MAV_COMP_ID_AUTOPILOT1) - a documented
 * single-vehicle-SITL simplification, not a general multi-vehicle API.
 *
 * Detection logic (nav/, link/, ...) needs IMU/baro data at a real sample
 * rate to do dead-reckoning accurately - SITL/MAVProxy's default forwarding
 * rate for SCALED_IMU2 is only a few Hz, far too coarse for that. Callers
 * should request at least 50Hz (interval_us=20000) for SCALED_IMU2.
 */
bool core_mavlink_request_message_interval(core_mavlink_conn_t *conn, uint32_t message_id, int32_t interval_us);

#endif /* CORE_MAVLINK_SEND_H */
