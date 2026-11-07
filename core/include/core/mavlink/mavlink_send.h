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

#endif /* CORE_MAVLINK_SEND_H */
