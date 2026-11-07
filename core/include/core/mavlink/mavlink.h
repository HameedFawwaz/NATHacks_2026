#ifndef CORE_MAVLINK_H
#define CORE_MAVLINK_H

/*
 * Single umbrella header for the shared MAVLink interfacing layer. Every
 * other core module (nav, link, autonomy, crypto) should include only this
 * header to get connection handling, telemetry, and send helpers - never the
 * vendored MAVLink headers directly.
 */

#include "core/mavlink/mavlink_connection.h"
#include "core/mavlink/mavlink_send.h"
#include "core/mavlink/mavlink_telemetry.h"

#endif /* CORE_MAVLINK_H */
