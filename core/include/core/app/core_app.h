#ifndef CORE_APP_CORE_APP_H
#define CORE_APP_CORE_APP_H

/*
 * core's main loop, independent of how it reaches the flight controller:
 * GNSS spoofing and lost-link detection on the live telemetry stream, both
 * trust scores fed to autonomy (which commands the vehicle's response), and
 * the scores published back over the same MAVLink connection.
 *
 * core_main (src/main.c) runs it over UDP on a computer; the ESP32 firmware
 * (firmware/esp32) runs it over the board's UART, wired to the flight
 * controller's (or SITL's) telemetry port. Nothing here touches the OS: the
 * caller owns the connection, the clock and the loop cadence.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "core/autonomy/autonomy.h"
#include "core/common/trust_score.h"
#include "core/link/link_monitor.h"
#include "core/mavlink/mavlink.h"
#include "core/nav/nav.h"

typedef struct {
    /* Accept the dashboard's recovery on/off and ground-reset requests. */
    bool allow_recovery_control;
    /* Allow GNSS reacquisition once a GPS-free backup source is confirmed. */
    bool gps_free_backup;
    /* Where autonomy logs its transitions as CSV; NULL = stderr. */
    FILE *autonomy_log;
    /* Receives one human-readable diagnostic line at a time (no trailing
     * newline). NULL = no diagnostics, e.g. when the only serial port carries
     * MAVLink. */
    void (*print)(void *ctx, const char *line);
    void *print_ctx;
} core_app_config_t;

typedef struct {
    core_app_config_t cfg;
    core_mavlink_conn_t *conn;

    core_nav_config_t nav_cfg;
    core_nav_detector_t detector;
    core_link_config_t link_cfg;
    core_link_monitor_t link_monitor;
    core_link_result_t link;
    core_nav_estimate_config_t estimate_cfg;
    core_nav_estimator_t estimator;
    core_nav_backup_recovery_t backup_recovery;
    core_trust_score_t score;
    core_autonomy_config_t autonomy_cfg;
    core_autonomy_t autonomy;

    /* Latest results, for callers that want to show them. */
    core_nav_result_t last_result;
    core_nav_estimate_t last_estimate;

    uint64_t last_own_heartbeat_sent_ms;
    uint64_t last_publish_ms;
    uint64_t last_print_ms;
    uint64_t last_rate_request_ms;
    bool rates_requested;
    uint32_t prev_flags;
    uint32_t recovery_request_count;
    uint32_t recovery_ack;
    uint16_t reset_ack;
    uint16_t pending_reset;
    uint64_t reset_source_sent_ms;
    uint64_t reset_started_ms;
} core_app_t;

/* conn must already be open; the app never closes it. */
void core_app_init(core_app_t *app, core_mavlink_conn_t *conn, const core_app_config_t *cfg);

/* One main-loop step: polls the connection, then detects, decides and
 * publishes. Call every ~20 ms. */
void core_app_tick(core_app_t *app);

#endif /* CORE_APP_CORE_APP_H */
