/*
 * The real integration loop: connects to the flight controller over MAVLink,
 * runs GNSS spoofing detection and lost-link detection on the live telemetry
 * stream, feeds both trust scores to autonomy, which commands the vehicle's
 * response, and publishes the scores back over MAVLink.
 *
 * Usage:
 *   core_main [port]      (default port: 14551)
 *
 * Point ArduPilot SITL at the same port (see core/README.md), or chain it
 * through core/sitl/gnss_spoof_proxy to inject a spoofing scenario on top of
 * a real SITL instance.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/autonomy/autonomy.h"
#include "core/common/time.h"
#include "core/common/trust_score.h"
#include "core/link/link_monitor.h"
#include "core/mavlink/mavlink.h"
#include "core/nav/nav.h"

static volatile sig_atomic_t g_running = 1;

static void handle_sigint(int signum) {
    (void)signum;
    g_running = 0;
}

static void print_status(const core_nav_result_t *result, const core_link_result_t *link,
                         const core_autonomy_t *autonomy) {
    printf("[nav_trust=%.3f] jump=%.3f dr=%.3f drift=%.3f ekf=%.3f time=%.3f eph=%.3f health=%.3f flags=0x%02x"
           " | [link %s trust=%.3f] loss=%.2f silent=%ums %s | behavior=%s\n",
           result->nav_trust, result->score_jump, result->score_dr, result->score_drift, result->score_ekf,
           result->score_time, result->score_eph, result->score_health, result->flags,
           core_link_state_name(link->state), link->link_trust, link->loss, link->silence_ms, link->reason,
           core_autonomy_behavior_name(autonomy->behavior));
    fflush(stdout);
}

/* One STATUSTEXT per link state change, so operators and the log see it. */
static void report_link_change(core_mavlink_conn_t *conn, const core_link_result_t *link) {
    core_mav_severity_t severity = CORE_MAV_SEVERITY_INFO;
    switch (link->state) {
    case CORE_LINK_JAMMED:
    case CORE_LINK_LOST: severity = CORE_MAV_SEVERITY_CRITICAL; break;
    case CORE_LINK_DEGRADED: severity = CORE_MAV_SEVERITY_WARNING; break;
    case CORE_LINK_NOMINAL:
    case CORE_LINK_UNKNOWN: break;
    }
    char text[51];
    snprintf(text, sizeof(text), "LINK %s: %s", core_link_state_name(link->state), link->reason);
    core_mavlink_send_statustext(conn, severity, text);
    printf("%s\n", text);
}

static void alert_new_flags(core_mavlink_conn_t *conn, uint32_t prev_flags, uint32_t flags) {
    uint32_t newly_set = flags & ~prev_flags;
    if (newly_set & CORE_NAV_FLAG_JUMP) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_CRITICAL, "GNSS spoof suspected: position jump");
    }
    if (newly_set & CORE_NAV_FLAG_TIMESTAMP) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_CRITICAL, "GNSS spoof suspected: replay/stale fix");
    }
    if (newly_set & CORE_NAV_FLAG_SLOW_DRIFT) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_WARNING, "GNSS spoof suspected: slow drift");
    }
    if (newly_set & CORE_NAV_FLAG_DR_DIVERGENCE) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_WARNING, "GNSS vs dead-reckoning diverging");
    }
    if (newly_set & CORE_NAV_FLAG_EKF_DIVERGENCE) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_WARNING, "GNSS vs EKF position diverging");
    }
    if (newly_set & CORE_NAV_FLAG_EPH_ABNORMAL) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_WARNING, "GNSS HDOP behavior abnormal");
    }
}

int main(int argc, char **argv) {
    uint16_t port = 14551;
    if (argc > 1) {
        port = (uint16_t)atoi(argv[1]);
    }

    signal(SIGINT, handle_sigint);

    core_mavlink_conn_config_t conn_cfg;
    memset(&conn_cfg, 0, sizeof(conn_cfg));
    conn_cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(conn_cfg.transport.udp.bind_addr, "0.0.0.0", sizeof(conn_cfg.transport.udp.bind_addr) - 1);
    conn_cfg.transport.udp.bind_port = port;
    conn_cfg.transport.udp.remote_port = 0; /* learn peer from first inbound packet */
    conn_cfg.system_id = 1;
    conn_cfg.component_id = 191; /* MAV_COMP_ID_ONBOARD_COMPUTER */
    conn_cfg.reconnect_interval_ms = 1000;
    conn_cfg.heartbeat_timeout_ms = 5000;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&conn_cfg);
    if (!conn) {
        fprintf(stderr, "failed to open mavlink connection on udp port %u\n", port);
        return 1;
    }

    core_nav_config_t nav_cfg;
    core_nav_config_default(&nav_cfg);

    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &nav_cfg);

    core_link_config_t link_cfg;
    core_link_config_default(&link_cfg);
    core_link_monitor_t link_monitor;
    core_link_monitor_init(&link_monitor, &link_cfg);
    core_link_result_t link = {0};

    core_trust_score_t score;
    core_trust_score_init(&score);

    core_autonomy_config_t autonomy_cfg = core_autonomy_default_config();
    FILE *autonomy_log = fopen("autonomy_transitions.csv", "w");
    if (!autonomy_log) {
        fprintf(stderr, "[WARN] could not open autonomy_transitions.csv, logging transitions to stderr only\n");
    }
    core_autonomy_t autonomy;
    core_autonomy_init(&autonomy, &autonomy_cfg, autonomy_log);

    printf("drone survivability core - GNSS spoofing and lost-link detection + autonomy\n");
    printf("listening for MAVLink on udp 0.0.0.0:%u\n", port);
    printf("point ArduPilot SITL (optionally via core/sitl/gnss_spoof_proxy) at this port\n");

    uint64_t last_own_heartbeat_sent_ms = 0;
    uint64_t last_publish_ms = 0;
    uint64_t last_print_ms = 0;
    uint64_t last_rate_request_ms = 0;
    uint32_t prev_flags = 0;

    /* A GCS or MAVProxy sitting between us and the flight controller often
     * already asks for a baseline set of streams, but core_main must not
     * depend on one being there: wired directly to a flight controller's own
     * serial port (the real deployment this is for), nothing else will ever
     * ask, and the autopilot's own per-link stream rates default to zero, so
     * silence here means the detector never sees anything at all - not even
     * GPS or position. Re-requested periodically (not just once) so it
     * survives a reconnect to a flight controller that reset its stream
     * rates.
     *
     * Deliberately NOT boosting GPS_RAW_INT past its default: the underlying
     * simulated GPS only refreshes at its own native rate (commonly 5Hz in
     * SITL), so requesting it faster just makes the autopilot re-send the
     * same cached fix with an unchanged time_usec to fill the gap - which our
     * replay/timestamp check correctly (if confusingly, self-inflictedly)
     * flags as suspicious, since a non-advancing timestamp is
     * indistinguishable from a genuine stale replay. Only request a rate at
     * or below what the underlying sensor can actually produce. */
    const uint32_t MSGID_SCALED_PRESSURE = 29;
    const uint32_t MSGID_SCALED_IMU2 = 116;
    const uint32_t MSGID_GPS_RAW_INT = 24;
    const uint32_t MSGID_GLOBAL_POSITION_INT = 33;
    const uint32_t MSGID_ATTITUDE = 30;
    const uint32_t MSGID_RC_CHANNELS = 65; /* carries the telemetry radio's RSSI, see link_monitor.h */
    bool rates_requested = false;

    while (g_running) {
        core_mavlink_conn_poll(conn);

        /* Every 30s once they've gone out, but every second until then: they
         * can't be sent before the vehicle's first heartbeat identifies it,
         * and waiting a full 30s for the next try left the detector with no
         * telemetry at all for the first half-minute after startup. */
        uint64_t rate_now = core_now_ms();
        if (rate_now - last_rate_request_ms >= (rates_requested ? 30000u : 1000u)) {
            bool ok = core_mavlink_request_message_interval(conn, MSGID_SCALED_IMU2, 20000);       /* 50Hz */
            ok = core_mavlink_request_message_interval(conn, MSGID_SCALED_PRESSURE, 100000) && ok;  /* 10Hz */
            ok = core_mavlink_request_message_interval(conn, MSGID_GPS_RAW_INT, 200000) && ok;      /* 5Hz: native rate, see above */
            /* 10Hz: EKF output, not sensor-rate-limited */
            ok = core_mavlink_request_message_interval(conn, MSGID_GLOBAL_POSITION_INT, 100000) && ok;
            /* 10Hz: the nav detector seeds its yaw from this while disarmed */
            ok = core_mavlink_request_message_interval(conn, MSGID_ATTITUDE, 100000) && ok;
            /* 2Hz: the link monitor's jamming-vs-loss evidence */
            ok = core_mavlink_request_message_interval(conn, MSGID_RC_CHANNELS, 500000) && ok;
            rates_requested = ok;
            last_rate_request_ms = rate_now;
        }

        uint64_t now = core_now_ms();

        core_telemetry_t telemetry;
        core_mavlink_conn_get_telemetry(conn, &telemetry);

        core_nav_result_t result = core_nav_detector_update(&detector, &telemetry, &nav_cfg);
        core_trust_score_set_component(&score, CORE_TRUST_SOURCE_NAV, result.nav_trust);

        alert_new_flags(conn, prev_flags, result.flags);
        prev_flags = result.flags;

        /* Before autonomy, so it acts on this tick's link verdict. While
         * UNKNOWN nothing is reported: autonomy ignores a link that has never
         * been judged, and treats one that stops being judgeable as stale
         * (untrusted) after its input_stale_ms. */
        link = core_link_monitor_update(&link_monitor, &telemetry, now);
        if (link.state != CORE_LINK_UNKNOWN) {
            core_trust_score_set_component(&score, CORE_TRUST_SOURCE_LINK, link.link_trust);
        }
        if (link.state_changed) {
            report_link_change(conn, &link);
        }

        /* Reads the nav and link trust just set above (and hardware, once
         * that detector exists) and drives whatever response is warranted -
         * mode switches, EKF source failover, etc - over this same
         * connection. Updates score->overall_trust as a side effect. */
        core_autonomy_tick(&autonomy, conn, &score, &telemetry, now);

        if (now - last_own_heartbeat_sent_ms >= 1000) {
            core_mavlink_send_heartbeat(conn);
            last_own_heartbeat_sent_ms = now;
        }

        if (now - last_publish_ms >= 200) {
            core_mavlink_publish_trust_score(conn, &score);
            if (link.state != CORE_LINK_UNKNOWN) {
                core_mavlink_send_named_value_float(conn, "LNK_STATE", (float)link.state);
                core_mavlink_send_named_value_float(conn, "LNK_LOSS", link.loss);
            }
            last_publish_ms = now;
        }

        if (now - last_print_ms >= 1000) {
            print_status(&result, &link, &autonomy);
            last_print_ms = now;
        }

        usleep(20000); /* 20ms poll cadence */
    }

    printf("\nshutting down\n");
    core_mavlink_conn_close(conn);
    if (autonomy_log) {
        fclose(autonomy_log);
    }
    return 0;
}
