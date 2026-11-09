/*
 * The real integration loop: connects to the flight controller over MAVLink,
 * runs GNSS spoofing detection and lost-link detection on the live telemetry
 * stream, and publishes the navigation and link trust scores back over MAVLink.
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

static void print_status(const core_nav_result_t *result, const core_link_result_t *link) {
    printf("[nav_trust=%.3f] jump=%.2f dr=%.2f drift=%.2f ekf=%.2f time=%.2f eph=%.2f flags=0x%02x"
           " | [link %s trust=%.3f] loss=%.2f silent=%ums %s\n",
           result->nav_trust, result->score_jump, result->score_dr, result->score_drift, result->score_ekf,
           result->score_time, result->score_eph, result->flags, core_link_state_name(link->state),
           link->link_trust, link->loss, link->silence_ms, link->reason);
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

    printf("drone survivability core - GNSS spoofing and lost-link detection\n");
    printf("listening for MAVLink on udp 0.0.0.0:%u\n", port);
    printf("point ArduPilot SITL (optionally via core/sitl/gnss_spoof_proxy) at this port\n");

    uint64_t last_own_heartbeat_sent_ms = 0;
    uint64_t last_publish_ms = 0;
    uint64_t last_print_ms = 0;
    uint64_t last_rate_request_ms = 0;
    uint32_t prev_flags = 0;

    /* SITL/MAVProxy's default forwarding rate for SCALED_IMU2 is only a few
     * Hz - far too coarse for the dead-reckoning filter to integrate
     * accurately during real maneuvering flight. Re-requested periodically
     * (not just once) so it survives a reconnect to a flight controller
     * that reset its stream rates.
     *
     * Deliberately NOT boosting GPS_RAW_INT: the underlying simulated GPS
     * only refreshes at its own native rate (commonly 5Hz in SITL), so
     * requesting it faster just makes the autopilot re-send the same cached
     * fix with an unchanged time_usec to fill the gap - which our replay/
     * timestamp check correctly (if confusingly, self-inflictedly) flags as
     * suspicious, since a non-advancing timestamp is indistinguishable from
     * a genuine stale replay. Only request a rate at or below what the
     * underlying sensor can actually produce. */
    const uint32_t MSGID_SCALED_PRESSURE = 29;
    const uint32_t MSGID_SCALED_IMU2 = 116;
    const uint32_t MSGID_RC_CHANNELS = 65; /* carries the radio's RSSI, see link_monitor.h */

    while (g_running) {
        core_mavlink_conn_poll(conn);

        uint64_t rate_now = core_now_ms();
        if (rate_now - last_rate_request_ms >= 5000) {
            core_mavlink_request_message_interval(conn, MSGID_SCALED_IMU2, 20000);      /* 50Hz */
            core_mavlink_request_message_interval(conn, MSGID_SCALED_PRESSURE, 100000); /* 10Hz */
            core_mavlink_request_message_interval(conn, MSGID_RC_CHANNELS, 500000);     /* 2Hz */
            last_rate_request_ms = rate_now;
        }

        core_telemetry_t telemetry;
        core_mavlink_conn_get_telemetry(conn, &telemetry);

        core_nav_result_t result = core_nav_detector_update(&detector, &telemetry, &nav_cfg);
        core_trust_score_set_component(&score, CORE_TRUST_SOURCE_NAV, result.nav_trust);

        alert_new_flags(conn, prev_flags, result.flags);
        prev_flags = result.flags;

        uint64_t now = core_now_ms();

        /* While UNKNOWN nothing is reported, so autonomy ignores a link that has
         * never been seen and treats one that goes unjudgeable as stale. */
        link = core_link_monitor_update(&link_monitor, &telemetry, now);
        if (link.state != CORE_LINK_UNKNOWN) {
            core_trust_score_set_component(&score, CORE_TRUST_SOURCE_LINK, link.link_trust);
        }
        if (link.state_changed) {
            report_link_change(conn, &link);
        }

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
            print_status(&result, &link);
            last_print_ms = now;
        }

        usleep(20000); /* 20ms poll cadence */
    }

    printf("\nshutting down\n");
    core_mavlink_conn_close(conn);
    return 0;
}
