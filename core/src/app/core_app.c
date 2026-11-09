#include "core/app/core_app.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/common/time.h"

static void app_printf(const core_app_t *app, const char *fmt, ...) {
    if (!app->cfg.print) {
        return;
    }
    char line[320];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    app->cfg.print(app->cfg.print_ctx, line);
}

static void print_status(const core_app_t *app, const core_nav_result_t *result, const core_link_result_t *link,
                         const core_autonomy_t *autonomy) {
    app_printf(app,
               "[nav_trust=%.3f] jump=%.3f dr=%.3f drift=%.3f ekf=%.3f time=%.3f eph=%.3f health=%.3f flags=0x%02x"
               " | [link %s trust=%.3f] loss=%.2f silent=%ums %s | behavior=%s",
               result->nav_trust, result->score_jump, result->score_dr, result->score_drift, result->score_ekf,
               result->score_time, result->score_eph, result->score_health, result->flags,
               core_link_state_name(link->state), link->link_trust, link->loss, link->silence_ms, link->reason,
               core_autonomy_behavior_name(autonomy->behavior));
}

/* One STATUSTEXT per link state change, so operators and the log see it. */
static void report_link_change(const core_app_t *app, const core_link_result_t *link) {
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
    core_mavlink_send_statustext(app->conn, severity, text);
    app_printf(app, "%s", text);
}

static void print_estimate(const core_app_t *app, const core_nav_estimate_t *estimate) {
    if (estimate->status == CORE_NAV_EST_READY) {
        app_printf(app, "[nav_est] ready north=%.2f east=%.2f dr_age_ms=%u", estimate->north_m, estimate->east_m,
                   estimate->dead_reckon_ms);
    } else {
        app_printf(app, "[nav_est] %s (not published) dr_age_ms=%u", core_nav_estimate_status_name(estimate->status),
                   estimate->dead_reckon_ms);
    }
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

void core_app_init(core_app_t *app, core_mavlink_conn_t *conn, const core_app_config_t *cfg) {
    memset(app, 0, sizeof(*app));
    app->cfg = *cfg;
    app->conn = conn;

    core_nav_config_default(&app->nav_cfg);
    core_nav_detector_init(&app->detector, &app->nav_cfg);

    core_link_config_default(&app->link_cfg);
    core_link_monitor_init(&app->link_monitor, &app->link_cfg);

    core_nav_estimate_config_default(&app->estimate_cfg);
    core_nav_estimator_init(&app->estimator);

    core_trust_score_init(&app->score);

    app->autonomy_cfg = core_autonomy_default_config();
    core_autonomy_init(&app->autonomy, &app->autonomy_cfg, cfg->autonomy_log);
}

/* A GCS or MAVProxy sitting between us and the flight controller often
 * already asks for a baseline set of streams, but core must not depend on one
 * being there: wired directly to a flight controller's own serial port (the
 * real deployment this is for), nothing else will ever ask, and the
 * autopilot's own per-link stream rates default to zero, so silence here
 * means the detector never sees anything at all - not even GPS or position.
 * Re-requested periodically (not just once) so it survives a reconnect to a
 * flight controller that reset its stream rates.
 *
 * Deliberately NOT boosting GPS_RAW_INT past its default: the underlying
 * simulated GPS only refreshes at its own native rate (commonly 5Hz in SITL),
 * so requesting it faster just makes the autopilot re-send the same cached fix
 * with an unchanged time_usec to fill the gap - which our replay/timestamp
 * check flags when it persists. An occasional exact cached resend is ignored
 * by backup reacquisition, but a sustained non-advancing stream is still
 * stale/replay evidence. Request a rate at or below what the underlying
 * sensor can actually produce. */
static void request_rates(core_app_t *app) {
    const uint32_t MSGID_SCALED_PRESSURE = 29;
    const uint32_t MSGID_SCALED_IMU2 = 116;
    const uint32_t MSGID_GPS_RAW_INT = 24;
    const uint32_t MSGID_GLOBAL_POSITION_INT = 33;
    const uint32_t MSGID_ATTITUDE = 30;
    const uint32_t MSGID_RC_CHANNELS = 65; /* carries the telemetry radio's RSSI, see link_monitor.h */
    const uint32_t MSGID_HOME_POSITION = 242;

    /* Every 30s once they've gone out, but every second until then: they
     * can't be sent before the vehicle's first heartbeat identifies it, and
     * waiting a full 30s for the next try left the detector with no telemetry
     * at all for the first half-minute after startup. */
    uint64_t rate_now = core_now_ms();
    if (rate_now - app->last_rate_request_ms < (app->rates_requested ? 30000u : 1000u)) {
        return;
    }
    core_mavlink_conn_t *conn = app->conn;
    bool ok = core_mavlink_request_message_interval(conn, MSGID_SCALED_IMU2, 20000);       /* 50Hz */
    ok = core_mavlink_request_message_interval(conn, MSGID_SCALED_PRESSURE, 100000) && ok;  /* 10Hz */
    ok = core_mavlink_request_message_interval(conn, MSGID_GPS_RAW_INT, 200000) && ok;      /* 5Hz: native rate, see above */
    /* 10Hz: EKF output, not sensor-rate-limited */
    ok = core_mavlink_request_message_interval(conn, MSGID_GLOBAL_POSITION_INT, 100000) && ok;
    /* 10Hz: the nav detector seeds its yaw from this while disarmed */
    ok = core_mavlink_request_message_interval(conn, MSGID_ATTITUDE, 100000) && ok;
    /* 2Hz: the link monitor's jamming-vs-loss evidence */
    ok = core_mavlink_request_message_interval(conn, MSGID_RC_CHANNELS, 500000) && ok;
    /* 1Hz: EST_N/EST_E are relative to home, which moves when the vehicle arms */
    ok = core_mavlink_request_message_interval(conn, MSGID_HOME_POSITION, 1000000) && ok;
    app->rates_requested = ok;
    app->last_rate_request_ms = rate_now;
}

static void handle_recovery_control(core_app_t *app, const core_telemetry_t *telemetry, uint64_t now) {
    core_mavlink_conn_t *conn = app->conn;
    if (telemetry->recovery_control.count != app->recovery_request_count) {
        app->recovery_request_count = telemetry->recovery_control.count;
        if (telemetry->recovery_control.reset_estimate) {
            if (app->reset_ack != telemetry->recovery_control.nonce &&
                app->pending_reset != telemetry->recovery_control.nonce &&
                core_nav_detector_operator_rebaseline(&app->detector, telemetry, &app->nav_cfg, now, true)) {
                app->pending_reset = telemetry->recovery_control.nonce;
                app->reset_started_ms = now;
                app->reset_source_sent_ms = 0;
            }
        } else {
            core_autonomy_set_enabled(&app->autonomy, telemetry->recovery_control.enabled, now);
            app->recovery_ack =
                (uint32_t)telemetry->recovery_control.nonce * 2u + (app->autonomy.enabled ? 1u : 0u);
            core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_NOTICE,
                                        app->autonomy.enabled ? "Recovery enabled; detection stays on" :
                                                                "Recovery disabled; detection stays on");
        }
    }
    if (app->pending_reset != 0) {
        bool ground = telemetry->heartbeat.valid && !(telemetry->heartbeat.base_mode & 128u) &&
                      now >= telemetry->heartbeat.last_update_ms && now - telemetry->heartbeat.last_update_ms < 2000;
        const core_command_ack_t *ack = &telemetry->command_ack;
        if (!ground || now - app->reset_started_ms > 5000) {
            app->pending_reset = 0; /* never continue a ground reset after arming */
        } else if (app->reset_source_sent_ms && ack->valid && ack->command == CORE_MAV_CMD_SET_EKF_SOURCE_SET &&
                   ack->result == CORE_MAV_RESULT_ACCEPTED && ack->last_update_ms >= app->reset_source_sent_ms) {
            core_autonomy_reset_ground(&app->autonomy, telemetry, now);
            app->reset_ack = app->pending_reset;
            app->pending_reset = 0;
            core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_NOTICE, "Ground reset: GPS source confirmed");
        } else if (!app->reset_source_sent_ms || now - app->reset_source_sent_ms >= 1000) {
            core_mavlink_send_set_ekf_source_set(conn, app->autonomy_cfg.primary_source_set);
            app->reset_source_sent_ms = now;
        }
    }
}

void core_app_tick(core_app_t *app) {
    core_mavlink_conn_t *conn = app->conn;
    core_mavlink_conn_poll(conn);
    request_rates(app);

    uint64_t now = core_now_ms();

    core_telemetry_t telemetry;
    core_mavlink_conn_get_telemetry(conn, &telemetry);
    handle_recovery_control(app, &telemetry, now);

    core_nav_result_t result = core_nav_detector_update(&app->detector, &telemetry, &app->nav_cfg);
    bool backup_confirmed = app->cfg.gps_free_backup && app->autonomy.enabled && !app->autonomy.failover_failed &&
                            app->autonomy.active_source_set == app->autonomy_cfg.fallback_source_set &&
                            app->autonomy.wanted_source_set == app->autonomy_cfg.fallback_source_set;
    uint32_t previous_interruptions = app->backup_recovery.interrupted_windows;
    uint64_t previous_gps_time = app->backup_recovery.last_gps_time_usec;
    if (core_nav_detector_recover_from_backup(&app->detector, &app->backup_recovery, &telemetry, now,
                                              backup_confirmed)) {
        core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_NOTICE, "GNSS recovery: backup agreement verified");
    }
    if (app->backup_recovery.interrupted_windows > previous_interruptions) {
        app_printf(app, "[gps_recovery_blocked] blockers=0x%02x gps_us=%llu->%llu",
                   app->backup_recovery.last_blockers, (unsigned long long)previous_gps_time,
                   (unsigned long long)telemetry.gps_raw.time_usec);
    }
    core_trust_score_set_component(&app->score, CORE_TRUST_SOURCE_NAV, result.nav_trust);
    core_nav_estimate_t estimate = core_nav_estimator_update(&app->estimator, &app->detector, &telemetry,
                                                             &app->nav_cfg, &app->estimate_cfg, now);
    app->last_result = result;
    app->last_estimate = estimate;

    alert_new_flags(conn, app->prev_flags, result.flags);
    app->prev_flags = result.flags;

    /* Before autonomy, so it acts on this tick's link verdict. While UNKNOWN
     * nothing is reported: autonomy ignores a link that has never been
     * judged, and treats one that stops being judgeable as stale (untrusted)
     * after its input_stale_ms. */
    app->link = core_link_monitor_update(&app->link_monitor, &telemetry, now);
    const core_link_result_t *link = &app->link;
    if (link->state != CORE_LINK_UNKNOWN) {
        core_trust_score_set_component(&app->score, CORE_TRUST_SOURCE_LINK, link->link_trust);
    }
    if (link->state_changed) {
        report_link_change(app, link);
    }

    /* Reads the nav and link trust just set above (and hardware, once that
     * detector exists) and drives whatever response is warranted - mode
     * switches, EKF source failover, etc - over this same connection. Updates
     * score->overall_trust as a side effect. A confirmed ground-reset
     * transaction owns EKF source commands until its ACK arrives: don't
     * interleave an old automatic source retry. */
    if (app->pending_reset == 0) core_autonomy_tick(&app->autonomy, conn, &app->score, &telemetry, now);

    if (now - app->last_own_heartbeat_sent_ms >= 1000) {
        core_mavlink_send_heartbeat(conn);
        app->last_own_heartbeat_sent_ms = now;
    }

    if (now - app->last_publish_ms >= 200) {
        const core_nav_backup_recovery_t *recovery = &app->backup_recovery;
        core_mavlink_publish_trust_score(conn, &app->score);
        core_mavlink_send_named_value_float(conn, "RECOV_EN", app->autonomy.enabled ? 1.0f : 0.0f);
        core_mavlink_send_named_value_float(conn, "RECOV_CTL", app->cfg.allow_recovery_control ? 1.0f : 0.0f);
        core_mavlink_send_named_value_float(conn, "AUTO_STATE", (float)app->autonomy.behavior);
        core_mavlink_send_named_value_float(conn, "RECOV_GPS", (float)recovery->status);
        if (recovery->status >= 2 && recovery->status <= 5 && isfinite(recovery->position_error_m))
            core_mavlink_send_named_value_float(conn, "RECOV_ERR", recovery->position_error_m);
        float recovery_seconds = recovery->start_ms && now >= recovery->start_ms ?
                                 (float)(now - recovery->start_ms) / 1000.0f : 0.0f;
        core_mavlink_send_named_value_float(conn, "RECOV_SEC", recovery_seconds);
        if (app->recovery_ack != 0) core_mavlink_send_named_value_float(conn, "RECOV_ACK", (float)app->recovery_ack);
        if (app->reset_ack != 0) core_mavlink_send_named_value_float(conn, "RESET_ACK", (float)app->reset_ack);
        if (link->state != CORE_LINK_UNKNOWN) {
            core_mavlink_send_named_value_float(conn, "LNK_STATE", (float)link->state);
            core_mavlink_send_named_value_float(conn, "LNK_LOSS", link->loss);
        }
        core_mavlink_publish_link_security(conn, &telemetry.link_security);
        core_nav_estimate_publish(conn, &estimate); /* sends nothing unless usable */
        app->last_publish_ms = now;
    }

    if (now - app->last_print_ms >= 1000) {
        const core_nav_backup_recovery_t *recovery = &app->backup_recovery;
        print_status(app, &result, link, &app->autonomy);
        print_estimate(app, &estimate);
        app_printf(app, "[gps_recovery] status=%u window_ms=%llu interruptions=%u blockers=0x%02x position_m=%.2f",
                   recovery->status, (unsigned long long)(recovery->start_ms ? now - recovery->start_ms : 0),
                   recovery->interrupted_windows, recovery->last_blockers, recovery->position_error_m);
        app->last_print_ms = now;
    }
}
