#include "core/nav/nav_estimate.h"

#include <math.h>
#include <string.h>

#include "core/mavlink/mavlink_send.h"

/* cos(85 deg) ~ 0.087: past this the east scale is too degenerate to trust. */
#define MAX_ORIGIN_ABS_LAT_DEG 85.0

void core_nav_estimate_config_default(core_nav_estimate_config_t *cfg) {
    /* 25 samples at the requested 50 Hz; a longer silence means the filter
     * has stopped propagating and anything published would be stale. */
    cfg->imu_max_age_ms = 500;
    /* Normal flight re-anchors every dr_long_horizon_ms (8 s); only a spoof
     * the drift check rejects keeps the anchor older. With no bias tracking,
     * pure dead reckoning degrades fast: in the SITL slow-ramp run (after the
     * autopilot failed over) error versus truth stayed under ~5 m to 15 s
     * from the anchor, passed 10 m near 19 s and grew ~4 m/s after that. */
    cfg->max_dead_reckon_ms = 15000;
    /* Same bound the dashboard applies to EST_N/EST_E. */
    cfg->max_abs_m = 100000.0f;
}

void core_nav_estimator_init(core_nav_estimator_t *estimator) {
    memset(estimator, 0, sizeof(*estimator));
}

static bool lat_lon_in_range(double lat_deg, double lon_deg) {
    return isfinite(lat_deg) && isfinite(lon_deg) && lat_deg >= -90.0 && lat_deg <= 90.0 && lon_deg >= -180.0 &&
           lon_deg <= 180.0;
}

bool core_nav_local_from_geo(double origin_lat_deg, double origin_lon_deg, double lat_deg, double lon_deg,
                             double *north_m, double *east_m) {
    if (!lat_lon_in_range(origin_lat_deg, origin_lon_deg) || !lat_lon_in_range(lat_deg, lon_deg) ||
        fabs(origin_lat_deg) > MAX_ORIGIN_ABS_LAT_DEG) {
        return false;
    }
    double dlon_deg = lon_deg - origin_lon_deg;
    if (dlon_deg >= 180.0) dlon_deg -= 360.0;
    if (dlon_deg < -180.0) dlon_deg += 360.0;
    const double deg_to_rad = M_PI / 180.0;
    *north_m = (lat_deg - origin_lat_deg) * deg_to_rad * CORE_NAV_EST_EARTH_RADIUS_M;
    *east_m = dlon_deg * deg_to_rad * CORE_NAV_EST_EARTH_RADIUS_M * cos(origin_lat_deg * deg_to_rad);
    return true;
}

static bool home_usable(const core_home_position_t *home) {
    if (!home->valid || (home->lat == 0 && home->lon == 0)) { /* 0,0 is ArduPilot's "not set" */
        return false;
    }
    return lat_lon_in_range((double)home->lat / 1e7, (double)home->lon / 1e7);
}

static bool age_below(uint64_t now_ms, uint64_t stamp_ms, uint32_t limit_ms) {
    /* A sample stamped slightly after now (same poll batch) is fresh, not
     * stale - see gnss_anomaly.c. */
    return now_ms <= stamp_ms || now_ms - stamp_ms < limit_ms;
}

static bool attitude_usable(const core_nav_detector_t *detector, const core_telemetry_t *telemetry,
                            const core_nav_config_t *nav_cfg, uint64_t now_ms) {
    /* Mirrors the detector's choice of which attitude rotates the KF input. */
    const core_attitude_t *att = &telemetry->attitude;
    bool fc_fresh = nav_cfg->kf_attitude_from_fc && att->valid && isfinite(att->roll) && isfinite(att->pitch) &&
                    isfinite(att->yaw) && age_below(now_ms, att->last_update_ms, nav_cfg->imu_stale_ms);
    bool core_seeded = detector->attitude.initialized && detector->yaw_seeded;
    return fc_fresh || core_seeded;
}

core_nav_estimate_t core_nav_estimator_update(core_nav_estimator_t *estimator,
                                               const core_nav_detector_t *detector,
                                               const core_telemetry_t *telemetry,
                                               const core_nav_config_t *nav_cfg,
                                               const core_nav_estimate_config_t *cfg,
                                               uint64_t now_ms) {
    core_nav_estimate_t out = {.status = CORE_NAV_EST_READY, .north_m = 0.0, .east_m = 0.0, .dead_reckon_ms = 0};
    const core_nav_kf_set_t *kf = &detector->kf_long;

    /* Gap tracking runs every tick, whatever else is missing, so a gap during
     * e.g. a home outage still forces a fresh anchor afterwards. Before the
     * first IMU sample there is nothing to integrate across, so no gap. */
    bool imu_fresh = telemetry->imu.valid && age_below(now_ms, telemetry->imu.last_update_ms, cfg->imu_max_age_ms);
    if (!imu_fresh) {
        if (estimator->imu_seen) {
            estimator->awaiting_reanchor = true;
            estimator->recovery_seen = false;
        }
    } else {
        estimator->imu_seen = true;
        if (estimator->awaiting_reanchor) {
            uint64_t anchor_ms = kf->anchored ? kf->anchor_time_ms : 0;
            if (!estimator->recovery_seen) {
                /* The gap may have been integrated in this very tick, before or
                 * after a re-anchor; only a later anchor is certainly clean. */
                estimator->recovery_seen = true;
                estimator->recovery_anchor_time_ms = anchor_ms;
            } else if (kf->anchored && anchor_ms != estimator->recovery_anchor_time_ms) {
                estimator->awaiting_reanchor = false;
                estimator->recovery_seen = false;
            }
        }
    }

    if (kf->anchored) {
        uint64_t age_ms = (now_ms > kf->anchor_time_ms) ? now_ms - kf->anchor_time_ms : 0;
        out.dead_reckon_ms = (age_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)age_ms;
    }

    if (!home_usable(&telemetry->home_position)) {
        out.status = CORE_NAV_EST_NO_HOME;
    } else if (!imu_fresh) {
        out.status = CORE_NAV_EST_IMU_STALE;
    } else if (!kf->anchored) {
        out.status = CORE_NAV_EST_NOT_ANCHORED;
    } else if (estimator->awaiting_reanchor) {
        out.status = CORE_NAV_EST_AWAIT_REANCHOR;
    } else if (!attitude_usable(detector, telemetry, nav_cfg, now_ms)) {
        out.status = CORE_NAV_EST_ATTITUDE_UNAVAILABLE;
    } else if (out.dead_reckon_ms > cfg->max_dead_reckon_ms) {
        out.status = CORE_NAV_EST_DR_EXPIRED;
    }
    if (out.status != CORE_NAV_EST_READY) {
        return out;
    }

    double anchor_north_m, anchor_east_m;
    bool projected = core_nav_local_from_geo((double)telemetry->home_position.lat / 1e7,
                                             (double)telemetry->home_position.lon / 1e7, kf->anchor_lat_deg,
                                             kf->anchor_lon_deg, &anchor_north_m, &anchor_east_m);
    double north_m = anchor_north_m + (double)kf->north.p;
    double east_m = anchor_east_m + (double)kf->east.p;
    if (!projected || !isfinite(north_m) || !isfinite(east_m) || fabs(north_m) > (double)cfg->max_abs_m ||
        fabs(east_m) > (double)cfg->max_abs_m) {
        out.status = CORE_NAV_EST_INVALID;
        return out;
    }
    out.north_m = north_m;
    out.east_m = east_m;
    return out;
}

bool core_nav_estimate_publish(core_mavlink_conn_t *conn, const core_nav_estimate_t *estimate) {
    if (estimate->status != CORE_NAV_EST_READY) {
        return false;
    }
    bool ok = core_mavlink_send_named_value_float(conn, "EST_N", (float)estimate->north_m);
    return core_mavlink_send_named_value_float(conn, "EST_E", (float)estimate->east_m) && ok;
}

const char *core_nav_estimate_status_name(core_nav_estimate_status_t status) {
    switch (status) {
    case CORE_NAV_EST_READY: return "ready";
    case CORE_NAV_EST_NO_HOME: return "no_home";
    case CORE_NAV_EST_NOT_ANCHORED: return "not_anchored";
    case CORE_NAV_EST_IMU_STALE: return "imu_stale";
    case CORE_NAV_EST_AWAIT_REANCHOR: return "await_reanchor";
    case CORE_NAV_EST_ATTITUDE_UNAVAILABLE: return "no_attitude";
    case CORE_NAV_EST_DR_EXPIRED: return "dr_expired";
    case CORE_NAV_EST_INVALID: return "invalid";
    }
    return "unknown";
}
