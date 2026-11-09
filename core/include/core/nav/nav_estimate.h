#ifndef CORE_NAV_ESTIMATE_H
#define CORE_NAV_ESTIMATE_H

/*
 * Core's dead-reckoned horizontal position, expressed as metres north and
 * east of the autopilot's HOME_POSITION, for publishing as the EST_N/EST_E
 * NAMED_VALUE_FLOATs the dashboard draws drift against (issue #16).
 *
 * Source: the long-horizon KF set (detector->kf_long). Its anchor is the last
 * GPS fix the detector trusted, and re-anchoring stops while its own drift
 * check shows divergence, so during a spoof it keeps dead-reckoning from the
 * pre-spoof anchor instead of following the false track. The short-horizon set
 * re-anchors every 1.5 s and absorbs a slow spoof, so it is not used.
 *
 * The KF's own origin is that moving GPS anchor, never home. The published
 * value is anchor-relative-to-home (projected with the same flat-earth formula
 * the dashboard inverts, see core_nav_local_from_geo) plus the KF's metre
 * offset from the anchor. Home is re-read on every update, so a home moved at
 * arming takes effect immediately and both ends use the same origin.
 *
 * Not GPS-independent: positions start from GPS anchors, and by default the
 * KF rotates accelerations with the flight controller's ATTITUDE, which EKF3
 * corrects slowly using GPS velocity. See core/README.md.
 */

#include <stdbool.h>
#include <stdint.h>

#include "core/mavlink/mavlink_connection.h"
#include "core/mavlink/mavlink_telemetry.h"
#include "core/nav/gnss_anomaly.h"
#include "core/nav/nav_config.h"

/* Sphere radius of the local projection. Must match
 * dashboard/internal/model/plan.go's earthRadiusM, which converts back. */
#define CORE_NAV_EST_EARTH_RADIUS_M 6371000.0

typedef enum {
    CORE_NAV_EST_READY = 0,
    CORE_NAV_EST_NO_HOME,               /* no usable HOME_POSITION from the autopilot yet */
    CORE_NAV_EST_NOT_ANCHORED,          /* filter not initialised from a trusted GPS fix yet */
    CORE_NAV_EST_IMU_STALE,             /* no recent IMU sample: the filter isn't propagating */
    CORE_NAV_EST_AWAIT_REANCHOR,        /* IMU came back after going stale; waiting for a fresh anchor */
    CORE_NAV_EST_ATTITUDE_UNAVAILABLE,  /* neither fresh FC attitude nor a seeded core attitude */
    CORE_NAV_EST_DR_EXPIRED,            /* dead-reckoning from one anchor for too long to trust */
    CORE_NAV_EST_INVALID                /* non-finite or beyond max_abs_m */
} core_nav_estimate_status_t;

typedef struct {
    uint32_t imu_max_age_ms;       /* newest IMU sample older than this = stale. default 500 */
    uint32_t max_dead_reckon_ms;   /* stop publishing this long after the last anchor. default 60000 */
    float max_abs_m;               /* |north|,|east| bound, matches the dashboard's. default 100000 */
} core_nav_estimate_config_t;

typedef struct {
    core_nav_estimate_status_t status;
    double north_m, east_m;  /* of HOME_POSITION; only meaningful when status == READY */
    uint32_t dead_reckon_ms; /* time since the filter's anchor fix */
} core_nav_estimate_t;

typedef struct {
    /* After the IMU stream goes stale, the first sample back is integrated
     * across the whole gap. Publishing resumes only after a re-anchor that
     * happened strictly after the stream was seen fresh again. */
    bool imu_seen;
    bool awaiting_reanchor;
    bool recovery_seen;
    uint64_t recovery_anchor_time_ms;
} core_nav_estimator_t;

void core_nav_estimate_config_default(core_nav_estimate_config_t *cfg);
void core_nav_estimator_init(core_nav_estimator_t *estimator);

/*
 * Call once per main-loop tick, after core_nav_detector_update. now_ms is on
 * the same clock as telemetry timestamps (core_now_ms() in core_main).
 */
core_nav_estimate_t core_nav_estimator_update(core_nav_estimator_t *estimator,
                                               const core_nav_detector_t *detector,
                                               const core_telemetry_t *telemetry,
                                               const core_nav_config_t *nav_cfg,
                                               const core_nav_estimate_config_t *cfg,
                                               uint64_t now_ms);

/*
 * Equirectangular projection about origin: north = R*dlat, east =
 * R*cos(origin_lat)*dlon, dlon wrapped to [-180,180). The exact inverse of
 * the dashboard's model.FromLocal. Returns false for non-finite or
 * out-of-range input, or an origin too near a pole for the projection.
 */
bool core_nav_local_from_geo(double origin_lat_deg, double origin_lon_deg, double lat_deg, double lon_deg,
                             double *north_m, double *east_m);

/* Sends EST_N then EST_E, and nothing at all unless estimate->status is READY.
 * Returns true only if both were sent. */
bool core_nav_estimate_publish(core_mavlink_conn_t *conn, const core_nav_estimate_t *estimate);

const char *core_nav_estimate_status_name(core_nav_estimate_status_t status);

#endif /* CORE_NAV_ESTIMATE_H */
