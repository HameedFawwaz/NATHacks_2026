#ifndef CORE_NAV_GNSS_ANOMALY_H
#define CORE_NAV_GNSS_ANOMALY_H

#include <stdbool.h>
#include <stdint.h>

#include "core/mavlink/mavlink_telemetry.h"
#include "core/nav/attitude_filter.h"
#include "core/nav/nav_config.h"
#include "core/nav/nav_kf.h"

typedef enum {
    CORE_NAV_FLAG_JUMP           = 1u << 0,
    CORE_NAV_FLAG_DR_DIVERGENCE  = 1u << 1,
    CORE_NAV_FLAG_SLOW_DRIFT     = 1u << 2,
    CORE_NAV_FLAG_EKF_DIVERGENCE = 1u << 3,
    CORE_NAV_FLAG_TIMESTAMP      = 1u << 4,
    CORE_NAV_FLAG_EPH_ABNORMAL   = 1u << 5,
    CORE_NAV_FLAG_GPS_UNHEALTHY  = 1u << 6, /* no/weak fix_type, or no GPS message in too long (possible jamming) */
    CORE_NAV_FLAG_IMU_LOST       = 1u << 7  /* IMU stream stale - can't verify GPS right now, not necessarily spoofed */
} core_nav_flag_t;

typedef struct {
    float score_jump, score_dr, score_drift, score_ekf, score_time, score_eph, score_health;
    float nav_trust; /* fused + slew-limited - what gets published */
    uint32_t flags;
    uint64_t last_update_ms;
} core_nav_result_t;

typedef struct {
    core_attitude_filter_t attitude;
    core_nav_kf_set_t kf_short;
    core_nav_kf_set_t kf_long;

    /* Only ever updated from a healthy fix (fix_type >= 2) - a no-fix
     * sample must never become the baseline for the next real fix's jump/
     * divergence/timestamp comparisons. */
    core_gps_raw_t prev_gps_raw;
    bool have_prev_gps;
    bool yaw_seeded; /* attitude yaw seeded from the first fix's course-over-ground */

    /* Consecutive non-advancing-timestamp GPS samples. A single one is
     * ambiguous (e.g. a receiver re-sending a cached fix when polled faster
     * than its native rate) - only a repeat is diagnostic of replay. */
    uint32_t timestamp_nonadvance_streak;

    /* Tracks ANY GPS_RAW_INT arrival (healthy or not), independent of
     * prev_gps_raw above, so staleness/jamming can be detected even while
     * fix_type is bad (which also means the message is arriving but
     * unhealthy, vs. not arriving at all). */
    uint64_t last_gps_message_ms;
    bool have_seen_any_gps;

    /* IMU freshness tracked against real wall-clock time, decoupled from
     * the deterministic (IMU-sample-paced) clock used for KF/slew timing -
     * see core_nav_detector_update's "now" for why. */
    uint64_t last_imu_wall_ms;
    uint64_t prev_imu_sample_ms;

    /* The seven raw sub-scores + flags, refreshed only when a new GPS sample
     * arrives. Deliberately separate from nav_trust below, which is
     * continuously tracked every call - mixing the two into one struct
     * previously caused the slew-limiter's "previous trust" read to be
     * clobbered by a freshly (and only partially) initialized result each
     * GPS tick. */
    core_nav_result_t cached_scores;
    bool have_cached_scores;

    float nav_trust; /* persisted, slew-limited published value */
    uint64_t last_slew_update_ms;
} core_nav_detector_t;

void core_nav_detector_init(core_nav_detector_t *detector, const core_nav_config_t *cfg);

/* Call on every telemetry poll tick (not just when GPS updates - the IMU/baro
 * driven filters, GPS/IMU staleness tracking, and the published trust slew
 * all need to run in between GPS fixes too). Internally only re-evaluates
 * the position-comparison checks when telemetry->gps_raw carries a new,
 * healthy (fix_type >= 2) sample. */
core_nav_result_t core_nav_detector_update(core_nav_detector_t *detector,
                                            const core_telemetry_t *telemetry,
                                            const core_nav_config_t *cfg);

#endif /* CORE_NAV_GNSS_ANOMALY_H */
