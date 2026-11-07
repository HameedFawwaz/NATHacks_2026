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
    CORE_NAV_FLAG_EPH_ABNORMAL   = 1u << 5
} core_nav_flag_t;

typedef struct {
    float score_jump, score_dr, score_drift, score_ekf, score_time, score_eph;
    float nav_trust; /* fused + slew-limited - what gets published */
    uint32_t flags;
    uint64_t last_update_ms;
} core_nav_result_t;

typedef struct {
    core_attitude_filter_t attitude;
    core_nav_kf_set_t kf_short;
    core_nav_kf_set_t kf_long;

    core_gps_raw_t prev_gps_raw;
    bool have_prev_gps;

    /* The six raw sub-scores + flags, refreshed only when a new GPS sample
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
 * driven filters and the published trust slew both need to run in between
 * GPS fixes too). Internally only re-evaluates the six GPS-driven checks
 * when telemetry->gps_raw carries a new sample. */
core_nav_result_t core_nav_detector_update(core_nav_detector_t *detector,
                                            const core_telemetry_t *telemetry,
                                            const core_nav_config_t *cfg);

#endif /* CORE_NAV_GNSS_ANOMALY_H */
