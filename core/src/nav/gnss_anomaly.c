#include "core/nav/gnss_anomaly.h"

#include <math.h>
#include <string.h>

#include "core/common/time.h"

#define METERS_PER_DEG_LAT 111320.0

static float linear_score(float value, float warn, float reject) {
    if (reject <= warn) {
        return (value <= warn) ? 1.0f : 0.0f;
    }
    float score = (reject - value) / (reject - warn);
    if (score < 0.0f) score = 0.0f;
    if (score > 1.0f) score = 1.0f;
    return score;
}

static float distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    double dlat_m = (lat2_deg - lat1_deg) * METERS_PER_DEG_LAT;
    double mean_lat_rad = (lat1_deg + lat2_deg) * 0.5 * M_PI / 180.0;
    double dlon_m = (lon2_deg - lon1_deg) * METERS_PER_DEG_LAT * cos(mean_lat_rad);
    return (float)sqrt(dlat_m * dlat_m + dlon_m * dlon_m);
}

static float kf_divergence_m(const core_nav_kf_set_t *kf, const core_gps_raw_t *gps) {
    double kf_lat, kf_lon, kf_alt;
    core_nav_kf_set_get_position(kf, &kf_lat, &kf_lon, &kf_alt);
    return distance_m(kf_lat, kf_lon, (double)gps->lat / 1e7, (double)gps->lon / 1e7);
}

static float cog_cdeg_to_rad(uint16_t cog_cdeg) {
    return (cog_cdeg == UINT16_MAX) ? 0.0f : (float)cog_cdeg * (float)M_PI / 18000.0f;
}

/* Prefers the GPS receiver's own clock (time_usec) over our wall-clock
 * arrival time for computing the interval between two fixes: two honest
 * fixes can arrive back-to-back closer together than the receiver's real
 * update interval (scheduling jitter), which would otherwise inflate the
 * implied speed between them. Falls back to arrival time only when
 * time_usec is unavailable or non-advancing (handled separately by the
 * timestamp check). */
static float fix_interval_seconds(const core_gps_raw_t *gps, const core_gps_raw_t *prev) {
    if (gps->time_usec != 0 && prev->time_usec != 0 && gps->time_usec > prev->time_usec) {
        return (float)(gps->time_usec - prev->time_usec) / 1e6f;
    }
    return (float)(gps->last_update_ms - prev->last_update_ms) / 1000.0f;
}

void core_nav_detector_init(core_nav_detector_t *detector, const core_nav_config_t *cfg) {
    memset(detector, 0, sizeof(*detector));
    core_attitude_filter_init(&detector->attitude, cfg->attitude_tau_s);
    core_nav_kf_set_init(&detector->kf_short, cfg->dr_short_horizon_ms, cfg->baro_ground_ref_samples);
    core_nav_kf_set_init(&detector->kf_long, cfg->dr_long_horizon_ms, cfg->baro_ground_ref_samples);
    detector->nav_trust = 1.0f;
}

core_nav_result_t core_nav_detector_update(core_nav_detector_t *detector,
                                            const core_telemetry_t *telemetry,
                                            const core_nav_config_t *cfg) {
    /* Paced by the IMU sample clock (like nav_kf.c/attitude_filter.c), not a
     * fresh core_now_ms() call, so the whole detector - reanchor cadence and
     * publish slew included - is deterministically drivable by synthetic
     * telemetry timestamps in tests. Falls back to wall-clock only before
     * the first IMU sample has ever arrived. */
    uint64_t now = telemetry->imu.valid ? telemetry->imu.last_update_ms : core_now_ms();

    /* IMU freshness, tracked against REAL wall-clock time, decoupled from
     * `now` above. If the IMU stream itself dies, `now` (and thus the KF's
     * predict step and its reanchor-due check) would otherwise freeze in
     * whatever state they were in, letting a continuing real GPS trajectory
     * diverge from the now-static KF prediction at a rate bounded only by
     * real elapsed time - indistinguishable from a growing spoof. Synthetic
     * tests run in well under a second of real time regardless of how much
     * simulated time they cover, so this doesn't affect their determinism. */
    bool new_imu_sample = telemetry->imu.valid &&
                          (detector->prev_imu_sample_ms == 0 ||
                           telemetry->imu.last_update_ms != detector->prev_imu_sample_ms);
    if (new_imu_sample) {
        detector->last_imu_wall_ms = core_now_ms();
        detector->prev_imu_sample_ms = telemetry->imu.last_update_ms;
    }
    bool imu_fresh = detector->last_imu_wall_ms != 0 &&
                      (core_now_ms() - detector->last_imu_wall_ms) < cfg->imu_stale_ms;

    /* Filters run on every tick, independent of GPS rate. */
    core_attitude_filter_update(&detector->attitude, &telemetry->imu);
    core_nav_kf_set_update(&detector->kf_short, &telemetry->imu, &detector->attitude, &telemetry->baro,
                            cfg->kf_sigma_accel_mps2, cfg->kf_baro_r_m2);
    core_nav_kf_set_update(&detector->kf_long, &telemetry->imu, &detector->attitude, &telemetry->baro,
                            cfg->kf_sigma_accel_mps2, cfg->kf_baro_r_m2);

    /* --- GPS health: staleness (possible jamming) + fix quality. Runs every
     * tick regardless of fix_type - staleness is defined by the ABSENCE of
     * new data, and a bad fix_type is itself part of the jamming signature,
     * independent of the fix_type>=2 gate below (which only decides whether
     * a sample is usable as a position-comparison baseline). */
    float score_health = 1.0f;
    uint32_t health_flags = 0;
    bool new_gps_message = telemetry->gps_raw.valid &&
                            (!detector->have_seen_any_gps ||
                             telemetry->gps_raw.last_update_ms != detector->last_gps_message_ms);
    if (new_gps_message) {
        detector->last_gps_message_ms = telemetry->gps_raw.last_update_ms;
        detector->have_seen_any_gps = true;
        if (telemetry->gps_raw.fix_type < 2) {
            score_health = 0.0f;
        }
    }
    if (detector->have_seen_any_gps) {
        uint64_t age_ms = now - detector->last_gps_message_ms;
        float staleness_score = linear_score((float)age_ms, (float)cfg->gps_stale_warn_ms,
                                              (float)cfg->gps_stale_reject_ms);
        if (staleness_score < score_health) score_health = staleness_score;
    }
    if (score_health < 1.0f) health_flags |= CORE_NAV_FLAG_GPS_UNHEALTHY;

    /* --- Position-comparison checks: only on a new, HEALTHY (fix_type >= 2)
     * fix. A no-fix sample must never become the jump/divergence/timestamp
     * baseline for the next real fix (it would read as a huge jump from
     * (0,0)), and must never be used as a reanchor target either. */
    bool new_gps = telemetry->gps_raw.valid && telemetry->gps_raw.fix_type >= 2 &&
                   (!detector->have_prev_gps ||
                    telemetry->gps_raw.last_update_ms != detector->prev_gps_raw.last_update_ms);

    if (new_gps) {
        const core_gps_raw_t *gps = &telemetry->gps_raw;
        const core_gps_raw_t *prev = &detector->prev_gps_raw;
        bool have_prev = detector->have_prev_gps;

        float score_jump = 1.0f;
        float score_dr = 1.0f;
        float score_drift = 1.0f;
        float score_ekf = 1.0f;
        float score_time = 1.0f;
        float score_eph = 1.0f;
        uint32_t flags = 0;

        /* Seed yaw from the first real heading reference available, rather
         * than leaving it at the arbitrary 0 (true north) initial guess -
         * see attitude_filter.h. Only once; it free-integrates from gyro
         * after that, as designed. */
        if (!detector->yaw_seeded && gps->cog_cdeg != UINT16_MAX) {
            core_attitude_filter_seed_yaw(&detector->attitude, cog_cdeg_to_rad(gps->cog_cdeg));
            detector->yaw_seeded = true;
        }

        /* Check 1: jump / implausible speed. No debounce here - a single
         * sample already past jump_hard_reject_m in one GPS interval is
         * unambiguous on its own (and an instant, one-time teleport is
         * exactly the acceptance-criteria scenario this must catch; it
         * would show up on exactly one sample, so requiring 2+ consecutive
         * hits before latching would miss it entirely). Uses the GPS
         * receiver's own time_usec for the interval when available, since
         * two honest fixes can arrive closer together (scheduling jitter)
         * than the receiver's real update rate. */
        if (have_prev) {
            float dist_m = distance_m((double)prev->lat / 1e7, (double)prev->lon / 1e7,
                                       (double)gps->lat / 1e7, (double)gps->lon / 1e7);
            if (dist_m > cfg->jump_hard_reject_m) {
                score_jump = 0.0f;
            } else {
                float dt_s = fix_interval_seconds(gps, prev);
                if (dt_s > 0.0f) {
                    float implied_speed = dist_m / dt_s;
                    score_jump = linear_score(implied_speed, 0.8f * cfg->max_plausible_speed_mps,
                                               cfg->max_plausible_speed_mps);
                }
            }
        }

        /* Check 2 / 3: KF divergence, short and long horizon - compared
         * BEFORE any re-anchoring below, against whatever the filters had
         * predicted independently since their last anchor. Suppressed to
         * neutral if the IMU stream is stale (see imu_fresh above) - a
         * frozen filter's growing "divergence" against a continuing real
         * GPS trajectory isn't evidence of anything but a dead IMU. */
        if (imu_fresh && detector->kf_short.anchored) {
            float dist_m = kf_divergence_m(&detector->kf_short, gps);
            score_dr = linear_score(dist_m, cfg->dr_short_divergence_warn_m, cfg->dr_short_divergence_reject_m);
        }
        if (imu_fresh && detector->kf_long.anchored) {
            float dist_m = kf_divergence_m(&detector->kf_long, gps);
            score_drift = linear_score(dist_m, cfg->dr_long_divergence_warn_m, cfg->dr_long_divergence_reject_m);
        }

        /* Check 4: raw GPS vs flight controller's own EKF output. Only
         * catches a spoof that fools our telemetry relay but NOT the flight
         * controller's own GPS reception - see core/README.md. */
        if (telemetry->global_position.valid) {
            float dist_m = distance_m((double)gps->lat / 1e7, (double)gps->lon / 1e7,
                                       (double)telemetry->global_position.lat / 1e7,
                                       (double)telemetry->global_position.lon / 1e7);
            score_ekf = linear_score(dist_m, cfg->ekf_divergence_warn_m, cfg->ekf_divergence_reject_m);
        }

        /* Check 5: timestamp monotonicity / replay. A zero time_usec means
         * the receiver doesn't populate this field - treat as unavailable
         * rather than a permanent false failure. Requires 2 CONSECUTIVE
         * non-advancing samples, unlike check 1's single-sample latch: a
         * lone non-advancing timestamp is genuinely ambiguous (e.g. a
         * receiver re-sending its last cached fix when polled faster than
         * its native rate produces exactly this, confirmed firsthand earlier
         * this session) - only a repeat is diagnostic of an actual replay. */
        bool timestamp_nonadvancing = have_prev && gps->time_usec != 0 && prev->time_usec != 0 &&
                                       gps->time_usec <= prev->time_usec;
        detector->timestamp_nonadvance_streak = timestamp_nonadvancing
                                                     ? detector->timestamp_nonadvance_streak + 1
                                                     : 0;
        if (detector->timestamp_nonadvance_streak >= 2) {
            score_time = 0.0f;
        }

        /* Check 6: HDOP/eph abnormality (C/N0 stand-in). UINT16_MAX is
         * MAVLink's documented "unknown" sentinel for eph - excluded from
         * both the absolute-value and abrupt-improvement scoring rather
         * than read as a literally terrible HDOP. */
        {
            float score_a = (gps->eph == UINT16_MAX)
                                 ? 1.0f
                                 : linear_score((float)gps->eph, (float)cfg->eph_nominal_max, (float)cfg->eph_reject);
            float score_b = 1.0f;
            if (have_prev && prev->eph != UINT16_MAX && gps->eph != UINT16_MAX && prev->eph > gps->eph) {
                /* Abrupt improvement in HDOP (satellite-quality signal suddenly
                 * looking "too good") is a known spoofing signature. Penalize
                 * only once the improvement exceeds eph_jump_threshold, ramping
                 * to a full penalty by 3x that threshold. */
                float improvement = (float)(prev->eph - gps->eph);
                score_b = linear_score(improvement, (float)cfg->eph_jump_threshold,
                                        (float)cfg->eph_jump_threshold * 3.0f);
            }
            score_eph = (score_a < score_b) ? score_a : score_b;
        }

        if (score_dr < 1.0f) flags |= CORE_NAV_FLAG_DR_DIVERGENCE;
        if (score_drift < 1.0f) flags |= CORE_NAV_FLAG_SLOW_DRIFT;
        if (score_ekf < 1.0f) flags |= CORE_NAV_FLAG_EKF_DIVERGENCE;
        if (score_eph < 1.0f) flags |= CORE_NAV_FLAG_EPH_ABNORMAL;
        if (score_jump <= 0.0f) flags |= CORE_NAV_FLAG_JUMP;
        if (score_time <= 0.0f) flags |= CORE_NAV_FLAG_TIMESTAMP;

        detector->cached_scores.score_jump = score_jump;
        detector->cached_scores.score_dr = score_dr;
        detector->cached_scores.score_drift = score_drift;
        detector->cached_scores.score_ekf = score_ekf;
        detector->cached_scores.score_time = score_time;
        detector->cached_scores.score_eph = score_eph;
        detector->cached_scores.flags = flags;
        detector->have_cached_scores = true;

        /* Re-anchor the KF sets, gated on no hard failure AND no visible
         * divergence of their OWN check this round (never let a confirmed
         * attack's data become the new trusted baseline, and - the fix for
         * a slow/sustained spoof the flight controller's own EKF has
         * accepted, where score_ekf gives no signal - don't let a filter
         * that's already showing meaningful divergence be silently re-synced
         * to GPS before that divergence has a chance to grow into a
         * reject-level flag). Once genuinely re-converged (score back at
         * ceiling), reanchoring resumes normally. */
        bool hard_failure = (score_jump <= 0.0f) || (score_time <= 0.0f);
        if (!hard_failure && score_dr >= 1.0f && core_nav_kf_set_should_reanchor(&detector->kf_short, now)) {
            core_nav_kf_set_reanchor(&detector->kf_short, gps);
        }
        if (!hard_failure && score_drift >= 1.0f && core_nav_kf_set_should_reanchor(&detector->kf_long, now)) {
            core_nav_kf_set_reanchor(&detector->kf_long, gps);
        }

        detector->prev_gps_raw = *gps;
        detector->have_prev_gps = true;
    }

    /* Build this tick's result from the cached (GPS-tick-refreshed) scores,
     * then layer in the two every-tick, non-cached signals: IMU-staleness
     * (overrides the KF-based scores back to neutral, recomputing their
     * flags fresh rather than trusting a now-stale cached flag bit) and GPS
     * health (always fresh, never gated on a healthy fix_type). */
    core_nav_result_t out = detector->have_cached_scores
                                 ? detector->cached_scores
                                 : (core_nav_result_t){.score_jump = 1.0f, .score_dr = 1.0f, .score_drift = 1.0f,
                                                        .score_ekf = 1.0f, .score_time = 1.0f, .score_eph = 1.0f,
                                                        .score_health = 1.0f, .flags = 0};

    uint32_t flags = out.flags & ~(CORE_NAV_FLAG_DR_DIVERGENCE | CORE_NAV_FLAG_SLOW_DRIFT);
    if (!imu_fresh) {
        out.score_dr = 1.0f;
        out.score_drift = 1.0f;
        flags |= CORE_NAV_FLAG_IMU_LOST;
    } else {
        if (out.score_dr < 1.0f) flags |= CORE_NAV_FLAG_DR_DIVERGENCE;
        if (out.score_drift < 1.0f) flags |= CORE_NAV_FLAG_SLOW_DRIFT;
    }
    out.score_health = score_health;
    flags |= health_flags;
    out.flags = flags;

    /* Slew-limited publish: drop instantly on evidence, recover at a bounded
     * rate. Runs every call (even without a new GPS sample) so the published
     * value keeps updating in real time. detector->nav_trust is a dedicated
     * field, never aliased with the cached sub-scores above, so it can't be
     * clobbered by a partially-built result on the next GPS tick. */
    float raw = fminf(fminf(fminf(out.score_jump, out.score_dr), fminf(out.score_drift, out.score_ekf)),
                       fminf(fminf(out.score_time, out.score_eph), out.score_health));

    float dt_s = (detector->last_slew_update_ms == 0)
                     ? 0.0f
                     : (float)(now - detector->last_slew_update_ms) / 1000.0f;
    float prev_trust = detector->nav_trust;

    float nav_trust;
    if (raw < prev_trust) {
        nav_trust = raw;
    } else {
        float recovered = prev_trust + cfg->trust_recover_rate_per_s * dt_s;
        nav_trust = (raw < recovered) ? raw : recovered;
    }
    if (nav_trust < 0.0f) nav_trust = 0.0f;
    if (nav_trust > 1.0f) nav_trust = 1.0f;

    detector->nav_trust = nav_trust;
    detector->last_slew_update_ms = now;

    out.nav_trust = nav_trust;
    out.last_update_ms = now;

    return out;
}
