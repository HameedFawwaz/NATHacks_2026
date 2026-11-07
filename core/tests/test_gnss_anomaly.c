/*
 * Fully synthetic (no sockets, no SITL) scenario tests for the GNSS
 * spoofing detector: normal flight (false-positive baseline), instant
 * jump, slow drift, and replay (live-timestamp and stale-timestamp
 * variants). Telemetry timestamps are hand-advanced, which is what lets
 * this run in well under a second instead of needing to actually wait out
 * the detector's real re-anchor/slew intervals.
 */
#include <assert.h>
#include <math.h>
#include <string.h>

#include "core/nav/gnss_anomaly.h"
#include "core/nav/nav_config.h"

#define METERS_PER_DEG_LAT 111320.0
#define GROUND_PRESSURE_HPA 1013.25f

typedef struct {
    uint64_t t_ms;
    double lat_deg, lon_deg, alt_m;
    float vn_mps, ve_mps;
} sim_vehicle_t;

static void sim_vehicle_init(sim_vehicle_t *v, double lat_deg, double lon_deg, double alt_m,
                              float vn_mps, float ve_mps) {
    v->t_ms = 1000;
    v->lat_deg = lat_deg;
    v->lon_deg = lon_deg;
    v->alt_m = alt_m;
    v->vn_mps = vn_mps;
    v->ve_mps = ve_mps;
}

static void sim_vehicle_step(sim_vehicle_t *v, uint32_t dt_ms) {
    double dt_s = (double)dt_ms / 1000.0;
    double mean_lat_rad = v->lat_deg * M_PI / 180.0;
    v->lat_deg += (v->vn_mps * dt_s) / METERS_PER_DEG_LAT;
    v->lon_deg += (v->ve_mps * dt_s) / (METERS_PER_DEG_LAT * cos(mean_lat_rad));
    v->t_ms += dt_ms;
}

/* Builds one telemetry sample: IMU (level, non-accelerating, non-rotating),
 * baro (constant ground pressure - vehicle isn't changing altitude), and an
 * EKF global_position that matches ground truth exactly (as it would if
 * nothing fed back into the flight controller's own estimator - see
 * gnss_spoof_proxy.c's documented limitation). gps_raw defaults to ground
 * truth too; the caller overrides it afterward to inject an attack. */
static core_telemetry_t make_telemetry(const sim_vehicle_t *v) {
    core_telemetry_t t;
    memset(&t, 0, sizeof(t));

    t.imu.xacc_mps2 = 0.0f;
    t.imu.yacc_mps2 = 0.0f;
    t.imu.zacc_mps2 = -9.80665f; /* level, stationary-equivalent (constant velocity -> zero accel) */
    t.imu.xgyro_rps = 0.0f;
    t.imu.ygyro_rps = 0.0f;
    t.imu.zgyro_rps = 0.0f;
    t.imu.last_update_ms = v->t_ms;
    t.imu.valid = true;

    t.baro.press_abs_hpa = GROUND_PRESSURE_HPA;
    t.baro.last_update_ms = v->t_ms;
    t.baro.valid = true;

    float speed = sqrtf(v->vn_mps * v->vn_mps + v->ve_mps * v->ve_mps);
    float cog_rad = atan2f(v->ve_mps, v->vn_mps);
    float cog_cdeg = cog_rad * 18000.0f / (float)M_PI;
    if (cog_cdeg < 0.0f) cog_cdeg += 36000.0f;

    t.global_position.lat = (int32_t)(v->lat_deg * 1e7);
    t.global_position.lon = (int32_t)(v->lon_deg * 1e7);
    t.global_position.alt_mm = (int32_t)(v->alt_m * 1000.0);
    t.global_position.vx = (int16_t)(v->vn_mps * 100.0f);
    t.global_position.vy = (int16_t)(v->ve_mps * 100.0f);
    t.global_position.last_update_ms = v->t_ms;
    t.global_position.valid = true;

    t.gps_raw.fix_type = 3;
    t.gps_raw.lat = (int32_t)(v->lat_deg * 1e7);
    t.gps_raw.lon = (int32_t)(v->lon_deg * 1e7);
    t.gps_raw.alt_mm = (int32_t)(v->alt_m * 1000.0);
    t.gps_raw.eph = 150;
    t.gps_raw.epv = 150;
    t.gps_raw.vel_cms = (uint16_t)(speed * 100.0f);
    t.gps_raw.cog_cdeg = (uint16_t)cog_cdeg;
    t.gps_raw.satellites_visible = 10;
    t.gps_raw.time_usec = v->t_ms * 1000ULL;
    t.gps_raw.last_update_ms = v->t_ms;
    t.gps_raw.valid = true;

    return t;
}

static void offset_gps_north(core_gps_raw_t *gps, double meters) {
    gps->lat += (int32_t)((meters / METERS_PER_DEG_LAT) * 1e7);
}

/* --- Case 1: normal flight, no attack - false-positive baseline --- */
static void test_normal_flight(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    float min_trust = 1.0f;
    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10; /* 5Hz GPS at 50Hz IMU */

    for (int i = 0; i < 1000; i++) { /* 20 seconds */
        core_telemetry_t t = make_telemetry(&v);
        if (i % gps_every_n_ticks != 0) {
            t.gps_raw.valid = false; /* only every Nth tick carries a fresh GPS fix */
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        if (result.nav_trust < min_trust) min_trust = result.nav_trust;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(min_trust > 0.9f);
}

/* --- Case 2: instant jump --- */
static void test_instant_jump(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;

    bool jumped = false;
    uint32_t flags_seen = 0;
    float min_trust_after_jump = 1.0f;

    /*
     * The hard jump check fires on exactly the one sample where the
     * position actually teleports (score_jump reverts to normal immediately
     * afterward, since the per-step delta from the new, attacker-controlled
     * baseline looks like ordinary motion again) - so this tracks whether
     * the flag was EVER raised and how low trust got immediately after,
     * rather than reading only the final sample. What eventually happens
     * further down the line (the KF sets re-anchoring to the spoofed
     * position and trust recovering) is the detector correctly reporting
     * "things look consistent again from here" - deciding what to DO about
     * that is the autonomy layer's job, out of scope for this detector.
     */
    for (int i = 0; i < 400; i++) { /* 8 seconds: long enough to anchor both KFs first */
        core_telemetry_t t = make_telemetry(&v);
        bool is_gps_tick = (i % gps_every_n_ticks == 0);
        if (!is_gps_tick) {
            t.gps_raw.valid = false;
        } else if (i >= 150) { /* after ~3s of normal flight, jump and hold */
            offset_gps_north(&t.gps_raw, 100.0); /* well past jump_hard_reject_m=30 */
            /* vel/cog deliberately left matching real motion - inconsistent
             * with the position jump, as the design intends. */
            jumped = true;
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        if (jumped && i < 200 && result.nav_trust < min_trust_after_jump) {
            min_trust_after_jump = result.nav_trust;
        }
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(jumped);
    assert(min_trust_after_jump <= 0.01f);
    assert(flags_seen & CORE_NAV_FLAG_JUMP);
}

/* --- Case 3: slow drift, under the short-horizon threshold per step but
 * accumulating past the long-horizon one. --- */
static void test_slow_drift(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;
    /* Needs to clear the long-horizon warn threshold (20m) within one
     * 8s re-anchor window (32m at this rate) while staying under the
     * short-horizon warn threshold (10m) within its own 1.5s window (6m). */
    const float drift_rate_mps = 4.0f; /* applied to reported position only */

    uint32_t flags_seen = 0;
    double drift_m = 0.0;

    for (int i = 0; i < 1250; i++) { /* 25 seconds */
        core_telemetry_t t = make_telemetry(&v);
        bool is_gps_tick = (i % gps_every_n_ticks == 0);
        if (!is_gps_tick) {
            t.gps_raw.valid = false;
        } else {
            drift_m += drift_rate_mps * ((double)(gps_every_n_ticks * imu_dt_ms) / 1000.0);
            offset_gps_north(&t.gps_raw, drift_m);
            /* vel/cog intentionally left matching real motion - the drift
             * rate is small enough not to trip the jump check on its own. */
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen & CORE_NAV_FLAG_SLOW_DRIFT);
    assert(!(flags_seen & CORE_NAV_FLAG_JUMP));
}

/* --- Case 4a: replay with a live (re-stamped) timestamp - caught by KF
 * divergence, not the timestamp check. --- */
static void test_replay_live_timestamp(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    /* Faster than the other scenarios on purpose: at 5 m/s, the divergence
     * accumulated between kf_short's 1.5s re-anchors (~7.5m) sits just under
     * its 8m warn threshold, so it keeps getting "forgiven" right before
     * tripping - a real effect (it's structurally what lets slow drift hide
     * from the short horizon too), but not what this case is isolating. At
     * 10 m/s the short-horizon divergence clearly exceeds its threshold
     * before the next re-anchor can reset it. */
    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 10.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;

    core_gps_raw_t frozen_fix = {0};
    bool have_frozen_fix = false;
    uint32_t flags_seen = 0;

    for (int i = 0; i < 400; i++) { /* 8 seconds */
        core_telemetry_t t = make_telemetry(&v);
        bool is_gps_tick = (i % gps_every_n_ticks == 0);
        if (!is_gps_tick) {
            t.gps_raw.valid = false;
        } else if (i < 150) {
            frozen_fix = t.gps_raw; /* keep capturing, so we freeze a recent, realistic fix */
        } else {
            if (!have_frozen_fix) {
                have_frozen_fix = true;
            }
            /* Replay the frozen position but re-stamp time to look live. */
            core_gps_raw_t replayed = frozen_fix;
            replayed.time_usec = v.t_ms * 1000ULL;
            replayed.last_update_ms = v.t_ms;
            t.gps_raw = replayed;
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(have_frozen_fix);
    assert(flags_seen & CORE_NAV_FLAG_DR_DIVERGENCE);
    assert(!(flags_seen & CORE_NAV_FLAG_TIMESTAMP));
}

/* --- Case 4b: replay with the original stale timestamp - caught by the
 * timestamp/monotonicity check. --- */
static void test_replay_stale_timestamp(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;

    core_gps_raw_t frozen_fix = {0};
    bool have_frozen_fix = false;
    uint32_t flags_seen = 0;

    for (int i = 0; i < 400; i++) {
        core_telemetry_t t = make_telemetry(&v);
        bool is_gps_tick = (i % gps_every_n_ticks == 0);
        if (!is_gps_tick) {
            t.gps_raw.valid = false;
        } else if (i < 150) {
            frozen_fix = t.gps_raw;
        } else {
            have_frozen_fix = true;
            uint64_t receive_time_ms = t.gps_raw.last_update_ms; /* "just arrived" per our own clock */
            t.gps_raw = frozen_fix;                              /* ...but payload is the old, replayed one */
            t.gps_raw.last_update_ms = receive_time_ms;          /* time_usec stays stale - that's the point */
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(have_frozen_fix);
    assert(flags_seen & CORE_NAV_FLAG_TIMESTAMP);
}

int main(void) {
    test_normal_flight();
    test_instant_jump();
    test_slow_drift();
    test_replay_live_timestamp();
    test_replay_stale_timestamp();
    return 0;
}
