/*
 * Fully synthetic (no sockets, no SITL) scenario tests for the GNSS
 * spoofing detector: normal flight (false-positive baseline), instant
 * jump, slow drift (both with the flight controller's EKF staying on
 * ground truth, and - the harder, more realistic case - following the
 * spoof), replay (live-timestamp and stale-timestamp variants), GPS
 * jamming/staleness, and a handful of ordinary-flight conditions that must
 * NOT false-positive. Telemetry timestamps are hand-advanced, which is what
 * lets this run in well under a second instead of needing to actually wait
 * out the detector's real re-anchor/slew intervals - except
 * test_imu_loss_not_treated_as_attack, which needs a couple of real
 * seconds to exercise IMU staleness (tracked against wall-clock time on
 * purpose - see gnss_anomaly.c).
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include "core/common/time.h"
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
 * gnss_spoof_proxy.c's documented limitation, and test_ekf_follows_drift
 * below for the harder case where it doesn't). gps_raw defaults to ground
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

static void offset_lat_north(int32_t *lat_degE7, double meters) {
    *lat_degE7 += (int32_t)((meters / METERS_PER_DEG_LAT) * 1e7);
}

static void offset_gps_north(core_gps_raw_t *gps, double meters) {
    offset_lat_north(&gps->lat, meters);
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
    float max_trust_while_held = 0.0f;

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
        if (jumped) {
            if (result.nav_trust < min_trust_after_jump) min_trust_after_jump = result.nav_trust;
            /* The whole rest of the run (through i=399, ~5s held) - trust must
             * stay down for as long as the jump holds, not just blip and
             * recover (the bug this re-anchor-gating fix addresses: a held
             * 100m jump previously recovered to 1.00 after ~7.4s). */
            if (result.nav_trust > max_trust_while_held) max_trust_while_held = result.nav_trust;
        }
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(jumped);
    assert(min_trust_after_jump <= 0.01f);
    assert(max_trust_while_held <= 0.1f);
    assert(flags_seen & CORE_NAV_FLAG_JUMP);
}

/* --- Case 3: slow drift, under the short-horizon threshold per step but
 * accumulating past the long-horizon one. EKF stays on ground truth here
 * (the easier case - see test_ekf_follows_drift for the harder, more
 * realistic one). --- */
static void test_slow_drift(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;
    /* Needs to clear the long-horizon warn threshold (10m) within one
     * 8s re-anchor window (32m at this rate) while staying under the
     * short-horizon warn threshold (10m) within its own 1.5s window (6m). */
    const float drift_rate_mps = 4.0f; /* applied to reported position only */

    uint32_t flags_seen = 0;
    double drift_m = 0.0;
    float min_trust_after_trip = 1.0f;
    bool tripped = false;

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
        if (result.flags & CORE_NAV_FLAG_SLOW_DRIFT) tripped = true;
        if (tripped && result.nav_trust < min_trust_after_trip) min_trust_after_trip = result.nav_trust;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen & CORE_NAV_FLAG_SLOW_DRIFT);
    assert(!(flags_seen & CORE_NAV_FLAG_JUMP));
    /* Sustained, not a blip: once it trips, trust should stay meaningfully
     * low for the remainder of the run (the drift never stops). */
    assert(tripped);
    assert(min_trust_after_trip <= 0.3f);
}

/* --- Case 3b: the harder, more realistic version of case 3 - the flight
 * controller's own EKF has ALSO been fooled (as a real RF-level spoof
 * would do, rather than our proxy's telemetry-relay-only limitation), so
 * global_position agrees with the spoofed gps_raw throughout and the
 * EKF-divergence check (#4) gives no signal at all. Detection must rest
 * entirely on the IMU-based KF checks, which never ingest GPS. This is
 * the scenario a review of this code found completely undetected
 * (trust 1.00, no flags) before the re-anchor-gating fix. --- */
static void test_ekf_follows_drift(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;
    const float drift_rate_mps = 1.5f; /* the concrete rate the review's own probe missed entirely */

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
            offset_lat_north(&t.global_position.lat, drift_m); /* EKF follows the spoof too */
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen & CORE_NAV_FLAG_SLOW_DRIFT);        /* caught via the IMU-based KF checks */
    assert(!(flags_seen & CORE_NAV_FLAG_EKF_DIVERGENCE));  /* confirms check #4 genuinely gives no signal here */
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
     * its 10m warn threshold, so it keeps getting "forgiven" right before
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
    bool tripped = false;
    float min_trust_after_trip = 1.0f;

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
        if (result.flags & CORE_NAV_FLAG_DR_DIVERGENCE) tripped = true;
        if (tripped && result.nav_trust < min_trust_after_trip) min_trust_after_trip = result.nav_trust;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(have_frozen_fix);
    assert(flags_seen & CORE_NAV_FLAG_DR_DIVERGENCE);
    assert(!(flags_seen & CORE_NAV_FLAG_TIMESTAMP));
    assert(tripped);
    assert(min_trust_after_trip <= 0.3f);
}

/* --- Case 4b: replay with the original stale timestamp - caught by the
 * timestamp/monotonicity check (after 2 consecutive non-advancing samples -
 * see the debounce note on the single-duplicate regression test below). --- */
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
    bool tripped = false;
    float min_trust_after_trip = 1.0f;

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
        if (result.flags & CORE_NAV_FLAG_TIMESTAMP) tripped = true;
        if (tripped && result.nav_trust < min_trust_after_trip) min_trust_after_trip = result.nav_trust;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(have_frozen_fix);
    assert(flags_seen & CORE_NAV_FLAG_TIMESTAMP);
    assert(tripped);
    assert(min_trust_after_trip <= 0.3f);
}

/* --- Case 5: GPS jamming - no GPS_RAW_INT at all for a while. Must be
 * flagged (it's in scope - "detect spoofing/jamming"), independent of
 * whatever the IMU-based checks say (which have nothing to compare
 * against either, with no new GPS arriving). --- */
static void test_gps_jamming_detected(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;

    uint32_t flags_seen = 0;
    float min_trust = 1.0f;

    /* ~3s of normal GPS, then jamming (no GPS at all) for the rest - well
     * past gps_stale_reject_ms (5000ms default). Clock is IMU-paced
     * (simulated), so this doesn't need a real sleep. */
    for (int i = 0; i < 700; i++) { /* 14 seconds */
        core_telemetry_t t = make_telemetry(&v);
        bool is_gps_tick = (i % gps_every_n_ticks == 0) && i < 150;
        if (!is_gps_tick) {
            t.gps_raw.valid = false;
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        if (result.nav_trust < min_trust) min_trust = result.nav_trust;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen & CORE_NAV_FLAG_GPS_UNHEALTHY);
    assert(min_trust <= 0.1f);
}

/* --- Regression: a no-fix (fix_type 0, degenerate (0,0)) sample must not
 * become the comparison baseline for the next real fix - it would read as
 * a huge jump from (0,0) otherwise. --- */
static void test_no_fix_then_real_fix_is_not_a_jump(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    uint32_t flags_seen_during_no_fix = 0;
    uint32_t flags_seen_after = 0;

    for (int i = 0; i < 50; i++) { /* 1s of no-fix samples at (0,0) */
        core_telemetry_t t = make_telemetry(&v);
        t.gps_raw.fix_type = 0;
        t.gps_raw.lat = 0;
        t.gps_raw.lon = 0;
        t.gps_raw.last_update_ms = v.t_ms;
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen_during_no_fix |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    for (int i = 0; i < 50; i++) { /* then a real fix at the true position */
        core_telemetry_t t = make_telemetry(&v);
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen_after |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen_during_no_fix & CORE_NAV_FLAG_GPS_UNHEALTHY);
    assert(!(flags_seen_after & CORE_NAV_FLAG_JUMP));
}

/* --- Regression: eph == UINT16_MAX ("unknown") must be excluded, not
 * scored as a maximally bad HDOP. --- */
static void test_eph_unknown_not_penalized(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;
    float min_trust = 1.0f;

    for (int i = 0; i < 500; i++) { /* 10 seconds, eph unknown throughout */
        core_telemetry_t t = make_telemetry(&v);
        if (i % gps_every_n_ticks != 0) {
            t.gps_raw.valid = false;
        } else {
            t.gps_raw.eph = UINT16_MAX;
        }
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        if (result.nav_trust < min_trust) min_trust = result.nav_trust;
        assert(!(result.flags & CORE_NAV_FLAG_EPH_ABNORMAL));
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(min_trust > 0.9f);
}

/* --- Regression: a single non-advancing timestamp (e.g. a receiver
 * re-sending its last cached fix) must not alone latch the replay flag -
 * only a genuine repeat (2+ consecutive) is diagnostic. --- */
static void test_single_duplicate_timestamp_not_flagged(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;
    uint32_t flags_seen = 0;
    uint64_t last_time_usec = 0;

    for (int i = 0; i < 500; i++) { /* 10 seconds, one duplicate in the middle */
        core_telemetry_t t = make_telemetry(&v);
        if (i % gps_every_n_ticks != 0) {
            t.gps_raw.valid = false;
        } else if (i == 250) {
            t.gps_raw.time_usec = last_time_usec; /* exactly one non-advancing sample */
        }
        if (t.gps_raw.valid) last_time_usec = t.gps_raw.time_usec;
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(!(flags_seen & CORE_NAV_FLAG_TIMESTAMP));
}

/* --- Regression: if the IMU stream itself goes stale, a frozen KF
 * prediction diverging from a continuing real GPS trajectory must not be
 * mistaken for an attack. Needs a real sleep past imu_stale_ms (2000ms
 * default), since IMU freshness is deliberately tracked against wall-clock
 * time, not the deterministic simulated clock used everywhere else. --- */
static void test_imu_loss_not_treated_as_attack(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 5.0f, 0.0f);

    const uint32_t imu_dt_ms = 20;
    const uint32_t gps_every_n_ticks = 10;

    /* ~3s of normal flight to anchor both KFs. */
    for (int i = 0; i < 150; i++) {
        core_telemetry_t t = make_telemetry(&v);
        if (i % gps_every_n_ticks != 0) t.gps_raw.valid = false;
        core_nav_detector_update(&detector, &t, &cfg);
        sim_vehicle_step(&v, imu_dt_ms);
    }

    /* IMU stream dies (stop advancing it / stop sending it), but the
     * vehicle keeps moving and GPS keeps reporting real motion. */
    /* The warm-up loop steps v.t_ms AFTER each detector_update call, so by
     * now it's one tick ahead of the imu.last_update_ms the detector's
     * prev_imu_sample_ms actually last saw - step back one to match, or
     * the first post-sleep sample would look "new" and refresh
     * last_imu_wall_ms, defeating the point of the sleep. */
    uint64_t frozen_imu_ms = v.t_ms - imu_dt_ms;
    usleep(2200000); /* real sleep past imu_stale_ms=2000 */

    uint32_t flags_seen = 0;
    for (int i = 0; i < 30; i++) { /* a few more GPS ticks, real motion continuing */
        core_telemetry_t t = make_telemetry(&v);
        t.imu.last_update_ms = frozen_imu_ms; /* frozen - stream is dead */
        core_nav_result_t result = core_nav_detector_update(&detector, &t, &cfg);
        flags_seen |= result.flags;
        sim_vehicle_step(&v, imu_dt_ms);
    }

    assert(flags_seen & CORE_NAV_FLAG_IMU_LOST);
    assert(!(flags_seen & CORE_NAV_FLAG_DR_DIVERGENCE));
    assert(!(flags_seen & CORE_NAV_FLAG_SLOW_DRIFT));
}

/* --- Yaw seeding: from the FC's heading while disarmed only, else from COG
 * only once moving. A wrong yaw misdirects every horizontal acceleration the
 * KFs integrate, so a seed from noise is a false alarm waiting for the first
 * forward flight. --- */
static void test_yaw_seed_from_fc_heading_until_armed(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);

    core_telemetry_t t;
    for (int i = 0; i < 50; i++) { /* on the pad, disarmed, COG reading noise */
        t = make_telemetry(&v);
        t.gps_raw.cog_cdeg = (uint16_t)((i * 7919) % 36000);
        t.heartbeat.valid = true;
        t.attitude.yaw = 1.0f;
        t.attitude.last_update_ms = core_now_ms();
        t.attitude.valid = true;
        core_nav_detector_update(&detector, &t, &cfg);
        sim_vehicle_step(&v, 20);
    }
    assert(detector.yaw_seeded);
    assert(fabsf(detector.attitude.yaw - 1.0f) < 1e-4f);

    /* Armed: the FC's heading no longer counts, so a spoof-steered EKF can't
     * drag core's yaw along with it. */
    for (int i = 0; i < 50; i++) {
        t = make_telemetry(&v);
        t.heartbeat.valid = true;
        t.heartbeat.base_mode = 128;
        t.attitude.yaw = 2.0f;
        t.attitude.last_update_ms = core_now_ms();
        t.attitude.valid = true;
        core_nav_detector_update(&detector, &t, &cfg);
        sim_vehicle_step(&v, 20);
    }
    assert(fabsf(detector.attitude.yaw - 1.0f) < 1e-4f);
}

static void test_yaw_not_seeded_from_stationary_cog(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);

    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);
    for (int i = 0; i < 50; i++) { /* no ATTITUDE stream, stationary */
        core_telemetry_t t = make_telemetry(&v);
        t.gps_raw.cog_cdeg = 31500;
        core_nav_detector_update(&detector, &t, &cfg);
        sim_vehicle_step(&v, 20);
    }
    assert(!detector.yaw_seeded);

    v.ve_mps = 5.0f; /* moving east: COG now means something */
    for (int i = 0; i < 50; i++) {
        core_telemetry_t t = make_telemetry(&v);
        core_nav_detector_update(&detector, &t, &cfg);
        sim_vehicle_step(&v, 20);
    }
    assert(detector.yaw_seeded);
    assert(fabsf(detector.attitude.yaw - (float)M_PI / 2.0f) < 0.01f);
}

static void test_gps_sample_ahead_of_imu_is_fresh(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);
    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);
    core_telemetry_t t = make_telemetry(&v);
    t.imu.last_update_ms = 1000;
    t.gps_raw.last_update_ms = 1001;
    core_nav_result_t r = core_nav_detector_update(&detector, &t, &cfg);
    assert(r.score_health == 1.0f);
    assert(!(r.flags & CORE_NAV_FLAG_GPS_UNHEALTHY));
}

static void test_bias_calibration_does_not_include_flight_gap(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);
    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);
    core_telemetry_t t = make_telemetry(&v);
    t.heartbeat.valid = true;
    t.imu.last_update_ms = 1000;
    core_nav_detector_update(&detector, &t, &cfg);
    t.heartbeat.base_mode = 128;
    t.imu.last_update_ms = 61000;
    core_nav_detector_update(&detector, &t, &cfg);
    t.heartbeat.base_mode = 0;
    t.imu.last_update_ms = 61020;
    t.imu.xgyro_rps = 0.5f; /* a brief landing motion, not a minute of bias */
    core_nav_detector_update(&detector, &t, &cfg);
    assert(detector.attitude.gyro_bias[0] == 0.0f);
}

static void test_unknown_speed_does_not_seed_cog(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);
    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);
    core_telemetry_t t = make_telemetry(&v);
    t.gps_raw.cog_cdeg = 9000;
    t.gps_raw.vel_cms = UINT16_MAX;
    core_nav_detector_update(&detector, &t, &cfg);
    assert(!detector.yaw_seeded);
}

static void test_nonfinite_fc_attitude_uses_fallback(void) {
    core_nav_config_t cfg;
    core_nav_config_default(&cfg);
    core_nav_detector_t detector;
    core_nav_detector_init(&detector, &cfg);
    sim_vehicle_t v;
    sim_vehicle_init(&v, 47.397742, -122.397456, 10.0, 0.0f, 0.0f);
    for (int i = 0; i < 150; i++) {
        core_telemetry_t t = make_telemetry(&v);
        t.attitude.valid = true;
        t.attitude.last_update_ms = core_now_ms();
        if (i % 3 == 0) t.attitude.roll = NAN;
        if (i % 3 == 1) t.attitude.pitch = INFINITY;
        if (i % 3 == 2) t.attitude.yaw = NAN;
        core_nav_result_t r = core_nav_detector_update(&detector, &t, &cfg);
        assert(isfinite(r.nav_trust));
        sim_vehicle_step(&v, 20);
    }
    double lat, lon, alt;
    core_nav_kf_set_get_position(&detector.kf_short, &lat, &lon, &alt);
    assert(isfinite(lat) && isfinite(lon) && isfinite(alt));
}

int main(void) {
    test_normal_flight();
    test_instant_jump();
    test_slow_drift();
    test_ekf_follows_drift();
    test_replay_live_timestamp();
    test_replay_stale_timestamp();
    test_gps_jamming_detected();
    test_no_fix_then_real_fix_is_not_a_jump();
    test_eph_unknown_not_penalized();
    test_single_duplicate_timestamp_not_flagged();
    test_imu_loss_not_treated_as_attack();
    test_yaw_seed_from_fc_heading_until_armed();
    test_yaw_not_seeded_from_stationary_cog();
    test_gps_sample_ahead_of_imu_is_fresh();
    test_bias_calibration_does_not_include_flight_gap();
    test_unknown_speed_does_not_seed_cog();
    test_nonfinite_fc_attitude_uses_fallback();
    return 0;
}
