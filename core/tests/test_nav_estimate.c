/*
 * Synthetic tests for the published position estimate (EST_N/EST_E, issue
 * #16): the home-relative projection, readiness gating, IMU staleness, the
 * dead-reckoning age limit, bounds, and that a slow GPS drift is not followed.
 * Timestamps are synthetic and passed as now_ms, so this runs instantly.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/nav/nav.h"

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            exit(1);                                                                  \
        }                                                                             \
    } while (0)

#define R_EARTH CORE_NAV_EST_EARTH_RADIUS_M
#define DEG (M_PI / 180.0)

/* ArduPilot SITL's default CMAC location. */
#define HOME_LAT -35.363261
#define HOME_LON 149.165230

/* The dashboard's model.FromLocal, ported, as the reference inverse. */
static void dashboard_from_local(double olat, double olon, double east_m, double north_m, double *lat, double *lon) {
    *lat = olat + (north_m / R_EARTH) / DEG;
    *lon = olon + (east_m / (R_EARTH * cos(olat * DEG))) / DEG;
}

/* Truth, as metres north/east of home, moved at constant velocity. */
typedef struct {
    uint64_t t_ms;
    double north_m, east_m;
    float vn, ve;
} truth_t;

static void truth_latlon(const truth_t *v, double *lat, double *lon) {
    dashboard_from_local(HOME_LAT, HOME_LON, v->east_m, v->north_m, lat, lon);
}

static core_telemetry_t make_telemetry(const truth_t *v, bool gps_tick, double gps_offset_north_m) {
    core_telemetry_t t;
    memset(&t, 0, sizeof(t));
    t.imu.zacc_mps2 = -9.80665f; /* level, constant velocity */
    t.imu.last_update_ms = v->t_ms;
    t.imu.valid = true;
    t.baro.press_abs_hpa = 1013.25f;
    t.baro.last_update_ms = v->t_ms;
    t.baro.valid = true;

    double lat, lon;
    truth_latlon(v, &lat, &lon);
    t.global_position.lat = (int32_t)lrint(lat * 1e7);
    t.global_position.lon = (int32_t)lrint(lon * 1e7);
    t.global_position.last_update_ms = v->t_ms;
    t.global_position.valid = true;

    if (gps_tick) {
        double glat, glon;
        dashboard_from_local(HOME_LAT, HOME_LON, v->east_m, v->north_m + gps_offset_north_m, &glat, &glon);
        float speed = hypotf(v->vn, v->ve);
        float cog = atan2f(v->ve, v->vn) * 18000.0f / (float)M_PI;
        if (cog < 0.0f) cog += 36000.0f;
        t.gps_raw.fix_type = 3;
        t.gps_raw.lat = (int32_t)lrint(glat * 1e7);
        t.gps_raw.lon = (int32_t)lrint(glon * 1e7);
        t.gps_raw.alt_mm = 600000;
        t.gps_raw.eph = 120;
        t.gps_raw.epv = 120;
        t.gps_raw.vel_cms = (uint16_t)(speed * 100.0f);
        t.gps_raw.cog_cdeg = (uint16_t)cog;
        t.gps_raw.satellites_visible = 12;
        t.gps_raw.time_usec = v->t_ms * 1000ULL;
        t.gps_raw.last_update_ms = v->t_ms;
        t.gps_raw.valid = true;
    }

    /* Fresh, level FC attitude facing along the track. */
    t.attitude.yaw = atan2f(v->ve, v->vn);
    t.attitude.last_update_ms = v->t_ms;
    t.attitude.valid = true;

    t.home_position.lat = (int32_t)lrint(HOME_LAT * 1e7);
    t.home_position.lon = (int32_t)lrint(HOME_LON * 1e7);
    t.home_position.alt_mm = 584000;
    t.home_position.last_update_ms = v->t_ms;
    t.home_position.valid = true;
    return t;
}

typedef struct {
    core_nav_config_t nav_cfg;
    core_nav_estimate_config_t est_cfg;
    core_nav_detector_t detector;
    core_nav_estimator_t estimator;
    truth_t v;
    int tick;
} rig_t;

static void rig_init(rig_t *r, double north_m, double east_m, float vn, float ve) {
    memset(r, 0, sizeof(*r));
    core_nav_config_default(&r->nav_cfg);
    core_nav_estimate_config_default(&r->est_cfg);
    core_nav_detector_init(&r->detector, &r->nav_cfg);
    core_nav_estimator_init(&r->estimator);
    r->v = (truth_t){.t_ms = 1000, .north_m = north_m, .east_m = east_m, .vn = vn, .ve = ve};
}

/* One 50 Hz tick with 5 Hz GPS, then truth advances. */
static core_nav_estimate_t rig_step_with(rig_t *r, core_telemetry_t *t) {
    core_nav_detector_update(&r->detector, t, &r->nav_cfg);
    core_nav_estimate_t e =
        core_nav_estimator_update(&r->estimator, &r->detector, t, &r->nav_cfg, &r->est_cfg, r->v.t_ms);
    r->v.t_ms += 20;
    r->v.north_m += r->v.vn * 0.02;
    r->v.east_m += r->v.ve * 0.02;
    r->tick++;
    return e;
}

static core_nav_estimate_t rig_step(rig_t *r, double gps_offset_north_m) {
    core_telemetry_t t = make_telemetry(&r->v, r->tick % 10 == 0, gps_offset_north_m);
    return rig_step_with(r, &t);
}

static double est_error_m(const core_nav_estimate_t *e, const truth_t *v) {
    return hypot(e->north_m - v->north_m, e->east_m - v->east_m);
}

static void test_projection_matches_dashboard_inverse(void) {
    const double offsets[][2] = {{0, 0}, {120.5, -80.25}, {-950.0, 1500.0}, {25000.0, -40000.0}};
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        double lat, lon, n, e;
        dashboard_from_local(HOME_LAT, HOME_LON, offsets[i][1], offsets[i][0], &lat, &lon);
        CHECK(core_nav_local_from_geo(HOME_LAT, HOME_LON, lat, lon, &n, &e));
        CHECK(fabs(n - offsets[i][0]) < 1e-6 && fabs(e - offsets[i][1]) < 1e-6);
    }
    /* Exact values: 0.001 deg of latitude, and of longitude at 60 deg. */
    double n, e;
    CHECK(core_nav_local_from_geo(0.0, 0.0, 0.001, 0.0, &n, &e));
    CHECK(fabs(n - R_EARTH * 0.001 * DEG) < 1e-9 && fabs(e) < 1e-9);
    CHECK(core_nav_local_from_geo(60.0, 10.0, 60.0, 10.001, &n, &e));
    CHECK(fabs(e - 0.5 * R_EARTH * 0.001 * DEG) < 1e-6 && fabs(n) < 1e-9);
    /* Antimeridian: a short hop east, not most of the way round. */
    CHECK(core_nav_local_from_geo(0.0, 179.9995, 0.0, -179.9995, &n, &e));
    CHECK(e > 100.0 && e < 120.0);
    CHECK(core_nav_local_from_geo(0.0, -179.9995, 0.0, 179.9995, &n, &e));
    CHECK(e < -100.0 && e > -120.0);
    /* Garbage in, nothing out. */
    CHECK(!core_nav_local_from_geo(NAN, 0, 0, 0, &n, &e));
    CHECK(!core_nav_local_from_geo(0, 0, 0, INFINITY, &n, &e));
    CHECK(!core_nav_local_from_geo(91.0, 0, 0, 0, &n, &e));
    CHECK(!core_nav_local_from_geo(0, 0, 0, 181.0, &n, &e));
    CHECK(!core_nav_local_from_geo(89.0, 0, 89.0, 0.1, &n, &e)); /* projection degenerate near a pole */
}

static void test_nothing_before_ready(void) {
    rig_t r;
    rig_init(&r, 0, 0, 0, 0);

    /* No telemetry at all. */
    core_telemetry_t empty;
    memset(&empty, 0, sizeof(empty));
    core_nav_estimate_t e = rig_step_with(&r, &empty);
    CHECK(e.status == CORE_NAV_EST_NO_HOME);

    /* IMU and home, but no GPS fix yet: the filter has no anchor. */
    core_telemetry_t t = make_telemetry(&r.v, false, 0);
    e = rig_step_with(&r, &t);
    CHECK(e.status == CORE_NAV_EST_NOT_ANCHORED);

    /* Home of 0,0 is ArduPilot's "not set". */
    t = make_telemetry(&r.v, true, 0);
    t.home_position.lat = t.home_position.lon = 0;
    e = rig_step_with(&r, &t);
    CHECK(e.status == CORE_NAV_EST_NO_HOME);
    CHECK(r.detector.kf_long.anchored);

    /* Anchored and home known, but no attitude source to rotate the IMU by. */
    t = make_telemetry(&r.v, false, 0);
    t.attitude.valid = false;
    e = rig_step_with(&r, &t);
    CHECK(e.status == CORE_NAV_EST_ATTITUDE_UNAVAILABLE);

    t = make_telemetry(&r.v, false, 0);
    e = rig_step_with(&r, &t);
    CHECK(e.status == CORE_NAV_EST_READY);

    /* publish() must refuse anything that isn't ready, before touching conn,
     * and NULL arguments anywhere. */
    core_nav_estimate_t not_ready = {.status = CORE_NAV_EST_NOT_ANCHORED};
    CHECK(!core_nav_estimate_publish(NULL, &not_ready));
    CHECK(!core_nav_estimate_publish(NULL, &e)); /* e is READY */
    CHECK(!core_nav_estimate_publish((core_mavlink_conn_t *)&r, NULL));
    double n = 1.0, east = 2.0;
    CHECK(!core_nav_local_from_geo(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON, NULL, &east));
    CHECK(!core_nav_local_from_geo(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON, &n, NULL));
    CHECK(n == 1.0 && east == 2.0);
    CHECK(core_nav_estimator_update(NULL, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    CHECK(core_nav_estimator_update(&r.estimator, NULL, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, NULL, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, NULL, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, NULL, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
}

/* The KF's origin is its last GPS anchor; the output must be relative to home,
 * not to that anchor. Start well away from home so the two differ. */
static void test_home_relative_in_normal_flight(void) {
    rig_t r;
    rig_init(&r, 300.0, -150.0, 4.0f, 3.0f);
    double worst = 0.0;
    int ready = 0;
    for (int i = 0; i < 1500; i++) { /* 30 s, several long-horizon re-anchors */
        core_nav_estimate_t e = rig_step(&r, 0.0);
        if (i < 5) continue;
        CHECK(e.status == CORE_NAV_EST_READY);
        ready++;
        truth_t at = r.v; /* undo the step to compare at the sample's time */
        at.north_m -= at.vn * 0.02;
        at.east_m -= at.ve * 0.02;
        double err = est_error_m(&e, &at);
        if (err > worst) worst = err;
        CHECK(e.dead_reckon_ms <= r.nav_cfg.dr_long_horizon_ms + 300);
    }
    CHECK(ready > 1400);
    CHECK(worst < 0.5); /* ~1 cm rounding of degE7 plus the 0.1% scale mismatch over one horizon */

    /* Moving home moves the published value by exactly the projected shift. */
    core_telemetry_t t = make_telemetry(&r.v, false, 0);
    core_nav_estimate_t before =
        core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms);
    t.home_position.lat += 1000; /* 0.0001 deg north */
    core_nav_estimate_t after =
        core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms);
    CHECK(before.status == CORE_NAV_EST_READY && after.status == CORE_NAV_EST_READY);
    CHECK(fabs((before.north_m - after.north_m) - R_EARTH * 0.0001 * DEG) < 1e-3);
    CHECK(fabs(before.east_m - after.east_m) < 1e-2);
}

static void test_stale_imu_withholds_until_reanchor(void) {
    rig_t r;
    rig_init(&r, 0, 0, 5.0f, 0.0f);
    for (int i = 0; i < 100; i++) CHECK(rig_step(&r, 0.0).status == CORE_NAV_EST_READY || i < 5);

    /* IMU stops; GPS, attitude and home continue. */
    uint64_t frozen = r.v.t_ms - 20;
    core_nav_estimate_t e;
    for (int i = 0; i < 40; i++) {
        core_telemetry_t t = make_telemetry(&r.v, r.tick % 10 == 0, 0);
        t.imu.last_update_ms = frozen;
        e = rig_step_with(&r, &t);
        if (r.v.t_ms - frozen > r.est_cfg.imu_max_age_ms + 20) CHECK(e.status == CORE_NAV_EST_IMU_STALE);
    }
    CHECK(e.status == CORE_NAV_EST_IMU_STALE);

    /* Stream returns: still withheld until a re-anchor after the return. */
    uint64_t anchor_at_return = r.detector.kf_long.anchor_time_ms;
    int ticks = 0;
    do {
        e = rig_step(&r, 0.0);
        ticks++;
        if (r.detector.kf_long.anchor_time_ms == anchor_at_return) CHECK(e.status == CORE_NAV_EST_AWAIT_REANCHOR);
    } while (e.status != CORE_NAV_EST_READY && ticks < 1000);
    CHECK(e.status == CORE_NAV_EST_READY);
    CHECK(r.detector.kf_long.anchor_time_ms != anchor_at_return);
    CHECK(est_error_m(&e, &r.v) < 0.5);
}

/* IMU and GPS both stop while the autopilot link (home, attitude) stays up:
 * the detector's IMU-paced clock freezes, but the estimate must still go. */
static void test_imu_and_gps_silence_withheld(void) {
    rig_t r;
    rig_init(&r, 10.0, 10.0, 3.0f, 0.0f);
    for (int i = 0; i < 100; i++) rig_step(&r, 0.0);
    core_telemetry_t frozen = make_telemetry(&r.v, true, 0);
    frozen.imu.last_update_ms = frozen.gps_raw.last_update_ms = r.v.t_ms - 20;
    core_nav_estimate_t e = {0};
    for (int i = 0; i < 50; i++) {
        frozen.attitude.last_update_ms = frozen.home_position.last_update_ms = r.v.t_ms;
        e = rig_step_with(&r, &frozen);
    }
    CHECK(e.status == CORE_NAV_EST_IMU_STALE);
}

static void test_imu_absent_from_start_is_not_a_gap(void) {
    rig_t r;
    rig_init(&r, 0, 0, 0, 0);
    core_telemetry_t t = make_telemetry(&r.v, true, 0);
    t.imu.valid = false;
    CHECK(rig_step_with(&r, &t).status == CORE_NAV_EST_IMU_STALE);
    CHECK(!r.estimator.awaiting_reanchor);
    rig_step(&r, 0.0);
    CHECK(rig_step(&r, 0.0).status == CORE_NAV_EST_READY);
}

static void test_dead_reckoning_age_limit(void) {
    rig_t r;
    rig_init(&r, 0, 0, 2.0f, 0.0f);
    r.est_cfg.max_dead_reckon_ms = 20000;
    for (int i = 0; i < 50; i++) rig_step(&r, 0.0);
    /* GPS stops: no more anchors, the filter dead-reckons on. */
    core_nav_estimate_t e;
    for (int i = 0; i < 1100; i++) { /* 22 s */
        core_telemetry_t t = make_telemetry(&r.v, false, 0);
        e = rig_step_with(&r, &t);
        if (e.dead_reckon_ms <= r.est_cfg.max_dead_reckon_ms) CHECK(e.status == CORE_NAV_EST_READY);
        else CHECK(e.status == CORE_NAV_EST_DR_EXPIRED);
    }
    CHECK(e.status == CORE_NAV_EST_DR_EXPIRED);
}

static void test_invalid_values_withheld(void) {
    rig_t r;
    rig_init(&r, 0, 0, 0, 0);
    for (int i = 0; i < 20; i++) rig_step(&r, 0.0);

    core_telemetry_t t = make_telemetry(&r.v, false, 0);
    t.home_position.lat += 20000000; /* home 2 deg (~222 km) away: past the bound */
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);

    t = make_telemetry(&r.v, false, 0);
    t.home_position.lat = 950000000; /* out of range */
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_NO_HOME);

    /* Home is a valid position, but too near the pole for the projection. */
    t = make_telemetry(&r.v, false, 0);
    t.home_position.lat = 890000000;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);

    /* A non-finite or out-of-range KF anchor fails projection. */
    t = make_telemetry(&r.v, false, 0);
    double saved_lat = r.detector.kf_long.anchor_lat_deg, saved_lon = r.detector.kf_long.anchor_lon_deg;
    r.detector.kf_long.anchor_lat_deg = NAN;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    r.detector.kf_long.anchor_lat_deg = saved_lat;
    r.detector.kf_long.anchor_lon_deg = INFINITY;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    r.detector.kf_long.anchor_lon_deg = 200.0;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
    r.detector.kf_long.anchor_lon_deg = saved_lon;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_READY);

    t = make_telemetry(&r.v, false, 0);
    r.detector.kf_long.east.p = NAN;
    CHECK(core_nav_estimator_update(&r.estimator, &r.detector, &t, &r.nav_cfg, &r.est_cfg, r.v.t_ms).status ==
          CORE_NAV_EST_INVALID);
}

/* 1.5 m/s spoof north, hovering truth: GPS walks away, the estimate stays
 * until the dead-reckoning cap, then is withdrawn rather than left to rot. */
static void test_slow_spoof_not_followed(void) {
    rig_t r;
    rig_init(&r, 40.0, 25.0, 0.0f, 0.0f);
    for (int i = 0; i < 500; i++) rig_step(&r, 0.0); /* 10 s clean */

    double offset = 0.0, worst = 0.0, widest_gap = 0.0;
    core_nav_estimate_t e = {0};
    for (int i = 0; i < 2000; i++) { /* 40 s of spoof */
        offset += 1.5 * 0.02;
        e = rig_step(&r, offset);
        if (e.status == CORE_NAV_EST_READY) {
            double err = est_error_m(&e, &r.v);
            if (err > worst) worst = err;
            if (offset - err > widest_gap) widest_gap = offset - err;
            CHECK(e.dead_reckon_ms <= r.est_cfg.max_dead_reckon_ms);
        } else {
            CHECK(e.status == CORE_NAV_EST_DR_EXPIRED);
        }
    }
    CHECK(e.status == CORE_NAV_EST_DR_EXPIRED);
    /* At most the drift absorbed before the long check reached warn (~10 m)
     * is carried, while the gap to GPS visibly opens. */
    CHECK(worst < 12.5);
    CHECK(widest_gap > 15.0);
}

static void test_operator_rebaseline(void) {
    rig_t r;
    rig_init(&r, 40, 25, 0, 0);
    for (int i = 0; i < 500; ++i) rig_step(&r, 0);
    for (int i = 0; i < 4000; ++i) {
        rig_step(&r, (i + 1) * 0.03);
        if (i > 1500) CHECK(r.detector.nav_trust < 0.1f);
    }
    CHECK(r.detector.nav_trust < 0.1f);
    uint64_t anchor = r.detector.kf_long.anchor_time_ms;
    /* Long after the attack starts, honest fixes alone may still not recover
     * if the unanchored IMU prediction has drifted. Force that condition. */
    r.detector.kf_short.north.p = r.detector.kf_long.north.p = 1000.0f;
    for (int i = 0; i < 500; ++i) rig_step(&r, 0);
    CHECK(r.detector.nav_trust < 0.1f);
    core_telemetry_t t = make_telemetry(&r.v, true, 0);
    t.heartbeat.valid = true;
    t.heartbeat.last_update_ms = r.v.t_ms;
    core_nav_detector_update(&r.detector, &t, &r.nav_cfg);
    float trust = r.detector.nav_trust;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, false));
    t.heartbeat.base_mode = 128;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, true));
    t.heartbeat.base_mode = 0;
    t.imu.xacc_mps2 = NAN;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, true));
    t.imu.xacc_mps2 = 0;
    t.gps_raw.vel_cms = UINT16_MAX;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, true));
    t.gps_raw.vel_cms = 0;
    t.gps_raw.eph = UINT16_MAX;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, true));
    t.gps_raw.eph = 120;
    CHECK(!core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms + 3000, true));
    CHECK(r.detector.kf_long.anchor_time_ms == anchor);
    CHECK(core_nav_detector_operator_rebaseline(&r.detector, &t, &r.nav_cfg, r.v.t_ms, true));
    CHECK(r.detector.nav_trust == trust); /* never force trust */
    for (int i = 0; i < 3000; ++i) rig_step(&r, 0);
    CHECK(r.detector.nav_trust > 0.99f);
}

static bool backup_step(rig_t *r, core_nav_backup_recovery_t *recovery, core_gps_raw_t *last_gps,
                        double offset, bool permitted, bool bad_velocity) {
    core_telemetry_t t = make_telemetry(&r->v, r->tick % 10 == 0, offset);
    if (t.gps_raw.valid) *last_gps = t.gps_raw;
    else t.gps_raw = *last_gps;
    t.global_position.vx = (int16_t)lrintf(r->v.vn * 100);
    t.global_position.vy = (int16_t)lrintf(r->v.ve * 100) + (bad_velocity ? 200 : 0);
    t.heartbeat.valid = true;
    t.heartbeat.last_update_ms = r->v.t_ms;
    core_nav_detector_update(&r->detector, &t, &r->nav_cfg);
    float trust_before = r->detector.nav_trust;
    bool recovered = core_nav_detector_recover_from_backup(&r->detector, recovery, &t, r->v.t_ms, permitted);
    CHECK(r->detector.nav_trust == trust_before);
    r->v.t_ms += 20;
    r->v.north_m += r->v.vn * 0.02;
    r->v.east_m += r->v.ve * 0.02;
    ++r->tick;
    return recovered;
}

static void test_backup_recovery(void) {
    for (int rate = 0; rate < 2; ++rate) {
        rig_t r;
        core_nav_backup_recovery_t rec = {0};
        core_gps_raw_t gps = {0};
        rig_init(&r, 40, 25, 2, 1);
        for (int i = 0; i < 500; ++i) CHECK(!backup_step(&r, &rec, &gps, 0, true, false));
        for (int i = 0; i < 4000; ++i) {
            CHECK(!backup_step(&r, &rec, &gps, (i + 1) * (rate ? 3.0 : 1.5) * .02, true, false));
            if (i > 1500) CHECK(r.detector.nav_trust < 0.1f);
        }
        r.detector.kf_short.north.p = r.detector.kf_long.north.p = 1000;
        for (int i = 0; i < 600; ++i) CHECK(!backup_step(&r, &rec, &gps, 0, false, false));
        CHECK(r.detector.nav_trust < 0.1f);
        for (int i = 0; i < 600; ++i) CHECK(!backup_step(&r, &rec, &gps, 0, true, true));
        for (int i = 0; i < 350; ++i) CHECK(!backup_step(&r, &rec, &gps, 0, true, false));
        /* A single missing backup or non-finite IMU sample cancels the full
         * window, rather than permitting a partial-window reacquisition. */
        core_telemetry_t invalid = make_telemetry(&r.v, false, 0);
        invalid.gps_raw = gps;
        invalid.heartbeat.valid = true;
        invalid.heartbeat.last_update_ms = r.v.t_ms;
        invalid.imu.xacc_mps2 = NAN;
        CHECK(!core_nav_detector_recover_from_backup(&r.detector, &rec, &invalid, r.v.t_ms, true));
        CHECK(rec.start_ms == 0);
        bool recovered = false;
        for (int i = 0; i < 1500; ++i) recovered |= backup_step(&r, &rec, &gps, 0, true, false);
        CHECK(recovered);
        CHECK(r.detector.nav_trust > .99f);
    }
}

int main(void) {
    test_projection_matches_dashboard_inverse();
    test_nothing_before_ready();
    test_home_relative_in_normal_flight();
    test_stale_imu_withholds_until_reanchor();
    test_imu_and_gps_silence_withheld();
    test_imu_absent_from_start_is_not_a_gap();
    test_dead_reckoning_age_limit();
    test_invalid_values_withheld();
    test_slow_spoof_not_followed();
    test_operator_rebaseline();
    test_backup_recovery();
    printf("test_nav_estimate: all passed\n");
    return 0;
}
