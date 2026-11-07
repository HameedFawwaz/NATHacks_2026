#include <assert.h>
#include <math.h>
#include <string.h>

#include "core/nav/attitude_filter.h"
#include "core/nav/nav_kf.h"

static bool approx(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static core_imu_t make_imu(float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro,
                            uint64_t t_ms) {
    core_imu_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.xacc_mps2 = xacc;
    imu.yacc_mps2 = yacc;
    imu.zacc_mps2 = zacc;
    imu.xgyro_rps = xgyro;
    imu.ygyro_rps = ygyro;
    imu.zgyro_rps = zgyro;
    imu.last_update_ms = t_ms;
    imu.valid = true;
    return imu;
}

/* Axis-level KF: stationary (zero accel input) should stay at the origin. */
static void test_axis_stationary(void) {
    core_nav_kf_axis_t axis;
    core_nav_kf_axis_reset(&axis, 0.0f, 0.0f, 1.0f, 0.25f);

    for (int i = 0; i < 100; i++) {
        core_nav_kf_axis_predict(&axis, 0.0f, 0.1f, 0.5f);
    }

    assert(approx(axis.p, 0.0f, 0.001f));
    assert(approx(axis.v, 0.0f, 0.001f));
}

/* Axis-level KF: constant acceleration should match the analytic solution. */
static void test_axis_constant_acceleration(void) {
    core_nav_kf_axis_t axis;
    core_nav_kf_axis_reset(&axis, 0.0f, 0.0f, 1.0f, 0.25f);

    float accel = 2.0f;
    float dt = 0.1f;
    int steps = 10; /* 1 second total */
    for (int i = 0; i < steps; i++) {
        core_nav_kf_axis_predict(&axis, accel, dt, 0.5f);
    }

    float t = steps * dt;
    float expected_v = accel * t;
    float expected_p = 0.5f * accel * t * t;

    assert(approx(axis.v, expected_v, 0.05f));
    assert(approx(axis.p, expected_p, 0.05f));
}

/* Axis-level KF: a measurement update should pull the estimate toward z. */
static void test_axis_measurement_update_pulls_toward_z(void) {
    core_nav_kf_axis_t axis;
    core_nav_kf_axis_reset(&axis, 0.0f, 0.0f, 1.0f, 0.25f);

    core_nav_kf_axis_update(&axis, 5.0f, 0.25f);

    assert(axis.p > 0.0f && axis.p < 5.0f); /* pulled toward 5, not snapped exactly (finite gain) */
    assert(axis.P[0][0] < 1.0f);            /* uncertainty shrinks after a measurement */
}

/* Attitude filter: level, stationary IMU (accel reads ~[0,0,-g] for a level,
 * stationary vehicle per our body-frame convention) should converge to
 * roll=pitch=0. */
static void test_attitude_filter_level_stationary(void) {
    core_attitude_filter_t filter;
    core_attitude_filter_init(&filter, 1.0f);

    uint64_t t = 1000;
    for (int i = 0; i < 50; i++) {
        core_imu_t imu = make_imu(0.0f, 0.0f, -9.80665f, 0.0f, 0.0f, 0.0f, t);
        core_attitude_filter_update(&filter, &imu);
        t += 20;
    }

    assert(approx(filter.roll, 0.0f, 0.01f));
    assert(approx(filter.pitch, 0.0f, 0.01f));
}

/* KF set: with zero IMU input and no re-anchor, position should stay at the
 * anchor (i.e. filtered lat/lon/alt should equal the GPS fix it was anchored
 * to). */
static void test_kf_set_reanchor_and_hold(void) {
    core_nav_kf_set_t set;
    core_nav_kf_set_init(&set, 1500, 5);

    core_gps_raw_t gps;
    memset(&gps, 0, sizeof(gps));
    gps.lat = 473977420;
    gps.lon = -1223974560;
    gps.alt_mm = 10000;
    gps.vel_cms = 0;
    gps.cog_cdeg = 0;
    gps.last_update_ms = 1000;
    gps.valid = true;

    core_nav_kf_set_reanchor(&set, &gps);

    double lat, lon, alt;
    core_nav_kf_set_get_position(&set, &lat, &lon, &alt);

    assert(approx((float)lat, 47.397742f, 0.0001f));
    assert(approx((float)lon, -122.397456f, 0.0001f));
    assert(approx((float)alt, 10.0f, 0.001f));
}

int main(void) {
    test_axis_stationary();
    test_axis_constant_acceleration();
    test_axis_measurement_update_pulls_toward_z();
    test_attitude_filter_level_stationary();
    test_kf_set_reanchor_and_hold();
    return 0;
}
