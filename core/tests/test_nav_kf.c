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

/* Specific force an unaccelerated vehicle at this attitude reads: R^T*[0,0,-g],
 * written out independently of the code under test. */
static core_imu_t make_tilted_imu(float roll, float pitch, uint64_t t_ms) {
    const float g = 9.80665f;
    return make_imu(g * sinf(pitch), -g * sinf(roll) * cosf(pitch), -g * cosf(roll) * cosf(pitch),
                    0.0f, 0.0f, 0.0f, t_ms);
}

/* Attitude filter: a steady tilt must come back with its own sign. Forward
 * flight is nose-down (negative pitch) and banked; a filter that mirrors
 * tilt passes the level test above but fails this. */
static void test_attitude_filter_tilt_sign(void) {
    const float cases[][2] = {{0.0f, -0.35f}, {0.0f, 0.35f}, {0.3f, 0.0f}, {-0.3f, 0.0f}, {0.25f, -0.3f}};
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        core_attitude_filter_t filter;
        core_attitude_filter_init(&filter, 1.0f);
        uint64_t t = 1000;
        for (int i = 0; i < 500; i++) { /* 10s at 50Hz, many time constants */
            core_imu_t imu = make_tilted_imu(cases[c][0], cases[c][1], t);
            core_attitude_filter_update(&filter, &imu);
            t += 20;
        }
        assert(approx(filter.roll, cases[c][0], 0.01f));
        assert(approx(filter.pitch, cases[c][1], 0.01f));
    }
}

/* Attitude filter: a copter tilted forward and accelerating reads pure thrust
 * along its own z-axis - the accelerometer says "level" whatever the real
 * tilt. The filter must hold the gyro's tilt through that, not decay toward
 * level; decaying lagged pitch 10-15 deg behind the truth through every
 * speed-up in SITL and false-alarmed the long-horizon KF check. */
static void test_attitude_filter_holds_tilt_while_accelerating(void) {
    const float g = 9.80665f, pitch = -0.3f;
    core_attitude_filter_t filter;
    core_attitude_filter_init(&filter, 1.0f);
    uint64_t t = 1000;
    core_imu_t still = make_tilted_imu(0.0f, pitch, t);
    core_attitude_filter_update(&filter, &still); /* initializes at the true tilt */
    for (int i = 0; i < 150; i++) { /* 3s of acceleration, gyro still */
        t += 20;
        core_imu_t imu = make_imu(0.0f, 0.0f, -g / cosf(pitch), 0.0f, 0.0f, 0.0f, t);
        core_attitude_filter_update(&filter, &imu);
    }
    assert(approx(filter.pitch, pitch, 0.02f));
}

/* Attitude filter: a biased gyro, learned on the pad, must stop pulling the
 * tilt away from the accelerometer's in flight. Unlearned, a complementary
 * filter settles tau*bias off the truth - in SITL ~1-2 deg of pitch in cruise,
 * which the KFs integrate as g*sin(that) of phantom deceleration. */
static void test_attitude_filter_learns_gyro_bias(void) {
    const float bias = 0.005f; /* ~0.3 deg/s */
    core_attitude_filter_t filter;
    core_attitude_filter_init(&filter, 1.0f);
    uint64_t t = 1000;
    for (int i = 0; i < 1500; i++) { /* 30s disarmed on the pad */
        core_imu_t imu = make_imu(0.0f, 0.0f, -9.80665f, 0.0f, bias, 0.0f, t);
        core_attitude_filter_learn_gyro_bias(&filter, &imu);
        core_attitude_filter_update(&filter, &imu);
        t += 20;
    }
    assert(approx(filter.gyro_bias[1], bias, 0.0002f));

    for (int i = 0; i < 500; i++) { /* 10s cruising nose-down, same bias */
        core_imu_t imu = make_tilted_imu(0.0f, -0.5f, t);
        imu.ygyro_rps = bias;
        core_attitude_filter_update(&filter, &imu);
        t += 20;
    }
    assert(approx(filter.pitch, -0.5f, 0.002f));
}

/* KF set: a vehicle cruising at constant velocity, tilted nose-down and
 * banked, has zero true acceleration, so the dead-reckoned position must
 * keep advancing at the anchor velocity and not pick up phantom
 * acceleration from the tilt. This is the unspoofed-cruise false positive
 * the inverted-tilt bug produced. */
static void test_kf_set_tilted_cruise_no_phantom_accel(void) {
    core_attitude_filter_t att;
    core_attitude_filter_init(&att, 1.0f);
    core_nav_kf_set_t set;
    core_nav_kf_set_init(&set, 8000, 5);

    const float roll = 0.1f, pitch = -0.3f; /* ~17 deg nose down, as at ~10 m/s */
    uint64_t t = 1000;
    for (int i = 0; i < 250; i++) { /* let the attitude converge first */
        core_imu_t imu = make_tilted_imu(roll, pitch, t);
        core_attitude_filter_update(&att, &imu);
        t += 20;
    }

    core_gps_raw_t gps;
    memset(&gps, 0, sizeof(gps));
    gps.lat = 473977420;
    gps.lon = -1223974560;
    gps.vel_cms = 1000; /* 10 m/s */
    gps.cog_cdeg = 0;   /* north */
    gps.last_update_ms = t;
    gps.valid = true;
    core_nav_kf_set_reanchor(&set, &gps);

    core_baro_t baro;
    memset(&baro, 0, sizeof(baro));
    for (int i = 0; i < 400; i++) { /* 8s */
        core_imu_t imu = make_tilted_imu(roll, pitch, t);
        core_attitude_filter_update(&att, &imu);
        core_nav_kf_set_update(&set, &imu, &att, &baro, 0.5f, 0.25f);
        t += 20;
    }

    assert(approx(set.north.v, 10.0f, 0.2f));
    assert(approx(set.east.v, 0.0f, 0.2f));
    assert(approx(set.north.p, 80.0f, 1.0f)); /* 10 m/s * 8s */
    assert(approx(set.east.p, 0.0f, 1.0f));
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
    test_attitude_filter_tilt_sign();
    test_attitude_filter_holds_tilt_while_accelerating();
    test_attitude_filter_learns_gyro_bias();
    test_kf_set_tilted_cruise_no_phantom_accel();
    test_kf_set_reanchor_and_hold();
    return 0;
}
