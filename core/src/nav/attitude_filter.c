#include "core/nav/attitude_filter.h"

#include <math.h>
#include <string.h>

#define ATTITUDE_GRAVITY_MPS2 9.80665f
/* Past this far from 1 g, the accelerometer's tilt reading is ignored - see
 * core_attitude_filter_update. sqrt(g^2 + a^2) - g = 0.3 at a = 2.4 m/s^2. */
#define ACCEL_TRUST_BAND_MPS2 0.3f

#define GYRO_BIAS_TAU_S 5.0f
#define GYRO_BIAS_MAX_RPS 0.05f /* ~3 deg/s: beyond any real MEMS gyro's bias */

void core_attitude_filter_init(core_attitude_filter_t *filter, float tau_s) {
    memset(filter, 0, sizeof(*filter));
    filter->tau_s = tau_s;
    filter->initialized = false;
}

void core_attitude_filter_update(core_attitude_filter_t *filter, const core_imu_t *imu) {
    if (!imu->valid) {
        return;
    }

    /*
     * Body frame is NED-aligned-when-level (FRD, z-axis down), matching the
     * convention assumed by nav_kf.c's gravity compensation: a stationary,
     * level vehicle reads specific force ~[0,0,-g] (accelerometer measures
     * the reaction force opposing free-fall, which points up physically but
     * reads negative on the down-pointing body z-axis). With R the body->NED
     * rotation nav_kf.c uses, an unaccelerated vehicle reads
     * f_body = R^T * [0,0,-g], i.e. -g times R's bottom row:
     *   xacc = g*sin(pitch), yacc = -g*sin(roll)*cos(pitch), zacc = -g*cos(roll)*cos(pitch)
     * (nose down = negative pitch = negative xacc). Both formulas below are
     * the exact (not small-angle) inverses of that:
     * roll:  atan2(-yacc, -zacc) - cos(pitch) cancels.
     * pitch: atan2(xacc, sqrt(yacc^2+zacc^2)) - the sqrt is g*cos(pitch).
     *
     * These must stay consistent with rotate_body_to_ned in nav_kf.c. They
     * once had both signs inverted, which a hover never shows (tilt ~0) but
     * which, in forward flight, rotates thrust through the mirrored tilt and
     * invents ~g*sin(2*tilt) of horizontal acceleration - tens of metres of
     * false KF divergence within seconds of an unspoofed cruise.
     */
    float roll_acc = atan2f(-imu->yacc_mps2, -imu->zacc_mps2);
    float pitch_acc = atan2f(imu->xacc_mps2,
                              sqrtf(imu->yacc_mps2 * imu->yacc_mps2 + imu->zacc_mps2 * imu->zacc_mps2));

    if (!filter->initialized) {
        /* No prior gyro-integrated estimate to blend with yet - start level
         * estimate straight from accel. Yaw starts at 0 (true north) only as
         * a placeholder until core_attitude_filter_seed_yaw replaces it with
         * a real heading reference; the caller is expected to seed it as
         * soon as one is available. */
        filter->roll = roll_acc;
        filter->pitch = pitch_acc;
        filter->yaw = 0.0f;
        filter->last_update_ms = imu->last_update_ms;
        filter->initialized = true;
        return;
    }

    float dt = (float)(imu->last_update_ms - filter->last_update_ms) / 1000.0f;
    if (dt <= 0.0f) {
        return; /* stale or duplicate sample */
    }

    /* Body rates -> Euler (ZYX) rates. Not the small-angle p,q,r shortcut: a
     * copter cruising pitched ~30 deg and yawing through a corner would leak
     * r*tan(pitch) of its yaw rate into roll that the shortcut never sees. */
    float sr = sinf(filter->roll), cr = cosf(filter->roll);
    float cp = cosf(filter->pitch);
    if (fabsf(cp) < 0.1f) cp = (cp < 0.0f) ? -0.1f : 0.1f; /* gimbal lock: nowhere a copter flies */
    float tp = sinf(filter->pitch) / cp;
    float p = imu->xgyro_rps - filter->gyro_bias[0];
    float q = imu->ygyro_rps - filter->gyro_bias[1];
    float r = imu->zgyro_rps - filter->gyro_bias[2];
    float roll_gyro = filter->roll + (p + (q * sr + r * cr) * tp) * dt;
    float pitch_gyro = filter->pitch + (q * cr - r * sr) * dt;
    float yaw = filter->yaw + ((q * sr + r * cr) / cp) * dt; /* free-integrated, drifts */

    /* The accelerometer only measures tilt when the vehicle isn't
     * accelerating. A multirotor accelerating horizontally reads nearly pure
     * thrust along its own z-axis, i.e. "level", whatever its real tilt: with
     * the correction always on, pitch lagged 10-15 deg behind the truth all
     * the way through every speed-up and slow-down, and the KFs integrated
     * g*sin(that) of phantom horizontal acceleration. So the correction is
     * weighted by how close the measured specific force is to 1 g, and off
     * entirely past ACCEL_TRUST_BAND_MPS2 from it (~2.4 m/s^2 of horizontal
     * acceleration), leaving the gyro to carry the estimate. */
    float norm = sqrtf(imu->xacc_mps2 * imu->xacc_mps2 + imu->yacc_mps2 * imu->yacc_mps2 +
                       imu->zacc_mps2 * imu->zacc_mps2);
    float trust = 1.0f - fabsf(norm - ATTITUDE_GRAVITY_MPS2) / ACCEL_TRUST_BAND_MPS2;
    if (trust < 0.0f) trust = 0.0f;

    float alpha = filter->tau_s / (filter->tau_s + dt * trust);
    filter->roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    filter->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    filter->yaw = yaw;

    filter->last_update_ms = imu->last_update_ms;
}

void core_attitude_filter_learn_gyro_bias(core_attitude_filter_t *filter, const core_imu_t *imu) {
    if (!imu->valid) {
        return;
    }
    if (filter->bias_last_ms == 0 || imu->last_update_ms <= filter->bias_last_ms) {
        filter->bias_last_ms = imu->last_update_ms;
        return;
    }
    float dt = (float)(imu->last_update_ms - filter->bias_last_ms) / 1000.0f;
    filter->bias_last_ms = imu->last_update_ms;
    float k = dt / (GYRO_BIAS_TAU_S + dt);
    const float rates[3] = {imu->xgyro_rps, imu->ygyro_rps, imu->zgyro_rps};
    for (int i = 0; i < 3; i++) {
        float b = filter->gyro_bias[i] + k * (rates[i] - filter->gyro_bias[i]);
        if (b > GYRO_BIAS_MAX_RPS) b = GYRO_BIAS_MAX_RPS;
        if (b < -GYRO_BIAS_MAX_RPS) b = -GYRO_BIAS_MAX_RPS;
        filter->gyro_bias[i] = b;
    }
}

void core_attitude_filter_seed_yaw(core_attitude_filter_t *filter, float yaw_rad) {
    filter->yaw = yaw_rad;
}
