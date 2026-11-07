#include "core/nav/attitude_filter.h"

#include <math.h>
#include <string.h>

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
     * reads negative on the down-pointing body z-axis). Both formulas below
     * are the exact (not small-angle) solutions for that convention:
     *   xacc = -g*cos(roll)*sin(pitch), yacc = g*sin(roll), zacc = -g*cos(roll)*cos(pitch)
     * roll:  atan2(yacc, sqrt(xacc^2+zacc^2)) - the sqrt makes this robust
     *        to the sign of zacc, since cos(roll) factors out either way.
     * pitch: atan2(-xacc, -zacc) - exact given the sign convention above.
     */
    float roll_acc = atan2f(imu->yacc_mps2,
                             sqrtf(imu->xacc_mps2 * imu->xacc_mps2 + imu->zacc_mps2 * imu->zacc_mps2));
    float pitch_acc = atan2f(-imu->xacc_mps2, -imu->zacc_mps2);

    if (!filter->initialized) {
        /* No prior gyro-integrated estimate to blend with yet - start level
         * estimate straight from accel, yaw at zero (no reference available). */
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

    float roll_gyro = filter->roll + imu->xgyro_rps * dt;
    float pitch_gyro = filter->pitch + imu->ygyro_rps * dt;
    float yaw = filter->yaw + imu->zgyro_rps * dt; /* free-integrated, drifts */

    float alpha = filter->tau_s / (filter->tau_s + dt);
    filter->roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    filter->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    filter->yaw = yaw;

    filter->last_update_ms = imu->last_update_ms;
}
