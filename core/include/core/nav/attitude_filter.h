#ifndef CORE_NAV_ATTITUDE_FILTER_H
#define CORE_NAV_ATTITUDE_FILTER_H

#include <stdint.h>

#include "core/mavlink/mavlink_telemetry.h"

/*
 * Complementary filter estimating roll/pitch/yaw purely from gyro+accel
 * (core_imu_t) - deliberately independent of the flight controller's own
 * ATTITUDE estimate, so it carries no GPS dependency of any kind.
 *
 * Simplification (documented, not a bug): treats body angular rate as
 * directly equal to Euler angle rate (no tan(pitch) conversion), valid for
 * the attitude range multirotors normally fly in. Yaw has no accel/mag
 * correction available and free-integrates from gyro alone, so it will
 * drift slowly over time.
 *
 * Yaw DOES matter to the position filter: nav_kf.c rotates body-frame
 * accelerometer readings into earth-frame using roll/pitch/yaw together, so
 * a wrong yaw misdirects the estimated earth-frame acceleration, which
 * shows up as spurious KF divergence. Yaw used to always initialize to 0
 * (true north) regardless of the vehicle's actual heading, which produced
 * heading-dependent false positives - see core_attitude_filter_seed_yaw.
 */
typedef struct {
    float roll, pitch, yaw; /* rad */
    float tau_s;            /* complementary filter time constant */
    uint64_t last_update_ms;
    bool initialized;
} core_attitude_filter_t;

void core_attitude_filter_init(core_attitude_filter_t *filter, float tau_s);

/* No-op if imu->valid is false. */
void core_attitude_filter_update(core_attitude_filter_t *filter, const core_imu_t *imu);

/* Overrides yaw directly - intended to be called ONCE, as soon as a real
 * heading reference (e.g. GPS course-over-ground) becomes available, to
 * replace the arbitrary initial guess with something close to reality.
 * Calling it repeatedly would fight the gyro integration; callers should
 * track whether they've already seeded it. */
void core_attitude_filter_seed_yaw(core_attitude_filter_t *filter, float yaw_rad);

#endif /* CORE_NAV_ATTITUDE_FILTER_H */
