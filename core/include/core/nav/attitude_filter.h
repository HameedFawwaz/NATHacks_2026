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
 * drift slowly over time - acceptable since yaw isn't used by the
 * short/long-horizon re-anchored position filters' divergence checks in a
 * way that's sensitive to slow yaw drift (see nav_kf.h).
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

#endif /* CORE_NAV_ATTITUDE_FILTER_H */
