#ifndef CORE_NAV_KF_H
#define CORE_NAV_KF_H

#include <stdbool.h>
#include <stdint.h>

#include "core/mavlink/mavlink_telemetry.h"
#include "core/nav/attitude_filter.h"

/* Generic 2-state (position, velocity) linear Kalman filter for one axis. */
typedef struct {
    float p, v;    /* position (m, relative to the current anchor), velocity (m/s) */
    float P[2][2]; /* covariance */
} core_nav_kf_axis_t;

void core_nav_kf_axis_reset(core_nav_kf_axis_t *axis, float p0, float v0, float p_var0, float v_var0);
void core_nav_kf_axis_predict(core_nav_kf_axis_t *axis, float accel_mps2, float dt_s, float sigma_accel_mps2);
void core_nav_kf_axis_update(core_nav_kf_axis_t *axis, float z, float r_variance);

/*
 * A full NED position/velocity estimate: three axis filters, re-anchored
 * periodically to the last trusted GPS fix (bounding horizontal drift, since
 * the north/east axes never receive a GPS measurement update - by design,
 * nothing GPS-derived ever feeds this filter directly). The down axis is
 * corrected continuously by barometer measurements.
 */
typedef struct {
    core_nav_kf_axis_t north, east, down;

    double anchor_lat_deg, anchor_lon_deg, anchor_alt_m;

    float baro_ground_ref_hpa;
    float baro_ref_sum_hpa;
    uint32_t baro_ref_count;
    uint32_t baro_ref_target_samples;
    bool baro_ground_ref_set;

    uint64_t anchor_time_ms;
    uint64_t last_update_ms;
    uint32_t reanchor_interval_ms;
    bool anchored;
} core_nav_kf_set_t;

void core_nav_kf_set_init(core_nav_kf_set_t *set, uint32_t reanchor_interval_ms, uint32_t baro_ground_ref_samples);

/* Resets the anchor to the given GPS fix (position + velocity-from-GPS as the
 * new zero-offset state) and small "just anchored" covariance. Does NOT
 * touch the barometer ground reference, which is only ever set once, at the
 * first baro_ground_ref_target_samples samples after init. */
void core_nav_kf_set_reanchor(core_nav_kf_set_t *set, const core_gps_raw_t *gps_raw);

/* Predicts all three axes forward using IMU accel rotated into NED via the
 * attitude filter, and applies a barometer measurement update to the down
 * axis once the ground reference pressure has been established. No-op if
 * imu->valid is false. */
void core_nav_kf_set_update(core_nav_kf_set_t *set, const core_imu_t *imu,
                             const core_attitude_filter_t *attitude, const core_baro_t *baro,
                             float sigma_accel_mps2, float baro_r_variance);

void core_nav_kf_set_get_position(const core_nav_kf_set_t *set,
                                   double *lat_deg_out, double *lon_deg_out, double *alt_m_out);

bool core_nav_kf_set_should_reanchor(const core_nav_kf_set_t *set, uint64_t now_ms);

#endif /* CORE_NAV_KF_H */
