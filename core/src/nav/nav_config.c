#include "core/nav/nav_config.h"

void core_nav_config_default(core_nav_config_t *cfg) {
    cfg->max_plausible_speed_mps = 35.0f;
    cfg->jump_hard_reject_m = 30.0f;

    /*
     * Thresholds below are calibrated against real ArduCopter SITL flight
     * (core/sitl/fly_square.py), not just synthetic/stationary data: the
     * simplified filter (no accel/gyro bias calibration, small-angle Euler
     * integration, and in this environment only ~4Hz IMU samples rather
     * than a proper 50Hz+ - SITL/MAVProxy didn't honor our
     * SET_MESSAGE_INTERVAL request to speed it up) accumulates roughly
     * bias-driven, ~quadratically-growing divergence during normal
     * maneuvering: under ~2m through 10s, but growing fast after that
     * (~16m by 15s). A long horizon that lets that compound unchecked was
     * producing false positives on completely normal flight - shortening it
     * has an outsized benefit precisely because the error grows with t^2,
     * not linearly.
     *
     * The long-horizon warn/reject below were tightened from an earlier,
     * looser pass (20m/50m) once re-anchoring was gated on divergence (see
     * gnss_anomaly.c): since a degraded filter no longer gets silently
     * re-synced to GPS every horizon, the minimum sustained-drift rate this
     * can detect is now warn_m / horizon_ms = 10/8 = 1.25 m/s, with ~5-7x
     * margin over the observed normal-flight baseline above. A slower,
     * more patient spoof than that remains undetectable without real IMU
     * bias estimation - see core/README.md's "Known limitation".
     */
    cfg->dr_short_horizon_ms = 1500;
    cfg->dr_short_divergence_warn_m = 10.0f; /* real flight briefly touched ~9m */
    cfg->dr_short_divergence_reject_m = 20.0f;

    cfg->dr_long_horizon_ms = 8000;
    cfg->dr_long_divergence_warn_m = 10.0f;
    cfg->dr_long_divergence_reject_m = 30.0f;

    cfg->ekf_divergence_warn_m = 10.0f;
    cfg->ekf_divergence_reject_m = 25.0f;

    cfg->eph_nominal_max = 200;
    cfg->eph_reject = 500;
    cfg->eph_jump_threshold = 150;

    cfg->gps_stale_warn_ms = 2000;
    cfg->gps_stale_reject_ms = 5000;

    cfg->imu_stale_ms = 2000;

    cfg->trust_recover_rate_per_s = 0.5f;

    cfg->attitude_tau_s = 1.0f;

    cfg->kf_sigma_accel_mps2 = 0.5f;
    cfg->kf_baro_r_m2 = 0.25f;
    cfg->baro_ground_ref_samples = 20;
}
