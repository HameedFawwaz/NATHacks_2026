#ifndef CORE_NAV_CONFIG_H
#define CORE_NAV_CONFIG_H

#include <stdint.h>

/*
 * All tunable thresholds and filter constants for GNSS spoofing detection,
 * gathered in one place rather than hardcoded in detection logic.
 */
typedef struct {
    /* Check 1: jump / implausible speed */
    float max_plausible_speed_mps; /* default 35.0 */
    float jump_hard_reject_m;      /* default 30.0 */

    /* Check 2: short-horizon KF divergence */
    uint32_t dr_short_horizon_ms;   /* default 1500 */
    float dr_short_divergence_warn_m;   /* default 8.0 */
    float dr_short_divergence_reject_m; /* default 20.0 */

    /* Check 3: long-horizon cumulative drift */
    uint32_t dr_long_horizon_ms;     /* default 15000 */
    float dr_long_divergence_warn_m;    /* default 15.0 */
    float dr_long_divergence_reject_m;  /* default 40.0 */

    /* Check 4: raw GPS vs EKF (GLOBAL_POSITION_INT) */
    float ekf_divergence_warn_m;   /* default 10.0 */
    float ekf_divergence_reject_m; /* default 25.0 */

    /* Check 6: HDOP/eph abnormality (C/N0 stand-in) */
    uint16_t eph_nominal_max;     /* default 200 (HDOP 2.0) */
    uint16_t eph_reject;          /* default 500 (HDOP 5.0) */
    uint16_t eph_jump_threshold;  /* default 150 (HDOP 1.5) */

    /* Publish smoothing - the two hard checks (jump, replay/timestamp) latch
     * on a single unambiguous sample by design (see gnss_anomaly.c); this is
     * what prevents a brief spoof event from looking instantly forgiven. */
    float trust_recover_rate_per_s;   /* default 0.5 */

    /* Attitude complementary filter */
    float attitude_tau_s; /* default 1.0 */

    /* Position/velocity Kalman filter */
    float kf_sigma_accel_mps2; /* process noise strength, default 0.5 */
    float kf_baro_r_m2;        /* barometer measurement noise variance, default 0.25 (0.5m stddev) */
    uint32_t baro_ground_ref_samples; /* samples averaged at init for ground pressure ref, default 20 */
} core_nav_config_t;

void core_nav_config_default(core_nav_config_t *cfg);

#endif /* CORE_NAV_CONFIG_H */
