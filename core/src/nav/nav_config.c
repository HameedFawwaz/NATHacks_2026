#include "core/nav/nav_config.h"

void core_nav_config_default(core_nav_config_t *cfg) {
    cfg->max_plausible_speed_mps = 35.0f;
    cfg->jump_hard_reject_m = 30.0f;

    cfg->dr_short_horizon_ms = 1500;
    cfg->dr_short_divergence_warn_m = 8.0f;
    cfg->dr_short_divergence_reject_m = 20.0f;

    cfg->dr_long_horizon_ms = 15000;
    cfg->dr_long_divergence_warn_m = 15.0f;
    cfg->dr_long_divergence_reject_m = 40.0f;

    cfg->ekf_divergence_warn_m = 10.0f;
    cfg->ekf_divergence_reject_m = 25.0f;

    cfg->eph_nominal_max = 200;
    cfg->eph_reject = 500;
    cfg->eph_jump_threshold = 150;

    cfg->trust_recover_rate_per_s = 0.5f;

    cfg->attitude_tau_s = 1.0f;

    cfg->kf_sigma_accel_mps2 = 0.5f;
    cfg->kf_baro_r_m2 = 0.25f;
    cfg->baro_ground_ref_samples = 20;
}
