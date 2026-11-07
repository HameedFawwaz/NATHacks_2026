#include "core/nav/nav_kf.h"

#include <math.h>
#include <string.h>

#define GRAVITY_MPS2 9.80665f
#define METERS_PER_DEG_LAT 111320.0

void core_nav_kf_axis_reset(core_nav_kf_axis_t *axis, float p0, float v0, float p_var0, float v_var0) {
    axis->p = p0;
    axis->v = v0;
    axis->P[0][0] = p_var0;
    axis->P[0][1] = 0.0f;
    axis->P[1][0] = 0.0f;
    axis->P[1][1] = v_var0;
}

void core_nav_kf_axis_predict(core_nav_kf_axis_t *axis, float accel_mps2, float dt_s, float sigma_accel_mps2) {
    if (dt_s <= 0.0f) {
        return;
    }

    float p = axis->p + axis->v * dt_s + 0.5f * accel_mps2 * dt_s * dt_s;
    float v = axis->v + accel_mps2 * dt_s;

    /* F = [[1,dt],[0,1]], B = [[0.5dt^2],[dt]], Q = B*B^T * sigma_accel^2 */
    float dt2 = dt_s * dt_s;
    float dt3 = dt2 * dt_s;
    float dt4 = dt2 * dt2;
    float var = sigma_accel_mps2 * sigma_accel_mps2;

    float q_pp = 0.25f * dt4 * var;
    float q_pv = 0.5f * dt3 * var;
    float q_vv = dt2 * var;

    float p00 = axis->P[0][0], p01 = axis->P[0][1];
    float p10 = axis->P[1][0], p11 = axis->P[1][1];

    /* F*P*F^T, F = [[1,dt],[0,1]] */
    float fp00 = p00 + dt_s * p10;
    float fp01 = p01 + dt_s * p11;
    float fp10 = p10;
    float fp11 = p11;

    float new00 = fp00 + dt_s * fp01; /* (F*P) * F^T, row 0 */
    float new01 = fp01;
    float new10 = fp10 + dt_s * fp11;
    float new11 = fp11;

    axis->p = p;
    axis->v = v;
    axis->P[0][0] = new00 + q_pp;
    axis->P[0][1] = new01 + q_pv;
    axis->P[1][0] = new10 + q_pv;
    axis->P[1][1] = new11 + q_vv;
}

void core_nav_kf_axis_update(core_nav_kf_axis_t *axis, float z, float r_variance) {
    float innovation = z - axis->p;
    float s = axis->P[0][0] + r_variance;
    if (s <= 0.0f) {
        return;
    }

    float k0 = axis->P[0][0] / s;
    float k1 = axis->P[1][0] / s;

    axis->p += k0 * innovation;
    axis->v += k1 * innovation;

    float p00 = axis->P[0][0], p01 = axis->P[0][1];
    float p10 = axis->P[1][0], p11 = axis->P[1][1];

    /* P = (I - K*H) * P, H = [1,0] */
    axis->P[0][0] = (1.0f - k0) * p00;
    axis->P[0][1] = (1.0f - k0) * p01;
    axis->P[1][0] = p10 - k1 * p00;
    axis->P[1][1] = p11 - k1 * p01;
}

static void rotate_body_to_ned(float roll, float pitch, float yaw, float ax, float ay, float az,
                                float *an, float *ae, float *ad) {
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    *an = cp * cy * ax + (sr * sp * cy - cr * sy) * ay + (cr * sp * cy + sr * sy) * az;
    *ae = cp * sy * ax + (sr * sp * sy + cr * cy) * ay + (cr * sp * sy - sr * cy) * az;
    *ad = -sp * ax + sr * cp * ay + cr * cp * az;
}

void core_nav_kf_set_init(core_nav_kf_set_t *set, uint32_t reanchor_interval_ms, uint32_t baro_ground_ref_samples) {
    memset(set, 0, sizeof(*set));
    set->reanchor_interval_ms = reanchor_interval_ms;
    set->baro_ref_target_samples = baro_ground_ref_samples;
    core_nav_kf_axis_reset(&set->north, 0.0f, 0.0f, 1.0f, 0.25f);
    core_nav_kf_axis_reset(&set->east, 0.0f, 0.0f, 1.0f, 0.25f);
    core_nav_kf_axis_reset(&set->down, 0.0f, 0.0f, 1.0f, 0.25f);
}

void core_nav_kf_set_reanchor(core_nav_kf_set_t *set, const core_gps_raw_t *gps_raw) {
    if (!gps_raw->valid) {
        return;
    }

    set->anchor_lat_deg = (double)gps_raw->lat / 1e7;
    set->anchor_lon_deg = (double)gps_raw->lon / 1e7;
    set->anchor_alt_m = (double)gps_raw->alt_mm / 1000.0;

    float vel_mps = (float)gps_raw->vel_cms / 100.0f;
    float cog_rad = (gps_raw->cog_cdeg == UINT16_MAX) ? 0.0f : (float)gps_raw->cog_cdeg * (float)M_PI / 18000.0f;
    float vn = vel_mps * cosf(cog_rad);
    float ve = vel_mps * sinf(cog_rad);

    core_nav_kf_axis_reset(&set->north, 0.0f, vn, 1.0f, 0.25f);
    core_nav_kf_axis_reset(&set->east, 0.0f, ve, 1.0f, 0.25f);
    core_nav_kf_axis_reset(&set->down, 0.0f, 0.0f, 1.0f, 0.25f);

    set->anchor_time_ms = gps_raw->last_update_ms;
    set->anchored = true;
}

void core_nav_kf_set_update(core_nav_kf_set_t *set, const core_imu_t *imu,
                             const core_attitude_filter_t *attitude, const core_baro_t *baro,
                             float sigma_accel_mps2, float baro_r_variance) {
    if (!imu->valid) {
        return;
    }

    /* Ground reference pressure: averaged once over the first N baro samples
     * seen, never reset afterward (it's a property of ambient pressure at
     * boot, not of GPS trust). */
    if (baro->valid && !set->baro_ground_ref_set) {
        set->baro_ref_sum_hpa += baro->press_abs_hpa;
        set->baro_ref_count++;
        if (set->baro_ref_count >= set->baro_ref_target_samples) {
            set->baro_ground_ref_hpa = set->baro_ref_sum_hpa / (float)set->baro_ref_count;
            set->baro_ground_ref_set = true;
        }
    }

    if (set->last_update_ms == 0) {
        /* First sample since init/reanchor: nothing to integrate against yet. */
        set->last_update_ms = imu->last_update_ms;
        return;
    }

    float dt = (float)(imu->last_update_ms - set->last_update_ms) / 1000.0f;
    if (dt <= 0.0f) {
        return;
    }

    float an, ae, ad;
    rotate_body_to_ned(attitude->roll, attitude->pitch, attitude->yaw,
                        imu->xacc_mps2, imu->yacc_mps2, imu->zacc_mps2, &an, &ae, &ad);
    ad += GRAVITY_MPS2; /* a_ned = R*specific_force + g_ned, g_ned = [0,0,+g] */

    core_nav_kf_axis_predict(&set->north, an, dt, sigma_accel_mps2);
    core_nav_kf_axis_predict(&set->east, ae, dt, sigma_accel_mps2);
    core_nav_kf_axis_predict(&set->down, ad, dt, sigma_accel_mps2);

    if (baro->valid && set->baro_ground_ref_set) {
        float alt_rel_m = 44330.0f * (1.0f - powf(baro->press_abs_hpa / set->baro_ground_ref_hpa, 1.0f / 5.255f));
        float z = -alt_rel_m; /* NED down is positive-down */
        core_nav_kf_axis_update(&set->down, z, baro_r_variance);
    }

    set->last_update_ms = imu->last_update_ms;
}

void core_nav_kf_set_get_position(const core_nav_kf_set_t *set,
                                   double *lat_deg_out, double *lon_deg_out, double *alt_m_out) {
    double meters_per_deg_lon = METERS_PER_DEG_LAT * cos(set->anchor_lat_deg * M_PI / 180.0);

    *lat_deg_out = set->anchor_lat_deg + (double)set->north.p / METERS_PER_DEG_LAT;
    *lon_deg_out = set->anchor_lon_deg + (double)set->east.p / meters_per_deg_lon;
    *alt_m_out = set->anchor_alt_m - (double)set->down.p;
}

bool core_nav_kf_set_should_reanchor(const core_nav_kf_set_t *set, uint64_t now_ms) {
    if (!set->anchored) {
        return true;
    }
    return (now_ms - set->anchor_time_ms) >= set->reanchor_interval_ms;
}
