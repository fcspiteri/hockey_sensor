#include "hks_attitude.h"

#include <math.h>
#include <string.h>

static void mat3_mul_vec(const float m[9], const float v[3], float out[3]) {
    const float x = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    const float y = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    const float z = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static float vec3_norm(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void hks_transform_sensor_to_body(const float r_bs[9], const float v_s[3], float v_b_out[3]) {
    mat3_mul_vec(r_bs, v_s, v_b_out);
}

void hks_canonical_r_bs_apply(const float v_s[3], float v_b_out[3]) {
    const float x = v_s[2];
    const float y = -v_s[1];
    const float z = v_s[0];
    v_b_out[0] = x;
    v_b_out[1] = y;
    v_b_out[2] = z;
}

float hks_mat3_det(const float m[9]) {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

static bool f32_is_finite(float x) {
    uint32_t bits = 0u;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

void hks_quat_normalize(float q[4]) {
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (f32_is_finite(q[0]) && f32_is_finite(q[1]) &&
        f32_is_finite(q[2]) && f32_is_finite(q[3]) &&
        f32_is_finite(n) && n > 1e-12f) {
        const float inv = 1.0f / n;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    } else {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}

void hks_quat_to_rotmat(const float q[4], float r_out[9]) {
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    r_out[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    r_out[1] = 2.0f * (qx * qy - qw * qz);
    r_out[2] = 2.0f * (qx * qz + qw * qy);

    r_out[3] = 2.0f * (qx * qy + qw * qz);
    r_out[4] = 1.0f - 2.0f * (qx * qx + qz * qz);
    r_out[5] = 2.0f * (qy * qz - qw * qx);

    r_out[6] = 2.0f * (qx * qz - qw * qy);
    r_out[7] = 2.0f * (qy * qz + qw * qx);
    r_out[8] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

void hks_quat_rotate_vec(const float q[4], const float v_b[3], float v_w_out[3]) {
    float r[9];
    hks_quat_to_rotmat(q, r);
    mat3_mul_vec(r, v_b, v_w_out);
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
    const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void hks_attitude_init(hks_attitude_t *att, const hks_cal_block_t *cal_opt) {
    if (att == NULL) {
        return;
    }
    memset(att, 0, sizeof(*att));
    if (cal_opt != NULL) {
        att->cal = *cal_opt;
    } else {
        hks_cal_block_init_default(&att->cal, 0u);
    }
    att->q_wb[0] = 1.0f;
    att->q_wb[1] = 0.0f;
    att->q_wb[2] = 0.0f;
    att->q_wb[3] = 0.0f;
    att->kp = 1.0f;
    att->ki = 0.02f;
    att->shift_state = HKS_SHIFT_STATE_BENCH;
}

static void calibrate_high_g_to_body(const hks_cal_block_t *cal,
                                     const hks_high_g_sample_t *hg,
                                     float out_b[3]) {
    const float raw[3] = {
        (float)hg->ax * HKS_HIGH_G_SCALE_MPS2_PER_LSB,
        (float)hg->ay * HKS_HIGH_G_SCALE_MPS2_PER_LSB,
        (float)hg->az * HKS_HIGH_G_SCALE_MPS2_PER_LSB
    };
    float cal_s[3];
    mat3_mul_vec(cal->high_g_misalign_scale, raw, cal_s);
    cal_s[0] -= cal->high_g_bias_mps2[0];
    cal_s[1] -= cal->high_g_bias_mps2[1];
    cal_s[2] -= cal->high_g_bias_mps2[2];
    hks_transform_sensor_to_body(cal->high_g_to_body_rotation_matrix, cal_s, out_b);
}

void hks_attitude_push_high_g(hks_attitude_t *att, const hks_high_g_sample_t *hg_sample) {
    if (att == NULL || hg_sample == NULL) {
        return;
    }
    calibrate_high_g_to_body(&att->cal, hg_sample, att->last_high_g_b);
    att->has_high_g_sample = true;
}

static uint16_t impact_severity_from_mps2(float mps2) {
    const float g = mps2 / HKS_GRAVITY_MPS2_F;
    if (g >= 50.0f) {
        return HKS_SEV_CRITICAL;
    }
    if (g >= 25.0f) {
        return HKS_SEV_WARNING;
    }
    return HKS_SEV_NOTICE;
}

void hks_attitude_step(hks_attitude_t *att,
                       const hks_imu_sample_t *icm_sample,
                       const hks_high_g_sample_t *hg_sample_opt,
                       uint64_t timestamp_us,
                       float dt_s,
                       hks_attitude_step_out_t *out) {
    if (att == NULL || icm_sample == NULL) {
        return;
    }

    hks_attitude_step_out_t local_out;
    memset(&local_out, 0, sizeof(local_out));

    if (hg_sample_opt != NULL) {
        hks_attitude_push_high_g(att, hg_sample_opt);
    }

    /* 1. Calibrate primary ICM-42688-P accel & gyro in Sensor frame S */
    const float a_raw_s[3] = {
        (float)icm_sample->ax * HKS_ACCEL_SCALE_MPS2_PER_LSB,
        (float)icm_sample->ay * HKS_ACCEL_SCALE_MPS2_PER_LSB,
        (float)icm_sample->az * HKS_ACCEL_SCALE_MPS2_PER_LSB
    };
    const float g_raw_s[3] = {
        (float)icm_sample->gx * HKS_GYRO_SCALE_RADS_PER_LSB,
        (float)icm_sample->gy * HKS_GYRO_SCALE_RADS_PER_LSB,
        (float)icm_sample->gz * HKS_GYRO_SCALE_RADS_PER_LSB
    };

    float a_cal_s[3];
    mat3_mul_vec(att->cal.accel_misalign_scale, a_raw_s, a_cal_s);
    a_cal_s[0] -= att->cal.accel_bias_mps2[0];
    a_cal_s[1] -= att->cal.accel_bias_mps2[1];
    a_cal_s[2] -= att->cal.accel_bias_mps2[2];

    float g_cal_s[3];
    mat3_mul_vec(att->cal.gyro_misalign_scale, g_raw_s, g_cal_s);
    g_cal_s[0] -= att->cal.gyro_bias_rads[0];
    g_cal_s[1] -= att->cal.gyro_bias_rads[1];
    g_cal_s[2] -= att->cal.gyro_bias_rads[2];

    float a_icm_b[3];
    hks_transform_sensor_to_body(att->cal.sensor_to_body_rotation_matrix, a_cal_s, a_icm_b);
    hks_transform_sensor_to_body(att->cal.sensor_to_body_rotation_matrix, g_cal_s, att->omega_b);

    /* 2. Dual-IMU > 14.5 g saturation handoff with hysteresis fallback */
    float max_axis_mps2 = fabsf(a_cal_s[0]);
    if (fabsf(a_cal_s[1]) > max_axis_mps2) {
        max_axis_mps2 = fabsf(a_cal_s[1]);
    }
    if (fabsf(a_cal_s[2]) > max_axis_mps2) {
        max_axis_mps2 = fabsf(a_cal_s[2]);
    }
    const float a_icm_inf_g = max_axis_mps2 / HKS_GRAVITY_MPS2_F;
    const bool raw_accel_sat = (icm_sample->ax <= -32767 || icm_sample->ax >= 32767 ||
                                icm_sample->ay <= -32767 || icm_sample->ay >= 32767 ||
                                icm_sample->az <= -32767 || icm_sample->az >= 32767);
    const bool raw_gyro_sat = (icm_sample->gx <= -32767 || icm_sample->gx >= 32767 ||
                               icm_sample->gy <= -32767 || icm_sample->gy >= 32767 ||
                               icm_sample->gz <= -32767 || icm_sample->gz >= 32767);

    if (a_icm_inf_g > HKS_HIGHG_HANDOFF_THRESH_G || raw_accel_sat) {
        att->high_g_active = true;
        att->below_fallback_count = 0u;
    } else if (att->high_g_active) {
        if (a_icm_inf_g < HKS_HIGHG_FALLBACK_THRESH_G) {
            att->below_fallback_count++;
            if (att->below_fallback_count >= HKS_HIGHG_FALLBACK_HYST_COUNT) {
                att->high_g_active = false;
                att->below_fallback_count = 0u;
                att->has_high_g_sample = false;
            }
        } else {
            att->below_fallback_count = 0u;
        }
    }

    if (att->high_g_active && att->has_high_g_sample) {
        att->f_b[0] = att->last_high_g_b[0];
        att->f_b[1] = att->last_high_g_b[1];
        att->f_b[2] = att->last_high_g_b[2];
    } else {
        att->f_b[0] = a_icm_b[0];
        att->f_b[1] = a_icm_b[1];
        att->f_b[2] = a_icm_b[2];
    }

    if (att->high_g_active) {
        local_out.frame_flags |= (uint8_t)(HKS_FLAG_ACCEL_SAT | HKS_FLAG_HIGH_G_ACTIVE);
    }
    if (raw_gyro_sat) {
        local_out.frame_flags |= (uint8_t)HKS_FLAG_GYRO_SAT;
    }

    /* 3. Mahony quaternion attitude filter (gravity feedback gated during dynamics/impacts) */
    const float f_norm = vec3_norm(att->f_b);
    float omega_corr[3] = { att->omega_b[0], att->omega_b[1], att->omega_b[2] };

    const bool gravity_gate_ok = (!att->high_g_active) &&
                                 (f_norm > 1e-3f) &&
                                 (fabsf(f_norm - HKS_GRAVITY_MPS2_F) <= (0.25f * HKS_GRAVITY_MPS2_F));

    if (gravity_gate_ok && dt_s > 0.0f) {
        const float inv_fn = 1.0f / f_norm;
        const float fn[3] = { att->f_b[0] * inv_fn, att->f_b[1] * inv_fn, att->f_b[2] * inv_fn };
        const float qw = att->q_wb[0];
        const float qx = att->q_wb[1];
        const float qy = att->q_wb[2];
        const float qz = att->q_wb[3];
        /* Expected body-up unit vector = R(q_WB)^T * [0, 0, +1] */
        const float v_exp[3] = {
            2.0f * (qx * qz - qw * qy),
            2.0f * (qy * qz + qw * qx),
            1.0f - 2.0f * (qx * qx + qy * qy)
        };
        float err[3];
        vec3_cross(fn, v_exp, err);
        const float err_norm = vec3_norm(err);

        for (int i = 0; i < 3; i++) {
            if (err_norm <= 0.15f) {
                att->integral_err_b[i] = clampf(att->integral_err_b[i] + att->ki * err[i] * dt_s,
                                                -0.05f, 0.05f);
            }
            omega_corr[i] += att->kp * err[i] + att->integral_err_b[i];
        }
    }

    if (dt_s > 0.0f) {
        const float theta[3] = {
            omega_corr[0] * dt_s,
            omega_corr[1] * dt_s,
            omega_corr[2] * dt_s
        };
        const float theta_norm = vec3_norm(theta);
        float half_sinc = 0.0f;
        if (theta_norm < 1e-4f) {
            half_sinc = 0.5f - (theta_norm * theta_norm) / 48.0f;
        } else {
            half_sinc = sinf(0.5f * theta_norm) / theta_norm;
        }
        const float dq[4] = {
            cosf(0.5f * theta_norm),
            theta[0] * half_sinc,
            theta[1] * half_sinc,
            theta[2] * half_sinc
        };
        float q_next[4];
        quat_mul(att->q_wb, dq, q_next);
        hks_quat_normalize(q_next);
        memcpy(att->q_wb, q_next, sizeof(att->q_wb));
    }

    /* 4. World-frame gravity removal: a_lin^W = R(q_WB) * f^B + [0, 0, -9.80665] */
    float f_w[3];
    hks_quat_rotate_vec(att->q_wb, att->f_b, f_w);
    att->a_lin_w[0] = f_w[0];
    att->a_lin_w[1] = f_w[1];
    att->a_lin_w[2] = f_w[2] - HKS_GRAVITY_MPS2_F;

    /* 5. Online IMPACT_SPIKE (> 12 g) detector with 250 ms peak-hold */
    const float impact_thresh_mps2 = HKS_IMPACT_THRESH_G * HKS_GRAVITY_MPS2_F;
    if (att->impact_active &&
        timestamp_us >= att->impact_start_us &&
        (timestamp_us - att->impact_start_us) >= HKS_IMPACT_PEAK_HOLD_US) {
        local_out.emit_impact = true;
        local_out.impact_event.event_code = HKS_EVT_IMPACT_SPIKE;
        local_out.impact_event.severity = impact_severity_from_mps2(att->impact_peak_mps2);
        local_out.impact_event.value_f32 = att->impact_peak_mps2;
        local_out.impact_event.detail_u64 = att->impact_peak_us;
        att->impact_active = false;
    }

    if (f_norm > impact_thresh_mps2) {
        if (!att->impact_active) {
            att->impact_active = true;
            att->impact_start_us = timestamp_us;
            att->impact_peak_us = timestamp_us;
            att->impact_peak_mps2 = f_norm;
        } else if (f_norm > att->impact_peak_mps2) {
            att->impact_peak_mps2 = f_norm;
            att->impact_peak_us = timestamp_us;
        }
    }

    /* 6. Online SHIFT_HINT_ON (0.25 s non-overlapping steps) / SHIFT_HINT_OFF (1.0 s window) */
    const float a_lin_sq = att->a_lin_w[0] * att->a_lin_w[0] +
                           att->a_lin_w[1] * att->a_lin_w[1] +
                           att->a_lin_w[2] * att->a_lin_w[2];
    const float w_sq = att->omega_b[0] * att->omega_b[0] +
                       att->omega_b[1] * att->omega_b[1] +
                       att->omega_b[2] * att->omega_b[2];

    att->win_a_sq[att->win_idx] = a_lin_sq;
    att->win_w_sq[att->win_idx] = w_sq;
    att->win_idx = (att->win_idx + 1u) % HKS_SHIFT_WIN_SAMPLES;
    if (att->win_count < HKS_SHIFT_WIN_SAMPLES) {
        att->win_count++;
    }
    att->step_a_sq_sum += a_lin_sq;
    att->step_counter++;

    if (att->step_counter >= HKS_SHIFT_STEP_SAMPLES) {
        att->step_counter = 0u;
        const float step_a_dyn_rms = sqrtf(att->step_a_sq_sum / (float)HKS_SHIFT_STEP_SAMPLES);
        att->step_a_sq_sum = 0.0f;

        if (att->shift_state == HKS_SHIFT_STATE_BENCH) {
            if (step_a_dyn_rms > (0.15f * HKS_GRAVITY_MPS2_F)) {
                att->on_ice_consec++;
                if (att->on_ice_consec >= HKS_SHIFT_ON_CONSEC_STEPS) {
                    att->shift_state = HKS_SHIFT_STATE_ON_ICE;
                    att->on_ice_consec = 0u;
                    att->bench_consec = 0u;
                    local_out.emit_shift_hint = true;
                    local_out.shift_event.event_code = HKS_EVT_SHIFT_HINT_ON;
                    local_out.shift_event.severity = HKS_SEV_INFO;
                    local_out.shift_event.value_f32 = step_a_dyn_rms;
                    local_out.shift_event.detail_u64 = timestamp_us;
                }
            } else {
                att->on_ice_consec = 0u;
            }
        } else if (att->win_count == HKS_SHIFT_WIN_SAMPLES) {
            float sum_a = 0.0f;
            float sum_w = 0.0f;
            for (size_t i = 0u; i < HKS_SHIFT_WIN_SAMPLES; i++) {
                sum_a += att->win_a_sq[i];
                sum_w += att->win_w_sq[i];
            }
            const float a_dyn_rms = sqrtf(sum_a / (float)HKS_SHIFT_WIN_SAMPLES);
            const float omega_rms = sqrtf(sum_w / (float)HKS_SHIFT_WIN_SAMPLES);

            float r[9];
            hks_quat_to_rotmat(att->q_wb, r);
            const float theta_pitch_rad = acosf(clampf(r[8], -1.0f, 1.0f));
            const float deg_to_rad = 3.14159265358979323846f / 180.0f;

            if (theta_pitch_rad < (8.0f * deg_to_rad) &&
                a_dyn_rms < (0.05f * HKS_GRAVITY_MPS2_F) &&
                omega_rms < (8.0f * deg_to_rad)) {
                att->bench_consec++;
                if (att->bench_consec >= HKS_SHIFT_OFF_CONSEC_STEPS) {
                    att->shift_state = HKS_SHIFT_STATE_BENCH;
                    att->bench_consec = 0u;
                    att->on_ice_consec = 0u;
                    local_out.emit_shift_hint = true;
                    local_out.shift_event.event_code = HKS_EVT_SHIFT_HINT_OFF;
                    local_out.shift_event.severity = HKS_SEV_INFO;
                    local_out.shift_event.value_f32 = a_dyn_rms;
                    local_out.shift_event.detail_u64 = timestamp_us;
                }
            } else {
                att->bench_consec = 0u;
            }
        }
    }

    if (out != NULL) {
        *out = local_out;
    }
}

bool hks_attitude_flush_impact(hks_attitude_t *att, hks_event_marker_t *evt_out) {
    if (att == NULL || !att->impact_active) {
        return false;
    }
    if (evt_out != NULL) {
        evt_out->event_code = HKS_EVT_IMPACT_SPIKE;
        evt_out->severity = impact_severity_from_mps2(att->impact_peak_mps2);
        evt_out->value_f32 = att->impact_peak_mps2;
        evt_out->detail_u64 = att->impact_peak_us;
    }
    att->impact_active = false;
    return true;
}
