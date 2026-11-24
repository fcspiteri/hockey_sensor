#ifndef HKS_ATTITUDE_H
#define HKS_ATTITUDE_H

#include "hks_contract.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HKS_HIGHG_HANDOFF_THRESH_G    14.5f
#define HKS_HIGHG_FALLBACK_THRESH_G   12.0f
#define HKS_HIGHG_FALLBACK_HYST_COUNT 4u     /* 20 ms at 200 Hz */
#define HKS_IMPACT_THRESH_G           12.0f
#define HKS_IMPACT_PEAK_HOLD_US       250000ULL

#define HKS_SHIFT_WIN_SAMPLES         200u   /* 1.0 s at 200 Hz */
#define HKS_SHIFT_STEP_SAMPLES        50u    /* 0.25 s step */
#define HKS_SHIFT_ON_CONSEC_STEPS     8u     /* 2.0 s sustained skating */
#define HKS_SHIFT_OFF_CONSEC_STEPS    20u    /* 5.0 s sustained stillness */

typedef enum {
    HKS_SHIFT_STATE_BENCH  = 0,
    HKS_SHIFT_STATE_ON_ICE = 1
} hks_shift_hint_state_t;

typedef struct {
    /* Active calibration (0x06) */
    hks_cal_block_t cal;

    /* Hamilton scalar-first Body->World quaternion q_WB = [qw, qx, qy, qz] */
    float q_wb[4];
    float kp;
    float ki;
    float integral_err_b[3];

    /* Dual-IMU saturation handoff state */
    bool     high_g_active;
    uint32_t below_fallback_count;
    bool     has_high_g_sample;
    float    last_high_g_b[3];

    /* Latest kinematics */
    float f_b[3];       /* Specific force in Body frame B (m/s^2) */
    float omega_b[3];   /* Angular velocity in Body frame B (rad/s) */
    float a_lin_w[3];   /* Gravity-removed linear acceleration in World frame W (m/s^2) */

    /* Online impact detector state (250 ms peak hold) */
    bool     impact_active;
    uint64_t impact_start_us;
    uint64_t impact_peak_us;
    float    impact_peak_mps2;

    /* Online shift-hint sliding window state */
    hks_shift_hint_state_t shift_state;
    float    win_a_sq[HKS_SHIFT_WIN_SAMPLES];
    float    win_w_sq[HKS_SHIFT_WIN_SAMPLES];
    float    step_a_sq_sum;
    size_t   win_idx;
    size_t   win_count;
    size_t   step_counter;
    uint32_t on_ice_consec;
    uint32_t bench_consec;
} hks_attitude_t;

typedef struct {
    bool              emit_impact;
    hks_event_marker_t impact_event;
    bool              emit_shift_hint;
    hks_event_marker_t shift_event;
    uint8_t           frame_flags;
} hks_attitude_step_out_t;

/* Canonical R_BS = [[0,0,1],[0,-1,0],[1,0,0]] (det=+1): v_B = R_BS * v_S */
void  hks_transform_sensor_to_body(const float r_bs[9], const float v_s[3], float v_b_out[3]);
void  hks_canonical_r_bs_apply(const float v_s[3], float v_b_out[3]);
float hks_mat3_det(const float m[9]);

/* Quaternion / Rotation helpers (Hamilton scalar-first q_WB = [qw, qx, qy, qz]) */
void  hks_quat_normalize(float q[4]);
void  hks_quat_to_rotmat(const float q[4], float r_out[9]);
void  hks_quat_rotate_vec(const float q[4], const float v_b[3], float v_w_out[3]);

/* Initialize attitude filter & online detectors */
void  hks_attitude_init(hks_attitude_t *att, const hks_cal_block_t *cal_opt);

/* Provide latest ADXL372 sample (right-aligned >> 4 int16_t) in sensor frame S */
void  hks_attitude_push_high_g(hks_attitude_t *att, const hks_high_g_sample_t *hg_sample);

/* Step filter at dt_s (e.g. 0.005f for 200 Hz) with primary ICM-42688-P sample */
void  hks_attitude_step(hks_attitude_t *att,
                        const hks_imu_sample_t *icm_sample,
                        const hks_high_g_sample_t *hg_sample_opt,
                        uint64_t timestamp_us,
                        float dt_s,
                        hks_attitude_step_out_t *out);

/* Flush any open impact hold window at session end */
bool  hks_attitude_flush_impact(hks_attitude_t *att, hks_event_marker_t *evt_out);

#ifdef __cplusplus
}
#endif

#endif /* HKS_ATTITUDE_H */
