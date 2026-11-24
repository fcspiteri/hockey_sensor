#include "hks_contract.h"
#include "hks_crc32.h"

size_t hks_frame_encode(uint8_t *out,
                        size_t out_cap,
                        uint8_t msg_type,
                        uint8_t flags,
                        uint16_t seq,
                        uint64_t timestamp_us,
                        const uint8_t *payload,
                        uint16_t payload_len) {
    if (out == NULL || payload_len > HKS_MAX_PAYLOAD_BYTES) {
        return 0u;
    }
    if (payload_len > 0u && payload == NULL) {
        return 0u;
    }
    const size_t total = (size_t)HKS_FRAME_OVERHEAD_BYTES + (size_t)payload_len;
    if (out_cap < total) {
        return 0u;
    }

    hks_put_u16_le(out + 0u, HKS_SYNC_MARKER);
    out[2] = msg_type;
    out[3] = flags;
    hks_put_u16_le(out + 4u, payload_len);
    hks_put_u16_le(out + 6u, seq);
    hks_put_u64_le(out + 8u, timestamp_us);
    hks_put_u32_le(out + 16u, 0u);

    if (payload_len > 0u) {
        memcpy(out + HKS_FRAME_HDR_BYTES, payload, (size_t)payload_len);
    }

    const uint32_t crc = hks_crc32(out, (size_t)HKS_FRAME_HDR_BYTES + (size_t)payload_len);
    hks_put_u32_le(out + HKS_FRAME_HDR_BYTES + payload_len, crc);
    return total;
}

bool hks_frame_decode_hdr(const uint8_t *buf, size_t len, hks_frame_hdr_t *hdr) {
    if (buf == NULL || hdr == NULL || len < HKS_FRAME_HDR_BYTES) {
        return false;
    }
    hdr->sync = hks_get_u16_le(buf + 0u);
    hdr->msg_type = buf[2];
    hdr->flags = buf[3];
    hdr->payload_len = hks_get_u16_le(buf + 4u);
    hdr->seq = hks_get_u16_le(buf + 6u);
    hdr->timestamp_us = hks_get_u64_le(buf + 8u);
    hdr->reserved = hks_get_u32_le(buf + 16u);

    if (hdr->sync != HKS_SYNC_MARKER ||
        hdr->msg_type < HKS_MSG_SESSION_HEADER ||
        hdr->msg_type > HKS_MSG_SESSION_FOOTER ||
        hdr->payload_len > HKS_MAX_PAYLOAD_BYTES) {
        return false;
    }
    return true;
}

bool hks_frame_verify(const uint8_t *buf, size_t len, hks_frame_hdr_t *hdr_out) {
    hks_frame_hdr_t hdr;
    if (!hks_frame_decode_hdr(buf, len, &hdr)) {
        return false;
    }
    const size_t expected_len = (size_t)HKS_FRAME_OVERHEAD_BYTES + (size_t)hdr.payload_len;
    if (len < expected_len) {
        return false;
    }
    const uint32_t expected_crc = hks_crc32(buf, (size_t)HKS_FRAME_HDR_BYTES + (size_t)hdr.payload_len);
    const uint32_t actual_crc = hks_get_u32_le(buf + HKS_FRAME_HDR_BYTES + hdr.payload_len);
    if (expected_crc != actual_crc) {
        return false;
    }
    if (hdr_out != NULL) {
        *hdr_out = hdr;
    }
    return true;
}

size_t hks_encode_session_header(uint8_t *out, size_t out_cap, const hks_session_header_t *hdr) {
    if (out == NULL || hdr == NULL || out_cap < HKS_PAYLOAD_SESSION_HEADER_BYTES) {
        return 0u;
    }
    memcpy(out + 0u, hdr->magic, 4u);
    hks_put_u16_le(out + 4u, hdr->format_version);
    hks_put_u16_le(out + 6u, hdr->hw_rev);
    hks_put_u32_le(out + 8u, hdr->fw_version);
    hks_put_u32_le(out + 12u, hdr->cal_crc32);
    hks_put_u64_le(out + 16u, hdr->session_id);
    hks_put_u64_le(out + 24u, hdr->start_utc_ms);
    hks_put_u64_le(out + 32u, hdr->device_id);
    hks_put_u16_le(out + 40u, hdr->imu_odr_hz);
    hks_put_u16_le(out + 42u, hdr->high_g_odr_hz);
    hks_put_u16_le(out + 44u, hdr->mag_odr_hz);
    hks_put_u16_le(out + 46u, hdr->baro_odr_hz);
    hks_put_f32_le(out + 48u, hdr->accel_scale_mps2_per_lsb);
    hks_put_f32_le(out + 52u, hdr->gyro_scale_rads_per_lsb);
    hks_put_f32_le(out + 56u, hdr->high_g_scale_mps2_per_lsb);
    hks_put_f32_le(out + 60u, hdr->mag_scale_ut_per_lsb);
    out[64] = hdr->rink_preset;
    out[65] = hdr->mount_location;
    hks_put_u16_le(out + 66u, hdr->player_height_cm);
    hks_put_f32_le(out + 68u, hdr->bench_gate_x_m);
    hks_put_f32_le(out + 72u, hdr->bench_gate_y_m);
    hks_put_u32_le(out + 76u, hdr->reserved);
    return HKS_PAYLOAD_SESSION_HEADER_BYTES;
}

bool hks_decode_session_header(const uint8_t *in, size_t in_len, hks_session_header_t *hdr) {
    if (in == NULL || hdr == NULL || in_len < HKS_PAYLOAD_SESSION_HEADER_BYTES) {
        return false;
    }
    memcpy(hdr->magic, in + 0u, 4u);
    hdr->format_version = hks_get_u16_le(in + 4u);
    hdr->hw_rev = hks_get_u16_le(in + 6u);
    hdr->fw_version = hks_get_u32_le(in + 8u);
    hdr->cal_crc32 = hks_get_u32_le(in + 12u);
    hdr->session_id = hks_get_u64_le(in + 16u);
    hdr->start_utc_ms = hks_get_u64_le(in + 24u);
    hdr->device_id = hks_get_u64_le(in + 32u);
    hdr->imu_odr_hz = hks_get_u16_le(in + 40u);
    hdr->high_g_odr_hz = hks_get_u16_le(in + 42u);
    hdr->mag_odr_hz = hks_get_u16_le(in + 44u);
    hdr->baro_odr_hz = hks_get_u16_le(in + 46u);
    hdr->accel_scale_mps2_per_lsb = hks_get_f32_le(in + 48u);
    hdr->gyro_scale_rads_per_lsb = hks_get_f32_le(in + 52u);
    hdr->high_g_scale_mps2_per_lsb = hks_get_f32_le(in + 56u);
    hdr->mag_scale_ut_per_lsb = hks_get_f32_le(in + 60u);
    hdr->rink_preset = in[64];
    hdr->mount_location = in[65];
    hdr->player_height_cm = hks_get_u16_le(in + 66u);
    hdr->bench_gate_x_m = hks_get_f32_le(in + 68u);
    hdr->bench_gate_y_m = hks_get_f32_le(in + 72u);
    hdr->reserved = hks_get_u32_le(in + 76u);
    return true;
}

size_t hks_encode_imu_batch(uint8_t *out, size_t out_cap,
                            const hks_imu_batch_hdr_t *hdr,
                            const hks_imu_sample_t *samples) {
    if (out == NULL || hdr == NULL || samples == NULL ||
        hdr->sample_count == 0u || hdr->sample_count > HKS_IMU_MAX_SAMPLES) {
        return 0u;
    }
    const size_t total = (size_t)HKS_PAYLOAD_IMU_BATCH_HDR_BYTES +
                         (size_t)hdr->sample_count * (size_t)HKS_IMU_SAMPLE_BYTES;
    if (out_cap < total) {
        return 0u;
    }
    hks_put_u16_le(out + 0u, hdr->sample_count);
    hks_put_u16_le(out + 2u, hdr->odr_hz);
    hks_put_i16_le(out + 4u, hdr->temp_c_x100);
    hks_put_u16_le(out + 6u, hdr->reserved);
    for (size_t i = 0u; i < (size_t)hdr->sample_count; i++) {
        uint8_t *dst = out + HKS_PAYLOAD_IMU_BATCH_HDR_BYTES + i * HKS_IMU_SAMPLE_BYTES;
        hks_put_i16_le(dst + 0u, samples[i].ax);
        hks_put_i16_le(dst + 2u, samples[i].ay);
        hks_put_i16_le(dst + 4u, samples[i].az);
        hks_put_i16_le(dst + 6u, samples[i].gx);
        hks_put_i16_le(dst + 8u, samples[i].gy);
        hks_put_i16_le(dst + 10u, samples[i].gz);
    }
    return total;
}

bool hks_decode_imu_batch(const uint8_t *in, size_t in_len,
                          hks_imu_batch_hdr_t *hdr,
                          hks_imu_sample_t *samples_out,
                          size_t max_samples) {
    if (in == NULL || hdr == NULL || in_len < HKS_PAYLOAD_IMU_BATCH_HDR_BYTES) {
        return false;
    }
    hdr->sample_count = hks_get_u16_le(in + 0u);
    hdr->odr_hz = hks_get_u16_le(in + 2u);
    hdr->temp_c_x100 = hks_get_i16_le(in + 4u);
    hdr->reserved = hks_get_u16_le(in + 6u);
    if (hdr->sample_count == 0u || hdr->sample_count > HKS_IMU_MAX_SAMPLES) {
        return false;
    }
    const size_t expected = (size_t)HKS_PAYLOAD_IMU_BATCH_HDR_BYTES +
                            (size_t)hdr->sample_count * (size_t)HKS_IMU_SAMPLE_BYTES;
    if (in_len < expected) {
        return false;
    }
    if (samples_out != NULL) {
        if (max_samples < (size_t)hdr->sample_count) {
            return false;
        }
        for (size_t i = 0u; i < (size_t)hdr->sample_count; i++) {
            const uint8_t *src = in + HKS_PAYLOAD_IMU_BATCH_HDR_BYTES + i * HKS_IMU_SAMPLE_BYTES;
            samples_out[i].ax = hks_get_i16_le(src + 0u);
            samples_out[i].ay = hks_get_i16_le(src + 2u);
            samples_out[i].az = hks_get_i16_le(src + 4u);
            samples_out[i].gx = hks_get_i16_le(src + 6u);
            samples_out[i].gy = hks_get_i16_le(src + 8u);
            samples_out[i].gz = hks_get_i16_le(src + 10u);
        }
    }
    return true;
}

size_t hks_encode_high_g_batch(uint8_t *out, size_t out_cap,
                               const hks_high_g_batch_hdr_t *hdr,
                               const hks_high_g_sample_t *samples) {
    if (out == NULL || hdr == NULL || samples == NULL ||
        hdr->sample_count == 0u || hdr->sample_count > HKS_HIGH_G_MAX_SAMPLES) {
        return 0u;
    }
    const size_t total = (size_t)HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES +
                         (size_t)hdr->sample_count * (size_t)HKS_HIGH_G_SAMPLE_BYTES;
    if (out_cap < total) {
        return 0u;
    }
    hks_put_u16_le(out + 0u, hdr->sample_count);
    hks_put_u16_le(out + 2u, hdr->odr_hz);
    out[4] = hdr->trigger_mode;
    out[5] = hdr->reserved0;
    hks_put_u16_le(out + 6u, hdr->reserved1);
    for (size_t i = 0u; i < (size_t)hdr->sample_count; i++) {
        uint8_t *dst = out + HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES + i * HKS_HIGH_G_SAMPLE_BYTES;
        hks_put_i16_le(dst + 0u, samples[i].ax);
        hks_put_i16_le(dst + 2u, samples[i].ay);
        hks_put_i16_le(dst + 4u, samples[i].az);
    }
    return total;
}

bool hks_decode_high_g_batch(const uint8_t *in, size_t in_len,
                             hks_high_g_batch_hdr_t *hdr,
                             hks_high_g_sample_t *samples_out,
                             size_t max_samples) {
    if (in == NULL || hdr == NULL || in_len < HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES) {
        return false;
    }
    hdr->sample_count = hks_get_u16_le(in + 0u);
    hdr->odr_hz = hks_get_u16_le(in + 2u);
    hdr->trigger_mode = in[4];
    hdr->reserved0 = in[5];
    hdr->reserved1 = hks_get_u16_le(in + 6u);
    if (hdr->sample_count == 0u || hdr->sample_count > HKS_HIGH_G_MAX_SAMPLES) {
        return false;
    }
    const size_t expected = (size_t)HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES +
                            (size_t)hdr->sample_count * (size_t)HKS_HIGH_G_SAMPLE_BYTES;
    if (in_len < expected) {
        return false;
    }
    if (samples_out != NULL) {
        if (max_samples < (size_t)hdr->sample_count) {
            return false;
        }
        for (size_t i = 0u; i < (size_t)hdr->sample_count; i++) {
            const uint8_t *src = in + HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES + i * HKS_HIGH_G_SAMPLE_BYTES;
            samples_out[i].ax = hks_get_i16_le(src + 0u);
            samples_out[i].ay = hks_get_i16_le(src + 2u);
            samples_out[i].az = hks_get_i16_le(src + 4u);
        }
    }
    return true;
}

size_t hks_encode_aux_env(uint8_t *out, size_t out_cap, const hks_aux_env_t *aux) {
    if (out == NULL || aux == NULL || out_cap < HKS_PAYLOAD_AUX_ENV_BYTES) {
        return 0u;
    }
    hks_put_i32_le(out + 0u, aux->mx_raw);
    hks_put_i32_le(out + 4u, aux->my_raw);
    hks_put_i32_le(out + 8u, aux->mz_raw);
    hks_put_u32_le(out + 12u, aux->pressure_pa_x64);
    hks_put_i16_le(out + 16u, aux->baro_temp_c_x100);
    hks_put_u16_le(out + 18u, aux->vbatt_mv);
    hks_put_i16_le(out + 20u, aux->batt_temp_c_x100);
    hks_put_u16_le(out + 22u, aux->soc_pct_x10);
    hks_put_u16_le(out + 24u, aux->pmic_status);
    hks_put_u16_le(out + 26u, aux->validity_flags);
    return HKS_PAYLOAD_AUX_ENV_BYTES;
}

bool hks_decode_aux_env(const uint8_t *in, size_t in_len, hks_aux_env_t *aux) {
    if (in == NULL || aux == NULL || in_len < HKS_PAYLOAD_AUX_ENV_BYTES) {
        return false;
    }
    aux->mx_raw = hks_get_i32_le(in + 0u);
    aux->my_raw = hks_get_i32_le(in + 4u);
    aux->mz_raw = hks_get_i32_le(in + 8u);
    aux->pressure_pa_x64 = hks_get_u32_le(in + 12u);
    aux->baro_temp_c_x100 = hks_get_i16_le(in + 16u);
    aux->vbatt_mv = hks_get_u16_le(in + 18u);
    aux->batt_temp_c_x100 = hks_get_i16_le(in + 20u);
    aux->soc_pct_x10 = hks_get_u16_le(in + 22u);
    aux->pmic_status = hks_get_u16_le(in + 24u);
    aux->validity_flags = hks_get_u16_le(in + 26u);
    return true;
}

size_t hks_encode_event_marker(uint8_t *out, size_t out_cap, const hks_event_marker_t *evt) {
    if (out == NULL || evt == NULL || out_cap < HKS_PAYLOAD_EVENT_MARKER_BYTES) {
        return 0u;
    }
    hks_put_u16_le(out + 0u, evt->event_code);
    hks_put_u16_le(out + 2u, evt->severity);
    hks_put_f32_le(out + 4u, evt->value_f32);
    hks_put_u64_le(out + 8u, evt->detail_u64);
    return HKS_PAYLOAD_EVENT_MARKER_BYTES;
}

bool hks_decode_event_marker(const uint8_t *in, size_t in_len, hks_event_marker_t *evt) {
    if (in == NULL || evt == NULL || in_len < HKS_PAYLOAD_EVENT_MARKER_BYTES) {
        return false;
    }
    evt->event_code = hks_get_u16_le(in + 0u);
    evt->severity = hks_get_u16_le(in + 2u);
    evt->value_f32 = hks_get_f32_le(in + 4u);
    evt->detail_u64 = hks_get_u64_le(in + 8u);
    return true;
}

static void put_f32_arr(uint8_t *dst, const float *arr, size_t count) {
    for (size_t i = 0u; i < count; i++) {
        hks_put_f32_le(dst + i * 4u, arr[i]);
    }
}

static void get_f32_arr(const uint8_t *src, float *arr, size_t count) {
    for (size_t i = 0u; i < count; i++) {
        arr[i] = hks_get_f32_le(src + i * 4u);
    }
}

size_t hks_encode_cal_block(uint8_t *out, size_t out_cap, const hks_cal_block_t *cal, bool recompute_crc) {
    if (out == NULL || cal == NULL || out_cap < HKS_PAYLOAD_CAL_BLOCK_BYTES) {
        return 0u;
    }
    hks_put_u16_le(out + 0u, cal->cal_version);
    hks_put_u16_le(out + 2u, cal->flags);
    hks_put_u64_le(out + 4u, cal->cal_utc_ms);
    put_f32_arr(out + 12u, cal->sensor_to_body_rotation_matrix, 9u);
    put_f32_arr(out + 48u, cal->accel_misalign_scale, 9u);
    put_f32_arr(out + 84u, cal->accel_bias_mps2, 3u);
    put_f32_arr(out + 96u, cal->gyro_misalign_scale, 9u);
    put_f32_arr(out + 132u, cal->gyro_bias_rads, 3u);
    put_f32_arr(out + 144u, cal->high_g_to_body_rotation_matrix, 9u);
    put_f32_arr(out + 180u, cal->high_g_misalign_scale, 9u);
    put_f32_arr(out + 216u, cal->high_g_bias_mps2, 3u);
    put_f32_arr(out + 228u, cal->mag_hard_iron_ut, 3u);
    put_f32_arr(out + 240u, cal->mag_soft_iron_matrix, 9u);
    hks_put_f32_le(out + 276u, cal->baseline_mag_norm_ut);
    hks_put_f32_le(out + 280u, cal->baseline_mag_dip_deg);

    const uint32_t crc = recompute_crc ? hks_crc32(out, HKS_CAL_BLOCK_CRC_BYTES) : cal->cal_crc32;
    hks_put_u32_le(out + 284u, crc);
    return HKS_PAYLOAD_CAL_BLOCK_BYTES;
}

bool hks_decode_cal_block(const uint8_t *in, size_t in_len, hks_cal_block_t *cal, bool verify_crc) {
    if (in == NULL || cal == NULL || in_len < HKS_PAYLOAD_CAL_BLOCK_BYTES) {
        return false;
    }
    cal->cal_version = hks_get_u16_le(in + 0u);
    cal->flags = hks_get_u16_le(in + 2u);
    cal->cal_utc_ms = hks_get_u64_le(in + 4u);
    get_f32_arr(in + 12u, cal->sensor_to_body_rotation_matrix, 9u);
    get_f32_arr(in + 48u, cal->accel_misalign_scale, 9u);
    get_f32_arr(in + 84u, cal->accel_bias_mps2, 3u);
    get_f32_arr(in + 96u, cal->gyro_misalign_scale, 9u);
    get_f32_arr(in + 132u, cal->gyro_bias_rads, 3u);
    get_f32_arr(in + 144u, cal->high_g_to_body_rotation_matrix, 9u);
    get_f32_arr(in + 180u, cal->high_g_misalign_scale, 9u);
    get_f32_arr(in + 216u, cal->high_g_bias_mps2, 3u);
    get_f32_arr(in + 228u, cal->mag_hard_iron_ut, 3u);
    get_f32_arr(in + 240u, cal->mag_soft_iron_matrix, 9u);
    cal->baseline_mag_norm_ut = hks_get_f32_le(in + 276u);
    cal->baseline_mag_dip_deg = hks_get_f32_le(in + 280u);
    cal->cal_crc32 = hks_get_u32_le(in + 284u);

    if (verify_crc) {
        const uint32_t expected = hks_crc32(in, HKS_CAL_BLOCK_CRC_BYTES);
        if (expected != cal->cal_crc32) {
            return false;
        }
    }
    return true;
}

static void set_identity_3x3(float m[9]) {
    memset(m, 0, sizeof(float) * 9u);
    m[0] = 1.0f;
    m[4] = 1.0f;
    m[8] = 1.0f;
}

static void set_canonical_r_bs(float m[9]) {
    memset(m, 0, sizeof(float) * 9u);
    m[2] = 1.0f;   /* [0][2] = +1 */
    m[4] = -1.0f;  /* [1][1] = -1 */
    m[6] = 1.0f;   /* [2][0] = +1 */
}

void hks_cal_block_init_default(hks_cal_block_t *cal, uint64_t cal_utc_ms) {
    if (cal == NULL) {
        return;
    }
    memset(cal, 0, sizeof(*cal));
    cal->cal_version = 1u;
    cal->flags = 0x000Fu;
    cal->cal_utc_ms = cal_utc_ms;
    set_canonical_r_bs(cal->sensor_to_body_rotation_matrix);
    set_identity_3x3(cal->accel_misalign_scale);
    set_identity_3x3(cal->gyro_misalign_scale);
    set_canonical_r_bs(cal->high_g_to_body_rotation_matrix);
    set_identity_3x3(cal->high_g_misalign_scale);
    set_identity_3x3(cal->mag_soft_iron_matrix);
    cal->baseline_mag_norm_ut = 51.2f;
    cal->baseline_mag_dip_deg = 66.4f;

    uint8_t tmp[HKS_PAYLOAD_CAL_BLOCK_BYTES];
    (void)hks_encode_cal_block(tmp, sizeof(tmp), cal, true);
    cal->cal_crc32 = hks_get_u32_le(tmp + 284u);
}

size_t hks_encode_session_footer(uint8_t *out, size_t out_cap, const hks_session_footer_t *ftr) {
    if (out == NULL || ftr == NULL || out_cap < HKS_PAYLOAD_SESSION_FOOTER_BYTES) {
        return 0u;
    }
    hks_put_u64_le(out + 0u, ftr->end_utc_ms);
    hks_put_u32_le(out + 8u, ftr->total_imu_samples);
    hks_put_u32_le(out + 12u, ftr->dropped_imu_samples);
    hks_put_u32_le(out + 16u, ftr->total_high_g_samples);
    hks_put_u32_le(out + 20u, ftr->dropped_high_g_samples);
    hks_put_u32_le(out + 24u, ftr->total_aux_samples);
    hks_put_u32_le(out + 28u, ftr->end_reason);
    hks_put_u64_le(out + 32u, ftr->total_bytes);
    memcpy(out + 40u, ftr->pre_footer_sha256, 32u);
    return HKS_PAYLOAD_SESSION_FOOTER_BYTES;
}

bool hks_decode_session_footer(const uint8_t *in, size_t in_len, hks_session_footer_t *ftr) {
    if (in == NULL || ftr == NULL || in_len < HKS_PAYLOAD_SESSION_FOOTER_BYTES) {
        return false;
    }
    ftr->end_utc_ms = hks_get_u64_le(in + 0u);
    ftr->total_imu_samples = hks_get_u32_le(in + 8u);
    ftr->dropped_imu_samples = hks_get_u32_le(in + 12u);
    ftr->total_high_g_samples = hks_get_u32_le(in + 16u);
    ftr->dropped_high_g_samples = hks_get_u32_le(in + 20u);
    ftr->total_aux_samples = hks_get_u32_le(in + 24u);
    ftr->end_reason = hks_get_u32_le(in + 28u);
    ftr->total_bytes = hks_get_u64_le(in + 32u);
    memcpy(ftr->pre_footer_sha256, in + 40u, 32u);
    return true;
}

size_t hks_encode_list_entry(uint8_t *out, size_t out_cap, const hks_session_list_entry_t *entry) {
    if (out == NULL || entry == NULL || out_cap < HKS_LIST_SESSIONS_ENTRY_BYTES) {
        return 0u;
    }
    hks_put_u64_le(out + 0u, entry->session_id);
    hks_put_u64_le(out + 8u, entry->start_utc_ms);
    hks_put_u32_le(out + 16u, entry->duration_ms);
    hks_put_u32_le(out + 20u, entry->byte_size);
    hks_put_u32_le(out + 24u, entry->sample_count);
    hks_put_u32_le(out + 28u, entry->flags);
    return HKS_LIST_SESSIONS_ENTRY_BYTES;
}

size_t hks_encode_device_status(uint8_t *out, size_t out_cap, const hks_device_status_t *st) {
    if (out == NULL || st == NULL || out_cap < HKS_GET_DEVICE_STATUS_RSP_BYTES) {
        return 0u;
    }
    hks_put_u32_le(out + 0u, st->fw_version);
    hks_put_u16_le(out + 4u, st->hw_rev);
    hks_put_u16_le(out + 6u, st->vbatt_mv);
    hks_put_u16_le(out + 8u, st->soc_pct_x10);
    out[10] = st->recording_state;
    out[11] = st->pmic_status;
    hks_put_u32_le(out + 12u, st->flash_free_kb);
    hks_put_u64_le(out + 16u, st->utc_ms);
    return HKS_GET_DEVICE_STATUS_RSP_BYTES;
}
