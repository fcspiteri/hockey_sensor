#define _POSIX_C_SOURCE 200809L
#include "hks_attitude.h"
#include "hks_contract.h"
#include "hks_crc32.h"
#include "hks_flash_log.h"
#include "hks_hal.h"
#include "hks_protocol.h"
#include "hks_sha256.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int16_t clamp_i16(float x) {
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lrintf(x);
}

static int16_t clamp_adxl_i12(float x) {
    if (x > 2047.0f) return 2047;
    if (x < -2048.0f) return -2048;
    return (int16_t)lrintf(x);
}

static uint16_t encode_adxl_lj12(int16_t val_i12) {
    uint16_t u = 0u;
    memcpy(&u, &val_i12, sizeof(u));
    return (uint16_t)(u << 4);
}

static int run_usb_selftest(hks_protocol_t *proto, uint64_t session_id,
                            const uint8_t *file_buf, uint32_t file_len) {
    uint8_t cmd[64];
    uint8_t rsp[HKS_PROTO_MAX_TX_FRAME];

    /* 1. SET_UTC_TIME */
    uint8_t utc_pl[8];
    hks_put_u64_le(utc_pl, 1763852500000ULL);
    size_t clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_SET_UTC_TIME, 1u, utc_pl, 8u);
    hks_hal_mock_usb_push_rx(cmd, clen);
    (void)hks_protocol_poll_usb(proto);
    size_t rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
    if (rlen < HKS_PROTO_RSP_OVERHEAD_BYTES || rsp[3] != HKS_STATUS_OK) {
        fprintf(stderr, "USB selftest SET_UTC_TIME failed\n");
        return -1;
    }

    /* 2. GET_DEVICE_STATUS */
    clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_GET_DEVICE_STATUS, 2u, NULL, 0u);
    hks_hal_mock_usb_push_rx(cmd, clen);
    (void)hks_protocol_poll_usb(proto);
    rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
    if (rlen != HKS_PROTO_RSP_OVERHEAD_BYTES + HKS_GET_DEVICE_STATUS_RSP_BYTES || rsp[3] != HKS_STATUS_OK) {
        fprintf(stderr, "USB selftest GET_DEVICE_STATUS failed\n");
        return -1;
    }

    /* 3. LIST_SESSIONS */
    clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_LIST_SESSIONS, 3u, NULL, 0u);
    hks_hal_mock_usb_push_rx(cmd, clen);
    (void)hks_protocol_poll_usb(proto);
    rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
    if (rlen < HKS_PROTO_RSP_OVERHEAD_BYTES + 4u || rsp[3] != HKS_STATUS_OK) {
        fprintf(stderr, "USB selftest LIST_SESSIONS failed\n");
        return -1;
    }
    const uint16_t sess_cnt = hks_get_u16_le(rsp + HKS_PROTO_RSP_HDR_BYTES);
    if (sess_cnt < 1u) {
        fprintf(stderr, "USB selftest LIST_SESSIONS count=0\n");
        return -1;
    }

    /* 4. GET_HEADER */
    uint8_t sid_pl[8];
    hks_put_u64_le(sid_pl, session_id);
    clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_GET_HEADER, 4u, sid_pl, 8u);
    hks_hal_mock_usb_push_rx(cmd, clen);
    (void)hks_protocol_poll_usb(proto);
    rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
    if (rlen != HKS_PROTO_RSP_OVERHEAD_BYTES + HKS_GET_HEADER_RSP_BYTES || rsp[3] != HKS_STATUS_OK) {
        fprintf(stderr, "USB selftest GET_HEADER failed\n");
        return -1;
    }
    if (memcmp(rsp + HKS_PROTO_RSP_HDR_BYTES, file_buf, HKS_GET_HEADER_RSP_BYTES) != 0) {
        fprintf(stderr, "USB selftest GET_HEADER payload mismatch\n");
        return -1;
    }

    /* 5. READ_CHUNK entire file and compare byte-for-byte */
    uint32_t offset = 0u;
    uint8_t seq = 5u;
    while (offset < file_len) {
        uint8_t rc_pl[14];
        const uint16_t req_len = (file_len - offset > HKS_PROTO_MAX_CHUNK_BYTES)
                                     ? (uint16_t)HKS_PROTO_MAX_CHUNK_BYTES
                                     : (uint16_t)(file_len - offset);
        hks_put_u64_le(rc_pl + 0u, session_id);
        hks_put_u32_le(rc_pl + 8u, offset);
        hks_put_u16_le(rc_pl + 12u, req_len);
        clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_READ_CHUNK, seq++, rc_pl, 14u);
        hks_hal_mock_usb_push_rx(cmd, clen);
        (void)hks_protocol_poll_usb(proto);
        rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
        if (rlen < HKS_PROTO_RSP_OVERHEAD_BYTES + HKS_READ_CHUNK_RSP_HDR_BYTES || rsp[3] != HKS_STATUS_OK) {
            fprintf(stderr, "USB selftest READ_CHUNK failed at offset %u\n", offset);
            return -1;
        }
        const uint16_t actual_len = hks_get_u16_le(rsp + HKS_PROTO_RSP_HDR_BYTES + 4u);
        if (actual_len != req_len ||
            memcmp(rsp + HKS_PROTO_RSP_HDR_BYTES + 6u, file_buf + offset, actual_len) != 0) {
            fprintf(stderr, "USB selftest READ_CHUNK mismatch at offset %u\n", offset);
            return -1;
        }
        offset += actual_len;
    }

    /* 6. ACK_VERIFY with whole-file CRC32 + SHA-256 (hash_mode == 2) */
    uint8_t ack_pl[44];
    hks_put_u64_le(ack_pl + 0u, session_id);
    hks_put_u32_le(ack_pl + 8u, hks_crc32(file_buf, file_len));
    hks_sha256(file_buf, file_len, ack_pl + 12u);
    clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), HKS_CMD_ACK_VERIFY, seq++, ack_pl, 44u);
    hks_hal_mock_usb_push_rx(cmd, clen);
    (void)hks_protocol_poll_usb(proto);
    rlen = hks_hal_mock_usb_pop_tx(rsp, sizeof(rsp));
    if (rlen != HKS_PROTO_RSP_OVERHEAD_BYTES + HKS_ACK_VERIFY_RSP_BYTES ||
        rsp[3] != HKS_STATUS_OK ||
        rsp[HKS_PROTO_RSP_HDR_BYTES] != 1u ||
        rsp[HKS_PROTO_RSP_HDR_BYTES + 1u] != 2u) {
        fprintf(stderr, "USB selftest ACK_VERIFY failed\n");
        return -1;
    }

    return 0;
}

static void print_usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s [--out <path.hks>] [--info-json <path.json>] [--usb-selftest]\n",
            argv0);
}

int main(int argc, char **argv) {
    const char *out_path = "build/sim_session.hks";
    const char *info_json_path = NULL;
    bool usb_selftest = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--info-json") == 0 && i + 1 < argc) {
            info_json_path = argv[++i];
        } else if (strcmp(argv[i], "--usb-selftest") == 0) {
            usb_selftest = true;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    hks_hal_mock_reset();
    hks_hal_mock_set_device_id(102030405060708ULL);
    hks_hal_mock_set_time_us(0ULL);

    hks_hal_pmic_info_t pmic = {
        .vbatt_mv = 3920u,
        .batt_temp_c_x100 = 1850,
        .soc_pct_x10 = 845u,
        .pmic_status = (uint8_t)(HKS_PMIC_FG_VALID)
    };
    hks_hal_mock_set_pmic(&pmic);

    static hks_flash_log_t flash_log;
    if (hks_flash_log_init(&flash_log) != 0) {
        fprintf(stderr, "hks_flash_log_init failed\n");
        return 1;
    }

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1763852400000ULL);
    if (hks_flash_log_write_factory_cal(&flash_log, &cal) != 0) {
        fprintf(stderr, "hks_flash_log_write_factory_cal failed\n");
        return 1;
    }

    hks_protocol_t proto;
    hks_protocol_init(&proto, &flash_log, HKS_FW_VERSION_DEFAULT, HKS_HW_REV_DEFAULT);
    proto.utc_synced = true;
    proto.utc_base_ms = 1763852500000ULL;
    proto.utc_sync_mono_us = 0ULL;

    hks_attitude_t att;
    hks_attitude_init(&att, &cal);

    const uint64_t session_id = 1001ULL;
    hks_session_header_t sh;
    memset(&sh, 0, sizeof(sh));
    memcpy(sh.magic, "HKS1", 4u);
    sh.format_version = HKS_FORMAT_VERSION;
    sh.hw_rev = HKS_HW_REV_DEFAULT;
    sh.fw_version = HKS_FW_VERSION_DEFAULT;
    sh.cal_crc32 = cal.cal_crc32;
    sh.session_id = session_id;
    sh.start_utc_ms = 1763852500000ULL;
    sh.device_id = hks_hal_device_id();
    sh.imu_odr_hz = 200u;
    sh.high_g_odr_hz = 3200u;
    sh.mag_odr_hz = 50u;
    sh.baro_odr_hz = 25u;
    sh.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;
    sh.rink_preset = HKS_RINK_NHL_200X85FT;
    sh.mount_location = HKS_MOUNT_STERNUM_POCKET;
    sh.player_height_cm = 183u;
    sh.bench_gate_x_m = -6.0f;
    sh.bench_gate_y_m = 12.954f;

    if (hks_flash_log_start_session(&flash_log, &sh, &cal, HKS_FLAG_UTC_SYNCED, 0ULL) != 0) {
        fprintf(stderr, "start_session failed\n");
        return 1;
    }

    hks_event_marker_t start_evt = {
        .event_code = HKS_EVT_SESSION_START,
        .severity = HKS_SEV_INFO,
        .value_f32 = 0.0f,
        .detail_u64 = sh.start_utc_ms
    };
    (void)hks_flash_log_append_event(&flash_log, HKS_FLAG_UTC_SYNCED, 0ULL, &start_evt);

    /* Simulate two shifts separated by a > 2 hour (> 71.5 min uint32 rollover) jump */
    const float dt_s = 0.005f; /* 200 Hz */
    const uint32_t samples_per_batch = 20u;
    hks_imu_sample_t imu_batch[20];
    uint32_t imu_frames_logged = 0u;
    uint32_t hg_frames_logged = 0u;
    uint32_t aux_frames_logged = 0u;
    uint32_t event_frames_logged = 1u;

    for (int phase = 0; phase < 2; phase++) {
        uint64_t t_us = (phase == 0) ? 0ULL : 7200000000ULL; /* > 2 hr */
        hks_hal_mock_set_time_us(t_us);

        /* 14 seconds per shift segment: 2s bench -> 5s skating (+ impact in phase 1) -> 7s bench */
        const uint32_t total_steps = 2800u; /* 14 s @ 200 Hz */
        uint32_t batch_idx = 0u;
        uint64_t batch_start_us = t_us;
        uint8_t batch_flags = HKS_FLAG_UTC_SYNCED;

        typedef struct {
            uint8_t kind; /* 1=AUX, 2=EVENT, 3=HG */
            uint8_t flags;
            uint64_t ts_us;
            hks_aux_env_t aux;
            hks_event_marker_t evt;
            hks_high_g_batch_hdr_t hgh;
            hks_high_g_sample_t hg_burst[32];
        } deferred_frame_t;
        deferred_frame_t def_q[32];
        size_t def_cnt = 0u;

        for (uint32_t step = 0u; step < total_steps; step++) {
            const float t_local = (float)step * dt_s;
            float ax_b = 0.0f, ay_b = 0.0f, az_b = HKS_GRAVITY_MPS2_F;
            float gx_b = 0.0f, gy_b = 0.0f, gz_b = 0.0f;
            bool inject_high_g = false;

            if (t_local >= 2.0f && t_local < 7.0f) {
                /* Active striding */
                const float w_stride = 2.0f * 3.14159265f * 1.4f;
                ax_b = 2.6f * sinf(w_stride * t_local);
                ay_b = 3.2f * cosf(w_stride * t_local);
                az_b = HKS_GRAVITY_MPS2_F + 1.2f * sinf(2.0f * w_stride * t_local);
                gz_b = 0.35f * sinf(0.5f * w_stride * t_local);
            }

            if (phase == 1 && t_local >= 4.50f && t_local < 4.55f) {
                /* > 14.5 g body check / board collision (~39 g = 382.5 m/s^2) */
                ax_b = -382.5f;
                ay_b = 45.0f;
                az_b = HKS_GRAVITY_MPS2_F;
                inject_high_g = true;
            }

            /* Convert Body B -> Sensor S via R_BS^T: S = [z_B, -y_B, x_B] */
            const float ax_s = az_b;
            const float ay_s = -ay_b;
            const float az_s = ax_b;
            const float gx_s = gz_b;
            const float gy_s = -gy_b;
            const float gz_s = gx_b;

            hks_imu_sample_t icm_s = {
                .ax = clamp_i16(ax_s / HKS_ACCEL_SCALE_MPS2_PER_LSB),
                .ay = clamp_i16(ay_s / HKS_ACCEL_SCALE_MPS2_PER_LSB),
                .az = clamp_i16(az_s / HKS_ACCEL_SCALE_MPS2_PER_LSB),
                .gx = clamp_i16(gx_s / HKS_GYRO_SCALE_RADS_PER_LSB),
                .gy = clamp_i16(gy_s / HKS_GYRO_SCALE_RADS_PER_LSB),
                .gz = clamp_i16(gz_s / HKS_GYRO_SCALE_RADS_PER_LSB)
            };

            hks_high_g_sample_t hg_s;
            hks_high_g_sample_t *hg_ptr = NULL;
            if (inject_high_g) {
                int16_t raw_lj_x = (int16_t)encode_adxl_lj12(clamp_adxl_i12(ax_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                int16_t raw_lj_y = (int16_t)encode_adxl_lj12(clamp_adxl_i12(ay_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                int16_t raw_lj_z = (int16_t)encode_adxl_lj12(clamp_adxl_i12(az_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                hks_hal_mock_push_highg_raw16(&raw_lj_x, &raw_lj_y, &raw_lj_z, 1u);
                size_t hg_got = 0u;
                (void)hks_hal_highg_read_fifo(&hg_s, 1u, &hg_got);
                if (hg_got > 0u) {
                    hg_ptr = &hg_s;
                }
            }

            hks_attitude_step_out_t step_out;
            hks_attitude_step(&att, &icm_s, hg_ptr, t_us, dt_s, &step_out);
            batch_flags |= step_out.frame_flags;

            if (batch_idx == 0u) {
                batch_start_us = t_us;
                def_cnt = 0u;
            }
            imu_batch[batch_idx++] = icm_s;

            if (step_out.emit_impact && def_cnt < 32u) {
                def_q[def_cnt].kind = 2u;
                def_q[def_cnt].flags = (uint8_t)(batch_flags | HKS_FLAG_HIGH_G_ACTIVE);
                def_q[def_cnt].ts_us = t_us;
                def_q[def_cnt].evt = step_out.impact_event;
                def_cnt++;
            }
            if (step_out.emit_shift_hint && def_cnt < 32u) {
                def_q[def_cnt].kind = 2u;
                def_q[def_cnt].flags = batch_flags;
                def_q[def_cnt].ts_us = t_us;
                def_q[def_cnt].evt = step_out.shift_event;
                def_cnt++;
            }

            /* 50 Hz AUX_ENV (every 4 primary samples) */
            if ((step % 4u) == 0u && def_cnt < 32u) {
                hks_aux_env_t aux = {
                    .mx_raw = 3200,
                    .my_raw = -1400,
                    .mz_raw = 7300,
                    .pressure_pa_x64 = 6484800u,
                    .baro_temp_c_x100 = 2140,
                    .vbatt_mv = pmic.vbatt_mv,
                    .batt_temp_c_x100 = pmic.batt_temp_c_x100,
                    .soc_pct_x10 = pmic.soc_pct_x10,
                    .pmic_status = pmic.pmic_status,
                    .validity_flags = (uint16_t)(HKS_AUX_MAG_FRESH | HKS_AUX_BARO_FRESH |
                                                 HKS_AUX_PMIC_FRESH | HKS_AUX_MAG_SET_RESET)
                };
                def_q[def_cnt].kind = 1u;
                def_q[def_cnt].flags = HKS_FLAG_UTC_SYNCED;
                def_q[def_cnt].ts_us = t_us;
                def_q[def_cnt].aux = aux;
                def_cnt++;
            }

            /* Emit a 32-sample 3200 Hz HIGH_G_BATCH burst at the phase 1 collision */
            if (phase == 1 && step == 900u && def_cnt < 32u) {
                def_q[def_cnt].kind = 3u;
                def_q[def_cnt].flags = (uint8_t)(HKS_FLAG_UTC_SYNCED | HKS_FLAG_ACCEL_SAT | HKS_FLAG_HIGH_G_ACTIVE);
                def_q[def_cnt].ts_us = t_us;
                for (size_t k = 0u; k < 32u; k++) {
                    const float env = sinf(3.14159265f * (float)k / 31.0f);
                    const float peak_ax_s = HKS_GRAVITY_MPS2_F;
                    const float peak_ay_s = -45.0f * env;
                    const float peak_az_s = -382.5f * env;
                    const uint16_t rx = encode_adxl_lj12(clamp_adxl_i12(peak_ax_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                    const uint16_t ry = encode_adxl_lj12(clamp_adxl_i12(peak_ay_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                    const uint16_t rz = encode_adxl_lj12(clamp_adxl_i12(peak_az_s / HKS_HIGH_G_SCALE_MPS2_PER_LSB));
                    def_q[def_cnt].hg_burst[k].ax = hks_adxl372_unpack_i12(rx);
                    def_q[def_cnt].hg_burst[k].ay = hks_adxl372_unpack_i12(ry);
                    def_q[def_cnt].hg_burst[k].az = hks_adxl372_unpack_i12(rz);
                }
                def_q[def_cnt].hgh.sample_count = 32u;
                def_q[def_cnt].hgh.odr_hz = 3200u;
                def_q[def_cnt].hgh.trigger_mode = HKS_HIGH_G_IMPACT_BURST;
                def_q[def_cnt].hgh.reserved0 = 0u;
                def_q[def_cnt].hgh.reserved1 = 0u;
                def_cnt++;
            }

            if (batch_idx == samples_per_batch) {
                hks_imu_batch_hdr_t bh = {
                    .sample_count = (uint16_t)samples_per_batch,
                    .odr_hz = 200u,
                    .temp_c_x100 = 2150,
                    .reserved = 0u
                };
                if (hks_flash_log_append_imu_batch(&flash_log, batch_flags, batch_start_us, &bh, imu_batch) != 0) {
                    fprintf(stderr, "append_imu_batch failed\n");
                    return 1;
                }
                imu_frames_logged++;

                for (size_t di = 0u; di < def_cnt; di++) {
                    if (def_q[di].kind == 1u) {
                        if (hks_flash_log_append_aux_env(&flash_log, def_q[di].flags, def_q[di].ts_us, &def_q[di].aux) != 0) {
                            fprintf(stderr, "append_aux_env failed\n");
                            return 1;
                        }
                        aux_frames_logged++;
                    } else if (def_q[di].kind == 2u) {
                        (void)hks_flash_log_append_event(&flash_log, def_q[di].flags, def_q[di].ts_us, &def_q[di].evt);
                        event_frames_logged++;
                    } else if (def_q[di].kind == 3u) {
                        if (hks_flash_log_append_high_g_batch(&flash_log, def_q[di].flags, def_q[di].ts_us,
                                                              &def_q[di].hgh, def_q[di].hg_burst) != 0) {
                            fprintf(stderr, "append_high_g_batch failed\n");
                            return 1;
                        }
                        hg_frames_logged++;
                    }
                }
                def_cnt = 0u;
                batch_idx = 0u;
                batch_flags = HKS_FLAG_UTC_SYNCED;
            }

            t_us += 5000ULL;
            hks_hal_mock_set_time_us(t_us);
        }
    }

    hks_event_marker_t flushed_imp;
    if (hks_attitude_flush_impact(&att, &flushed_imp)) {
        (void)hks_flash_log_append_event(&flash_log, HKS_FLAG_UTC_SYNCED, hks_hal_time_us(), &flushed_imp);
        event_frames_logged++;
    }

    hks_event_marker_t end_evt = {
        .event_code = HKS_EVT_SESSION_END,
        .severity = HKS_SEV_INFO,
        .value_f32 = 0.0f,
        .detail_u64 = hks_hal_time_us()
    };
    (void)hks_flash_log_append_event(&flash_log, HKS_FLAG_UTC_SYNCED, hks_hal_time_us(), &end_evt);
    event_frames_logged++;

    const uint64_t end_ts_us = hks_hal_time_us();
    const uint64_t end_utc_ms = sh.start_utc_ms + (end_ts_us / 1000ULL);
    if (hks_flash_log_stop_session(&flash_log, HKS_END_BUTTON_STOP, end_utc_ms, end_ts_us) != 0) {
        fprintf(stderr, "stop_session failed\n");
        return 1;
    }

    if (hks_hal_mock_get_nop_violations() != 0u || hks_hal_mock_get_bit_violations() != 0u) {
        fprintf(stderr, "NAND violation detected: nop=%u bit=%u\n",
                hks_hal_mock_get_nop_violations(), hks_hal_mock_get_bit_violations());
        return 1;
    }

    /* Read complete session back from NAND */
    hks_session_list_entry_t entries[4];
    size_t sess_cnt = 0u;
    if (hks_flash_log_list_sessions(&flash_log, entries, 4u, &sess_cnt) != HKS_STATUS_OK || sess_cnt != 1u) {
        fprintf(stderr, "list_sessions failed\n");
        return 1;
    }

    const uint32_t total_bytes = entries[0].byte_size;
    uint8_t *file_buf = (uint8_t *)malloc(total_bytes);
    if (file_buf == NULL) {
        fprintf(stderr, "malloc(%u) failed\n", total_bytes);
        return 1;
    }

    uint32_t read_off = 0u;
    while (read_off < total_bytes) {
        const uint16_t want = (total_bytes - read_off > 4096u) ? 4096u : (uint16_t)(total_bytes - read_off);
        uint16_t got = 0u;
        if (hks_flash_log_read_bytes(&flash_log, session_id, read_off, want, file_buf + read_off, &got) != HKS_STATUS_OK) {
            fprintf(stderr, "read_bytes failed at %u\n", read_off);
            free(file_buf);
            return 1;
        }
        read_off += got;
    }

    if (usb_selftest) {
        if (run_usb_selftest(&proto, session_id, file_buf, total_bytes) != 0) {
            free(file_buf);
            return 1;
        }
    }

    FILE *fp = fopen(out_path, "wb");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open output file: %s\n", out_path);
        free(file_buf);
        return 1;
    }
    if (fwrite(file_buf, 1u, total_bytes, fp) != total_bytes) {
        fprintf(stderr, "Failed writing output file\n");
        fclose(fp);
        free(file_buf);
        return 1;
    }
    fclose(fp);

    const uint32_t file_crc = hks_crc32(file_buf, total_bytes);
    uint8_t file_sha[32];
    hks_sha256(file_buf, total_bytes, file_sha);

    if (info_json_path != NULL) {
        FILE *jfp = fopen(info_json_path, "w");
        if (jfp == NULL) {
            fprintf(stderr, "Failed to open info-json: %s\n", info_json_path);
            free(file_buf);
            return 1;
        }
        fprintf(jfp, "{\n");
        fprintf(jfp, "  \"session_id\": %llu,\n", (unsigned long long)session_id);
        fprintf(jfp, "  \"total_bytes\": %u,\n", total_bytes);
        fprintf(jfp, "  \"duration_ms\": %u,\n", entries[0].duration_ms);
        fprintf(jfp, "  \"imu_samples\": %u,\n", entries[0].sample_count);
        fprintf(jfp, "  \"imu_frames\": %u,\n", imu_frames_logged);
        fprintf(jfp, "  \"high_g_frames\": %u,\n", hg_frames_logged);
        fprintf(jfp, "  \"aux_frames\": %u,\n", aux_frames_logged);
        fprintf(jfp, "  \"event_frames\": %u,\n", event_frames_logged);
        fprintf(jfp, "  \"nop_violations\": %u,\n", hks_hal_mock_get_nop_violations());
        fprintf(jfp, "  \"file_crc32\": %u,\n", file_crc);
        fprintf(jfp, "  \"file_sha256\": \"");
        for (size_t k = 0u; k < 32u; k++) {
            fprintf(jfp, "%02x", file_sha[k]);
        }
        fprintf(jfp, "\"\n}\n");
        fclose(jfp);
    }

    printf("HKS_SIM_CLI_OK out=%s bytes=%u duration_ms=%u imu_samples=%u hg_frames=%u aux_frames=%u evt_frames=%u crc32=0x%08X usb_selftest=%s nop_violations=%u\n",
           out_path, total_bytes, entries[0].duration_ms, entries[0].sample_count,
           hg_frames_logged, aux_frames_logged, event_frames_logged, file_crc,
           usb_selftest ? "PASS" : "SKIP", hks_hal_mock_get_nop_violations());

    free(file_buf);
    return 0;
}
