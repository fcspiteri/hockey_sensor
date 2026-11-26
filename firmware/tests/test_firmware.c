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

static int g_tests_run = 0;

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        exit(1); \
    } \
} while (0)

#define ASSERT_EQ_U32(a, b) do { \
    uint32_t _va = (uint32_t)(a); \
    uint32_t _vb = (uint32_t)(b); \
    if (_va != _vb) { \
        fprintf(stderr, "FAIL %s:%d: 0x%08X != 0x%08X\n", __FILE__, __LINE__, _va, _vb); \
        exit(1); \
    } \
} while (0)

#define ASSERT_EQ_U64(a, b) do { \
    uint64_t _va = (uint64_t)(a); \
    uint64_t _vb = (uint64_t)(b); \
    if (_va != _vb) { \
        fprintf(stderr, "FAIL %s:%d: %llu != %llu\n", __FILE__, __LINE__, \
                (unsigned long long)_va, (unsigned long long)_vb); \
        exit(1); \
    } \
} while (0)

#define ASSERT_NEAR(a, b, tol) do { \
    double _va = (double)(a); \
    double _vb = (double)(b); \
    if (fabs(_va - _vb) > (double)(tol)) { \
        fprintf(stderr, "FAIL %s:%d: %.6f vs %.6f (tol %.6f)\n", __FILE__, __LINE__, _va, _vb, (double)(tol)); \
        exit(1); \
    } \
} while (0)

#define RUN_TEST(fn) do { \
    g_tests_run++; \
    fn(); \
    printf("  [PASS] %s\n", #fn); \
} while (0)

static void test_crc32_and_sha256_vectors(void) {
    ASSERT_EQ_U32(hks_crc32("123456789", 9u), 0xCBF43926u);

    uint32_t st = hks_crc32_init();
    st = hks_crc32_update(st, "1234", 4u);
    st = hks_crc32_update(st, "56789", 5u);
    ASSERT_EQ_U32(hks_crc32_final(st), 0xCBF43926u);

    const uint8_t exp_empty[32] = {
        0xe3,0xb0,0xc4,0x42,0x98,0xfc,0x1c,0x14,0x9a,0xfb,0xf4,0xc8,0x99,0x6f,0xb9,0x24,
        0x27,0xae,0x41,0xe4,0x64,0x9b,0x93,0x4c,0xa4,0x95,0x99,0x1b,0x78,0x52,0xb8,0x55
    };
    const uint8_t exp_abc[32] = {
        0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
        0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad
    };
    uint8_t digest[32];
    hks_sha256("", 0u, digest);
    ASSERT_TRUE(memcmp(digest, exp_empty, 32u) == 0);
    hks_sha256("abc", 3u, digest);
    ASSERT_TRUE(memcmp(digest, exp_abc, 32u) == 0);
}

static void test_adxl372_unpack_and_misaligned_serdes(void) {
    ASSERT_TRUE(hks_adxl372_unpack_i12(0x8000u) == -2048);
    ASSERT_TRUE(hks_adxl372_unpack_i12(0x7FF0u) == 2047);
    ASSERT_TRUE(hks_adxl372_unpack_i12(0xFFF0u) == -1);
    ASSERT_TRUE(hks_adxl372_unpack_i12(0xFFFFu) == -1);
    ASSERT_TRUE(hks_adxl372_unpack_i12(0x0000u) == 0);

    /* Deliberately misalign wire buffer at +1, +3, +5 offsets under UBSan/ASan */
    uint8_t raw[512];
    for (size_t off = 1u; off <= 3u; off++) {
        uint8_t *p = raw + off;

        hks_session_header_t sh_in, sh_out;
        memset(&sh_in, 0, sizeof(sh_in));
        memcpy(sh_in.magic, "HKS1", 4u);
        sh_in.format_version = 1u;
        sh_in.hw_rev = 0x0100u;
        sh_in.fw_version = 0x01000000u;
        sh_in.cal_crc32 = 0xDEADBEEFu;
        sh_in.session_id = 0x1122334455667788ULL;
        sh_in.start_utc_ms = 1763852500000ULL;
        sh_in.device_id = 102030405060708ULL;
        sh_in.imu_odr_hz = 200u;
        sh_in.high_g_odr_hz = 3200u;
        sh_in.mag_odr_hz = 50u;
        sh_in.baro_odr_hz = 25u;
        sh_in.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
        sh_in.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
        sh_in.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
        sh_in.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;
        sh_in.rink_preset = HKS_RINK_NHL_200X85FT;
        sh_in.mount_location = HKS_MOUNT_STERNUM_POCKET;
        sh_in.player_height_cm = 183u;
        sh_in.bench_gate_x_m = -6.0f;
        sh_in.bench_gate_y_m = 12.954f;

        uint8_t pl[HKS_PAYLOAD_CAL_BLOCK_BYTES];
        ASSERT_TRUE(hks_encode_session_header(pl, sizeof(pl), &sh_in) == 80u);
        const size_t flen = hks_frame_encode(p, sizeof(raw) - off, HKS_MSG_SESSION_HEADER,
                                             HKS_FLAG_UTC_SYNCED, 7u, 7200000000ULL, pl, 80u);
        ASSERT_TRUE(flen == 104u);

        hks_frame_hdr_t fh;
        ASSERT_TRUE(hks_frame_verify(p, flen, &fh));
        ASSERT_EQ_U64(fh.timestamp_us, 7200000000ULL);
        ASSERT_TRUE(hks_decode_session_header(p + HKS_FRAME_HDR_BYTES, fh.payload_len, &sh_out));
        ASSERT_EQ_U64(sh_out.session_id, sh_in.session_id);
        ASSERT_NEAR(sh_out.bench_gate_y_m, 12.954f, 1e-6);

        hks_cal_block_t cal_in, cal_out;
        hks_cal_block_init_default(&cal_in, 1763852400000ULL);
        ASSERT_TRUE(hks_encode_cal_block(pl, sizeof(pl), &cal_in, true) == 288u);
        ASSERT_TRUE(hks_frame_encode(p, sizeof(raw) - off, HKS_MSG_CALIBRATION_BLOCK,
                                     0u, 8u, 7200005000ULL, pl, 288u) == 312u);
        ASSERT_TRUE(hks_frame_verify(p, 312u, &fh));
        ASSERT_TRUE(hks_decode_cal_block(p + HKS_FRAME_HDR_BYTES, fh.payload_len, &cal_out, true));
        ASSERT_EQ_U32(cal_out.cal_crc32, cal_in.cal_crc32);
    }
}

static void test_nand_nop4_and_bad_block_skip(void) {
    hks_hal_mock_reset();
    ASSERT_TRUE(hks_hal_nand_init() == 0);

    /* Verify explicit NOP > 4 enforcement in hal_mock */
    uint8_t page[HKS_NAND_PAGE_BYTES];
    memset(page, 0xFF, sizeof(page));
    page[0] = 0xFEu;
    for (int i = 0; i < 4; i++) {
        ASSERT_TRUE(hks_hal_nand_prog_page(500u, 0u, 0u, page, sizeof(page)) == 0);
    }
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_TRUE(hks_hal_nand_prog_page(500u, 0u, 0u, page, sizeof(page)) != 0);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 1u);
    ASSERT_TRUE(hks_hal_nand_erase_block(500u) == 0);

    /* Reset and inject bad blocks in FACTORY_CAL (0), SESSION_INDEX (2), and TELEMETRY_RING (6, 7) */
    hks_hal_mock_reset();
    hks_hal_mock_inject_bad_block(0u, false);
    hks_hal_mock_inject_bad_block(2u, false);
    hks_hal_mock_inject_bad_block(6u, false);
    hks_hal_mock_inject_bad_block(7u, true); /* runtime I/O fail */

    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);
    ASSERT_TRUE(hks_flash_log_write_factory_cal(&log, &cal) == 0);

    hks_cal_block_t cal_read;
    ASSERT_TRUE(hks_flash_log_read_factory_cal(&log, &cal_read) == 0);
    ASSERT_EQ_U32(cal_read.cal_crc32, cal.cal_crc32);

    hks_session_header_t sh;
    memset(&sh, 0, sizeof(sh));
    memcpy(sh.magic, "HKS1", 4u);
    sh.format_version = 1u;
    sh.hw_rev = 0x0100u;
    sh.fw_version = 0x01000000u;
    sh.session_id = 42ULL;
    sh.start_utc_ms = 1763852500000ULL;
    sh.device_id = 12345ULL;
    sh.imu_odr_hz = 200u;
    sh.high_g_odr_hz = 3200u;
    sh.mag_odr_hz = 50u;
    sh.baro_odr_hz = 25u;
    sh.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;

    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, HKS_FLAG_UTC_SYNCED, 7200000000ULL) == 0);
    ASSERT_TRUE(log.rec_start_block == 8u); /* skipped bad blocks 6 and 7 */

    hks_imu_sample_t s[20];
    memset(s, 0, sizeof(s));
    s[0].ax = 2048;
    hks_imu_batch_hdr_t bh = { .sample_count = 20u, .odr_hz = 200u, .temp_c_x100 = 2100, .reserved = 0u };
    for (int i = 0; i < 20; i++) {
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, 7200000000ULL + (uint64_t)i * 100000ULL, &bh, s) == 0);
    }
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 1763852510000ULL, 7202000000ULL) == 0);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);

    /* Ensure block 6 stayed completely untouched (page 0 NOP == 0) */
    ASSERT_TRUE(hks_hal_mock_get_page_nop(6u, 0u) == 0u);
}

static void test_watchdog_recover_torn_frame(void) {
    hks_hal_mock_reset();
    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);

    hks_session_header_t sh;
    memset(&sh, 0, sizeof(sh));
    memcpy(sh.magic, "HKS1", 4u);
    sh.format_version = 1u;
    sh.hw_rev = 0x0100u;
    sh.fw_version = 0x01000000u;
    sh.session_id = 777ULL;
    sh.start_utc_ms = 1763852500000ULL;
    sh.device_id = 999ULL;
    sh.imu_odr_hz = 200u;
    sh.high_g_odr_hz = 3200u;
    sh.mag_odr_hz = 50u;
    sh.baro_odr_hz = 25u;
    sh.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;

    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, HKS_FLAG_UTC_SYNCED, 1000ULL) == 0);
    hks_imu_sample_t s[20];
    memset(s, 0, sizeof(s));
    hks_imu_batch_hdr_t bh = { .sample_count = 20u, .odr_hz = 200u, .temp_c_x100 = 2000, .reserved = 0u };
    /* Write enough batches to flush page 0 and leave a partial page in SRAM, plus simulate a torn page in flash */
    for (int i = 0; i < 10; i++) {
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, 1000ULL + (uint64_t)i * 100000ULL, &bh, s) == 0);
    }
    /* Simulate torn frame flushed to current page right before sudden power loss */
    ASSERT_TRUE(log.page_fill > 40u);
    log.page_buf[log.page_fill - 10u] ^= 0xFFu; /* corrupt last frame CRC */
    memset(log.page_buf + log.page_fill, 0xFF, HKS_NAND_PAGE_BYTES - log.page_fill);
    ASSERT_TRUE(hks_hal_nand_prog_page(log.rec_cur_block, log.rec_cur_page, 0u, log.page_buf, HKS_NAND_PAGE_BYTES) == 0);

    /* Reboot! hks_flash_log_init() should detect OPEN session and WATCHDOG_RECOVER it */
    static hks_flash_log_t rebooted;
    ASSERT_TRUE(hks_flash_log_init(&rebooted) == 0);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);

    hks_session_list_entry_t entries[4];
    size_t cnt = 0u;
    ASSERT_TRUE(hks_flash_log_list_sessions(&rebooted, entries, 4u, &cnt) == HKS_STATUS_OK);
    ASSERT_TRUE(cnt == 1u);
    ASSERT_EQ_U64(entries[0].session_id, 777ULL);

    const uint32_t sz = entries[0].byte_size;
    uint8_t *buf = (uint8_t *)malloc(sz);
    ASSERT_TRUE(buf != NULL);
    uint32_t off = 0u;
    while (off < sz) {
        const uint16_t want = (sz - off > 4096u) ? 4096u : (uint16_t)(sz - off);
        uint16_t got = 0u;
        ASSERT_TRUE(hks_flash_log_read_bytes(&rebooted, 777ULL, off, want, buf + off, &got) == HKS_STATUS_OK);
        off += got;
    }

    /* Last 96 bytes must be valid 0x07 SESSION_FOOTER with WATCHDOG_RECOVER and matching pre_footer_sha256 */
    ASSERT_TRUE(sz > HKS_FRAME_SESSION_FOOTER_BYTES);
    hks_frame_hdr_t ftr_hdr;
    ASSERT_TRUE(hks_frame_verify(buf + sz - HKS_FRAME_SESSION_FOOTER_BYTES,
                                 HKS_FRAME_SESSION_FOOTER_BYTES, &ftr_hdr));
    ASSERT_TRUE(ftr_hdr.msg_type == HKS_MSG_SESSION_FOOTER);
    hks_session_footer_t ftr;
    ASSERT_TRUE(hks_decode_session_footer(buf + sz - HKS_FRAME_SESSION_FOOTER_BYTES + HKS_FRAME_HDR_BYTES,
                                          HKS_PAYLOAD_SESSION_FOOTER_BYTES, &ftr));
    ASSERT_EQ_U32(ftr.end_reason, HKS_END_WATCHDOG_RECOVER);
    ASSERT_EQ_U64(ftr.total_bytes, sz);

    uint8_t exp_pre_sha[32];
    hks_sha256(buf, sz - HKS_FRAME_SESSION_FOOTER_BYTES, exp_pre_sha);
    ASSERT_TRUE(memcmp(ftr.pre_footer_sha256, exp_pre_sha, 32u) == 0);

    uint8_t whole_sha[32];
    hks_sha256(buf, sz, whole_sha);
    ASSERT_TRUE(hks_flash_log_verify_session(&rebooted, 777ULL, hks_crc32(buf, sz), whole_sha, true) == HKS_STATUS_OK);
    free(buf);
}

static void send_cmd_expect(hks_protocol_t *proto, uint8_t cmd_id, uint8_t seq,
                            const uint8_t *pl, uint16_t plen,
                            uint8_t expected_status, uint8_t *rsp_buf, size_t *rsp_len_out) {
    uint8_t cmd[256];
    const size_t clen = hks_protocol_encode_cmd(cmd, sizeof(cmd), cmd_id, seq, pl, plen);
    ASSERT_TRUE(clen > 0u);
    const size_t rlen = hks_protocol_feed_bytes(proto, cmd, clen, rsp_buf, HKS_PROTO_MAX_TX_FRAME);
    ASSERT_TRUE(rlen >= HKS_PROTO_RSP_OVERHEAD_BYTES);
    ASSERT_EQ_U32(hks_get_u16_le(rsp_buf), HKS_RSP_SYNC_MARKER);
    ASSERT_EQ_U32(rsp_buf[2], cmd_id);
    ASSERT_EQ_U32(rsp_buf[3], expected_status);
    ASSERT_EQ_U32(rsp_buf[4], seq);
    const uint16_t out_plen = hks_get_u16_le(rsp_buf + 6u);
    ASSERT_EQ_U32(rlen, HKS_PROTO_RSP_OVERHEAD_BYTES + out_plen);
    const uint32_t exp_crc = hks_crc32(rsp_buf, HKS_PROTO_RSP_HDR_BYTES + out_plen);
    const uint32_t act_crc = hks_get_u32_le(rsp_buf + HKS_PROTO_RSP_HDR_BYTES + out_plen);
    ASSERT_EQ_U32(act_crc, exp_crc);
    if (rsp_len_out != NULL) {
        *rsp_len_out = rlen;
    }
}

static void test_usb_protocol_all_commands_and_10_statuses(void) {
    hks_hal_mock_reset();
    hks_hal_pmic_info_t pmic = {
        .vbatt_mv = 3810u,
        .batt_temp_c_x100 = 1200,
        .soc_pct_x10 = 735u,
        .pmic_status = (uint8_t)(HKS_PMIC_VBUS_PRESENT | HKS_PMIC_CHARGING | HKS_PMIC_FG_VALID)
    };
    hks_hal_mock_set_pmic(&pmic);

    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);
    hks_protocol_t proto;
    hks_protocol_init(&proto, &log, 0x01020304u, 0x0100u);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 5000ULL);
    hks_session_header_t sh;
    memset(&sh, 0, sizeof(sh));
    memcpy(sh.magic, "HKS1", 4u);
    sh.format_version = 1u;
    sh.hw_rev = 0x0100u;
    sh.fw_version = 0x01020304u;
    sh.session_id = 2026ULL;
    sh.start_utc_ms = 1763852500000ULL;
    sh.device_id = 555ULL;
    sh.imu_odr_hz = 200u;
    sh.high_g_odr_hz = 3200u;
    sh.mag_odr_hz = 50u;
    sh.baro_odr_hz = 25u;
    sh.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;

    uint8_t rsp[HKS_PROTO_MAX_TX_FRAME];
    size_t rlen = 0u;

    /* Status 0x07 ERR_BUSY_RECORDING during active session, while GET_DEVICE_STATUS still works */
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, HKS_FLAG_UTC_SYNCED, 0ULL) == 0);
    send_cmd_expect(&proto, HKS_CMD_LIST_SESSIONS, 1u, NULL, 0u, HKS_STATUS_ERR_BUSY_RECORDING, rsp, &rlen);
    send_cmd_expect(&proto, HKS_CMD_GET_DEVICE_STATUS, 2u, NULL, 0u, HKS_STATUS_OK, rsp, &rlen);
    ASSERT_EQ_U32(rsp[HKS_PROTO_RSP_HDR_BYTES + 10u], 1u); /* recording_state == 1 */
    /* pmic_status parity check: bits [7:0] */
    hks_aux_env_t aux = { .mx_raw = 1, .my_raw = 2, .mz_raw = 3, .pressure_pa_x64 = 6400000u,
                          .baro_temp_c_x100 = 2000, .vbatt_mv = pmic.vbatt_mv,
                          .batt_temp_c_x100 = pmic.batt_temp_c_x100, .soc_pct_x10 = pmic.soc_pct_x10,
                          .pmic_status = (uint16_t)pmic.pmic_status, .validity_flags = 7u };
    ASSERT_EQ_U32(rsp[HKS_PROTO_RSP_HDR_BYTES + 11u], aux.pmic_status & 0xFFu);
    ASSERT_TRUE(hks_flash_log_append_aux_env(&log, HKS_FLAG_UTC_SYNCED, 7200000000ULL, &aux) == 0);
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 1763859700000ULL, 7200000000ULL) == 0);

    /* Status 0x01 ERR_BAD_CRC */
    uint8_t bad_cmd[16];
    const size_t blen = hks_protocol_encode_cmd(bad_cmd, sizeof(bad_cmd), HKS_CMD_LIST_SESSIONS, 3u, NULL, 0u);
    bad_cmd[blen - 1u] ^= 0xFFu;
    rlen = hks_protocol_feed_bytes(&proto, bad_cmd, blen, rsp, sizeof(rsp));
    ASSERT_EQ_U32(rsp[3], HKS_STATUS_ERR_BAD_CRC);

    /* Status 0x02 ERR_UNKNOWN_CMD */
    send_cmd_expect(&proto, 0x7Eu, 4u, NULL, 0u, HKS_STATUS_ERR_UNKNOWN_CMD, rsp, &rlen);

    /* Status 0x09 ERR_INVALID_PARAM */
    uint8_t dummy[2] = { 0u, 0u };
    send_cmd_expect(&proto, HKS_CMD_LIST_SESSIONS, 5u, dummy, 2u, HKS_STATUS_ERR_INVALID_PARAM, rsp, &rlen);

    /* Status 0x03 ERR_SESSION_NOT_FOUND */
    uint8_t bad_sid[8];
    hks_put_u64_le(bad_sid, 999999ULL);
    send_cmd_expect(&proto, HKS_CMD_GET_HEADER, 6u, bad_sid, 8u, HKS_STATUS_ERR_SESSION_NOT_FOUND, rsp, &rlen);

    /* Status 0x00 OK: LIST_SESSIONS, GET_HEADER, READ_CHUNK, SET_UTC_TIME */
    send_cmd_expect(&proto, HKS_CMD_LIST_SESSIONS, 7u, NULL, 0u, HKS_STATUS_OK, rsp, &rlen);
    const uint32_t byte_size = hks_get_u32_le(rsp + HKS_PROTO_RSP_HDR_BYTES + 4u + 20u);
    ASSERT_TRUE(byte_size > HKS_GET_HEADER_RSP_BYTES);

    uint8_t sid_pl[8];
    hks_put_u64_le(sid_pl, 2026ULL);
    send_cmd_expect(&proto, HKS_CMD_GET_HEADER, 8u, sid_pl, 8u, HKS_STATUS_OK, rsp, &rlen);

    /* Status 0x04 ERR_OFFSET_OUT_OF_RANGE */
    uint8_t rc_bad[14];
    hks_put_u64_le(rc_bad, 2026ULL);
    hks_put_u32_le(rc_bad + 8u, byte_size);
    hks_put_u16_le(rc_bad + 12u, 64u);
    send_cmd_expect(&proto, HKS_CMD_READ_CHUNK, 9u, rc_bad, 14u, HKS_STATUS_ERR_OFFSET_OUT_OF_RANGE, rsp, &rlen);

    /* Read complete session bytes */
    uint8_t *file_buf = (uint8_t *)malloc(byte_size);
    ASSERT_TRUE(file_buf != NULL);
    uint8_t rc_ok[14];
    hks_put_u64_le(rc_ok, 2026ULL);
    hks_put_u32_le(rc_ok + 8u, 0u);
    hks_put_u16_le(rc_ok + 12u, 4096u);
    send_cmd_expect(&proto, HKS_CMD_READ_CHUNK, 10u, rc_ok, 14u, HKS_STATUS_OK, rsp, &rlen);
    const uint16_t got_chunk = hks_get_u16_le(rsp + HKS_PROTO_RSP_HDR_BYTES + 4u);
    ASSERT_EQ_U32(got_chunk, byte_size);
    memcpy(file_buf, rsp + HKS_PROTO_RSP_HDR_BYTES + 6u, got_chunk);

    /* Status 0x05 ERR_NOT_VERIFIED on DELETE_SESSION(force=0) before ACK_VERIFY */
    uint8_t del_pl[9];
    hks_put_u64_le(del_pl, 2026ULL);
    del_pl[8] = 0u;
    send_cmd_expect(&proto, HKS_CMD_DELETE_SESSION, 11u, del_pl, 9u, HKS_STATUS_ERR_NOT_VERIFIED, rsp, &rlen);

    /* Status 0x08 ERR_VERIFY_MISMATCH */
    uint8_t ack_pl[44];
    memset(ack_pl, 0, sizeof(ack_pl));
    hks_put_u64_le(ack_pl, 2026ULL);
    hks_put_u32_le(ack_pl + 8u, hks_crc32(file_buf, byte_size) ^ 1u);
    send_cmd_expect(&proto, HKS_CMD_ACK_VERIFY, 12u, ack_pl, 44u, HKS_STATUS_ERR_VERIFY_MISMATCH, rsp, &rlen);

    /* ACK_VERIFY hash_mode=1 (all-zero SHA-256) */
    hks_put_u32_le(ack_pl + 8u, hks_crc32(file_buf, byte_size));
    send_cmd_expect(&proto, HKS_CMD_ACK_VERIFY, 13u, ack_pl, 44u, HKS_STATUS_OK, rsp, &rlen);
    ASSERT_EQ_U32(rsp[HKS_PROTO_RSP_HDR_BYTES + 1u], 1u);

    /* ACK_VERIFY hash_mode=2 (CRC32 + SHA-256) */
    hks_sha256(file_buf, byte_size, ack_pl + 12u);
    send_cmd_expect(&proto, HKS_CMD_ACK_VERIFY, 14u, ack_pl, 44u, HKS_STATUS_OK, rsp, &rlen);
    ASSERT_EQ_U32(rsp[HKS_PROTO_RSP_HDR_BYTES + 1u], 2u);

    /* Status 0x06 ERR_FLASH_IO: inject runtime I/O fail on session's block and test READ_CHUNK */
    hks_hal_mock_inject_bad_block(log.sessions[0].start_block, true);
    send_cmd_expect(&proto, HKS_CMD_READ_CHUNK, 15u, rc_ok, 14u, HKS_STATUS_ERR_FLASH_IO, rsp, &rlen);

    /* Clear mock fault & verify DELETE_SESSION(force=0) succeeds now that session is VERIFIED */
    hks_hal_mock_reset();
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);
    hks_protocol_init(&proto, &log, 0x01020304u, 0x0100u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, HKS_FLAG_UTC_SYNCED, 0ULL) == 0);
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 1763852600000ULL, 100000ULL) == 0);
    uint16_t sz2 = 0u;
    uint8_t buf2[1024];
    ASSERT_TRUE(hks_flash_log_read_bytes(&log, 2026ULL, 0u, 1024u, buf2, &sz2) == HKS_STATUS_OK);
    hks_put_u64_le(ack_pl, 2026ULL);
    hks_put_u32_le(ack_pl + 8u, hks_crc32(buf2, sz2));
    hks_sha256(buf2, sz2, ack_pl + 12u);
    send_cmd_expect(&proto, HKS_CMD_ACK_VERIFY, 16u, ack_pl, 44u, HKS_STATUS_OK, rsp, &rlen);
    send_cmd_expect(&proto, HKS_CMD_DELETE_SESSION, 17u, del_pl, 9u, HKS_STATUS_OK, rsp, &rlen);
    ASSERT_EQ_U32(hks_get_u32_le(rsp + HKS_PROTO_RSP_HDR_BYTES), HKS_NAND_BLOCK_BYTES);

    /* Status 0x07 SET_UTC_TIME */
    uint8_t utc_pl[8];
    hks_put_u64_le(utc_pl, 1763853000000ULL);
    send_cmd_expect(&proto, HKS_CMD_SET_UTC_TIME, 18u, utc_pl, 8u, HKS_STATUS_OK, rsp, &rlen);
    ASSERT_EQ_U64(hks_get_u64_le(rsp + HKS_PROTO_RSP_HDR_BYTES), 1763853000000ULL);

    free(file_buf);
}

static void test_ring_reclamation_verified_vs_unverified(void) {
    hks_hal_mock_reset();
    /* Keep only blocks 6, 7, 8 usable in TELEMETRY_RING so ring fills deterministically */
    for (uint16_t b = 9u; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        hks_hal_mock_inject_bad_block(b, false);
    }

    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);

    hks_session_header_t sh;
    memset(&sh, 0, sizeof(sh));
    memcpy(sh.magic, "HKS1", 4u);
    sh.format_version = 1u;
    sh.hw_rev = 0x0100u;
    sh.fw_version = 0x01000000u;
    sh.device_id = 111ULL;
    sh.imu_odr_hz = 200u;
    sh.high_g_odr_hz = 3200u;
    sh.mag_odr_hz = 50u;
    sh.baro_odr_hz = 25u;
    sh.accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh.gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh.high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh.mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;

    /* Session 1 (verified), Session 2 (unverified), Session 3 (unverified) occupy blocks 6, 7, 8 */
    for (uint64_t sid = 1u; sid <= 3u; sid++) {
        sh.session_id = sid;
        ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, sid * 1000ULL) == 0);
        ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, sid * 1000ULL + 10000ULL) == 0);
    }

    /* Verify Session 1 only */
    uint8_t s1_buf[1024];
    uint16_t s1_len = 0u;
    ASSERT_TRUE(hks_flash_log_read_bytes(&log, 1u, 0u, sizeof(s1_buf), s1_buf, &s1_len) == HKS_STATUS_OK);
    ASSERT_TRUE(hks_flash_log_verify_session(&log, 1u, hks_crc32(s1_buf, s1_len), NULL, false) == HKS_STATUS_OK);

    /* Session 4 should succeed by reclaiming verified Session 1 */
    sh.session_id = 4u;
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 4000ULL) == 0);
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, 14000ULL) == 0);

    /* Now sessions 2, 3, 4 are ALL unverified; Session 5 MUST fail rather than overwrite unverified data */
    sh.session_id = 5u;
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 5000ULL) == -2);

    hks_session_list_entry_t list[8];
    size_t cnt = 0u;
    ASSERT_TRUE(hks_flash_log_list_sessions(&log, list, 8u, &cnt) == HKS_STATUS_OK);
    ASSERT_TRUE(cnt == 3u);
    for (size_t i = 0u; i < cnt; i++) {
        ASSERT_TRUE(list[i].session_id != 1u);
    }
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);
}

static void test_r_bs_dual_imu_attitude_and_detectors(void) {
    const float r_bs[9] = { 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f };
    ASSERT_NEAR(hks_mat3_det(r_bs), 1.0f, 1e-6);

    hks_attitude_t att;
    hks_attitude_init(&att, NULL);

    /* Stationary upright: f^S = [+9.80665, 0, 0] (ax_i16 = +2048) -> a_lin^W == [0, 0, 0] */
    hks_imu_sample_t upright = { .ax = 2048, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0 };
    hks_attitude_step_out_t out;
    hks_attitude_step(&att, &upright, NULL, 5000ULL, 0.005f, &out);
    ASSERT_NEAR(att.f_b[0], 0.0f, 1e-4);
    ASSERT_NEAR(att.f_b[1], 0.0f, 1e-4);
    ASSERT_NEAR(att.f_b[2], HKS_GRAVITY_MPS2_F, 1e-3);
    ASSERT_NEAR(att.a_lin_w[0], 0.0f, 1e-4);
    ASSERT_NEAR(att.a_lin_w[1], 0.0f, 1e-4);
    ASSERT_NEAR(att.a_lin_w[2], 0.0f, 1e-3);

    /* Dual-IMU handoff: 14.0 g stays on ICM; 15.0 g (> 14.5 g) switches to ADXL372;
     * hysteresis holds for 3 samples < 12.0 g and clears on 4th sample (< 12.0 g for 20 ms). */
    const int16_t icm_14g = (int16_t)lrintf(14.0f * 2048.0f);
    const int16_t icm_15g = (int16_t)lrintf(15.0f * 2048.0f);
    hks_imu_sample_t s14 = { .ax = 2048, .ay = 0, .az = icm_14g, .gx = 0, .gy = 0, .gz = 0 };
    hks_imu_sample_t s15 = { .ax = 2048, .ay = 0, .az = icm_15g, .gx = 0, .gy = 0, .gz = 0 };
    hks_high_g_sample_t hg39 = { .ax = 10, .ay = 0, .az = 390 }; /* +39 g on +Z_S (+X_B) */

    hks_attitude_step(&att, &s14, NULL, 10000ULL, 0.005f, &out);
    ASSERT_TRUE(!att.high_g_active);
    ASSERT_TRUE((out.frame_flags & HKS_FLAG_HIGH_G_ACTIVE) == 0u);

    hks_attitude_step(&att, &s15, &hg39, 15000ULL, 0.005f, &out);
    ASSERT_TRUE(att.high_g_active);
    ASSERT_TRUE((out.frame_flags & (HKS_FLAG_ACCEL_SAT | HKS_FLAG_HIGH_G_ACTIVE)) ==
                (HKS_FLAG_ACCEL_SAT | HKS_FLAG_HIGH_G_ACTIVE));
    ASSERT_NEAR(att.f_b[0], 390.0f * HKS_HIGH_G_SCALE_MPS2_PER_LSB, 1e-3);

    for (int k = 0; k < 3; k++) {
        hks_attitude_step(&att, &upright, NULL, 20000ULL + (uint64_t)k * 5000ULL, 0.005f, &out);
        ASSERT_TRUE(att.high_g_active);
    }
    hks_attitude_step(&att, &upright, NULL, 35000ULL, 0.005f, &out);
    ASSERT_TRUE(!att.high_g_active);

    /* Wait > 250 ms and verify IMPACT_SPIKE event emitted with ~39 g peak */
    hks_attitude_step(&att, &upright, NULL, 300000ULL, 0.005f, &out);
    ASSERT_TRUE(out.emit_impact);
    ASSERT_EQ_U32(out.impact_event.event_code, HKS_EVT_IMPACT_SPIKE);
    ASSERT_TRUE(out.impact_event.severity >= HKS_SEV_WARNING);
    ASSERT_TRUE(out.impact_event.value_f32 > 350.0f);

    /* 90 deg yaw integration & unit-norm check */
    hks_attitude_init(&att, NULL);
    const float target_wz = (3.14159265358979f / 2.0f); /* 90 deg/s for 1.0 s */
    const int16_t gx_s_for_gz_b = (int16_t)lrintf(target_wz / HKS_GYRO_SCALE_RADS_PER_LSB);
    hks_imu_sample_t yaw90 = { .ax = 2048, .ay = 0, .az = 0, .gx = gx_s_for_gz_b, .gy = 0, .gz = 0 };
    for (int k = 0; k < 200; k++) {
        hks_attitude_step(&att, &yaw90, NULL, (uint64_t)(k + 1) * 5000ULL, 0.005f, &out);
    }
    const float qnorm = sqrtf(att.q_wb[0]*att.q_wb[0] + att.q_wb[1]*att.q_wb[1] +
                              att.q_wb[2]*att.q_wb[2] + att.q_wb[3]*att.q_wb[3]);
    ASSERT_NEAR(qnorm, 1.0f, 1e-6);
    ASSERT_NEAR(fabsf(att.q_wb[0]), 0.70710678f, 1e-3);
    ASSERT_NEAR(fabsf(att.q_wb[3]), 0.70710678f, 1e-3);

    /* Shift hint ON -> OFF state machine */
    hks_attitude_init(&att, NULL);
    bool saw_on = false;
    bool saw_off = false;
    uint64_t ts = 0ULL;
    for (int k = 0; k < 800; k++) { /* 4.0 s skating */
        const float t = (float)k * 0.005f;
        const float dyn_mps2 = 3.5f * sinf(2.0f * 3.14159265f * 1.5f * t);
        hks_imu_sample_t skate = {
            .ax = 2048,
            .ay = (int16_t)lrintf(dyn_mps2 / HKS_ACCEL_SCALE_MPS2_PER_LSB),
            .az = 0, .gx = 0, .gy = 0, .gz = 0
        };
        ts += 5000ULL;
        hks_attitude_step(&att, &skate, NULL, ts, 0.005f, &out);
        if (out.emit_shift_hint && out.shift_event.event_code == HKS_EVT_SHIFT_HINT_ON) {
            saw_on = true;
        }
    }
    ASSERT_TRUE(saw_on);
    for (int k = 0; k < 1400; k++) { /* 7.0 s bench stillness */
        ts += 5000ULL;
        hks_attitude_step(&att, &upright, NULL, ts, 0.005f, &out);
        if (out.emit_shift_hint && out.shift_event.event_code == HKS_EVT_SHIFT_HINT_OFF) {
            saw_off = true;
        }
    }
    ASSERT_TRUE(saw_off);
}

static void init_test_sh(hks_session_header_t *sh, uint64_t sid) {
    memset(sh, 0, sizeof(*sh));
    memcpy(sh->magic, "HKS1", 4u);
    sh->format_version = 1u;
    sh->hw_rev = 0x0100u;
    sh->fw_version = 0x01000000u;
    sh->session_id = sid;
    sh->start_utc_ms = 1763852500000ULL;
    sh->device_id = 12345ULL;
    sh->imu_odr_hz = 200u;
    sh->high_g_odr_hz = 3200u;
    sh->mag_odr_hz = 50u;
    sh->baro_odr_hz = 25u;
    sh->accel_scale_mps2_per_lsb = HKS_ACCEL_SCALE_MPS2_PER_LSB;
    sh->gyro_scale_rads_per_lsb = HKS_GYRO_SCALE_RADS_PER_LSB;
    sh->high_g_scale_mps2_per_lsb = HKS_HIGH_G_SCALE_MPS2_PER_LSB;
    sh->mag_scale_ut_per_lsb = HKS_MAG_SCALE_UT_PER_LSB;
}

static void verify_session_stream_and_footer(hks_flash_log_t *log,
                                             uint64_t sid,
                                             hks_end_reason_t expected_reason,
                                             uint32_t expected_dropped_imu,
                                             uint32_t expected_dropped_hg) {
    hks_session_list_entry_t list[HKS_FLASH_MAX_SESSIONS];
    size_t cnt = 0u;
    ASSERT_TRUE(hks_flash_log_list_sessions(log, list, HKS_FLASH_MAX_SESSIONS, &cnt) == HKS_STATUS_OK);
    uint32_t byte_size = 0u;
    bool found = false;
    for (size_t i = 0u; i < cnt; i++) {
        if (list[i].session_id == sid) {
            byte_size = list[i].byte_size;
            found = true;
            break;
        }
    }
    ASSERT_TRUE(found);
    ASSERT_TRUE(byte_size > HKS_FRAME_SESSION_FOOTER_BYTES);

    uint8_t *buf = (uint8_t *)malloc(byte_size);
    ASSERT_TRUE(buf != NULL);
    uint32_t off = 0u;
    while (off < byte_size) {
        const uint16_t want = (byte_size - off > 4096u) ? 4096u : (uint16_t)(byte_size - off);
        uint16_t got = 0u;
        ASSERT_TRUE(hks_flash_log_read_bytes(log, sid, off, want, buf + off, &got) == HKS_STATUS_OK);
        ASSERT_TRUE(got == want);
        off += got;
    }

    /* Verify every frame's header + CRC32-IEEE */
    uint32_t pos = 0u;
    hks_frame_hdr_t last_hdr;
    memset(&last_hdr, 0, sizeof(last_hdr));
    while (pos < byte_size) {
        hks_frame_hdr_t fh;
        ASSERT_TRUE(hks_frame_verify(buf + pos, byte_size - pos, &fh));
        last_hdr = fh;
        pos += (uint32_t)HKS_FRAME_OVERHEAD_BYTES + (uint32_t)fh.payload_len;
    }
    ASSERT_EQ_U32(pos, byte_size);
    ASSERT_EQ_U32(last_hdr.msg_type, HKS_MSG_SESSION_FOOTER);

    hks_session_footer_t ftr;
    ASSERT_TRUE(hks_decode_session_footer(buf + byte_size - HKS_FRAME_SESSION_FOOTER_BYTES + HKS_FRAME_HDR_BYTES,
                                          HKS_PAYLOAD_SESSION_FOOTER_BYTES, &ftr));
    ASSERT_EQ_U32(ftr.end_reason, (uint32_t)expected_reason);
    ASSERT_EQ_U64(ftr.total_bytes, byte_size);
    ASSERT_EQ_U32(ftr.dropped_imu_samples, expected_dropped_imu);
    ASSERT_EQ_U32(ftr.dropped_high_g_samples, expected_dropped_hg);

    uint8_t exp_pre_sha[32];
    hks_sha256(buf, byte_size - HKS_FRAME_SESSION_FOOTER_BYTES, exp_pre_sha);
    ASSERT_TRUE(memcmp(ftr.pre_footer_sha256, exp_pre_sha, 32u) == 0);

    uint8_t whole_sha[32];
    hks_sha256(buf, byte_size, whole_sha);
    ASSERT_TRUE(hks_flash_log_verify_session(log, sid, hks_crc32(buf, byte_size), whole_sha, true) == HKS_STATUS_OK);
    free(buf);
}

static void test_noncontiguous_session_block_chain_reboot(void) {
    hks_hal_mock_reset();
    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);
    hks_session_header_t sh;

    /* Record Session 1 (1 blk), Session 2 (1 blk, unverified), Session 3 (1 blk, unverified) */
    for (uint64_t sid = 1u; sid <= 3u; sid++) {
        init_test_sh(&sh, sid);
        ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, sid * 1000ULL) == 0);
        hks_imu_sample_t s[10];
        memset(s, 0, sizeof(s));
        s[0].ax = (int16_t)(100 + sid);
        hks_imu_batch_hdr_t bh = { .sample_count = 10u, .odr_hz = 200u, .temp_c_x100 = 2100, .reserved = 0u };
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, sid * 1000ULL + 5000ULL, &bh, s) == 0);
        ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, sid * 1000ULL + 10000ULL) == 0);
    }

    /* Delete Session 1 (force=true), leaving block 6 free while blocks 7 & 8 are held by S2 & S3 */
    uint32_t freed = 0u;
    ASSERT_TRUE(hks_flash_log_delete_session(&log, 1u, true, &freed) == HKS_STATUS_OK);
    ASSERT_EQ_U32(freed, HKS_NAND_BLOCK_BYTES);

    /* Reboot (hks_flash_log_init resets cursor to block 6) */
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    /* Record Session 4 (> 140 KiB) spanning non-contiguously across gap: block 6 -> block 9 */
    init_test_sh(&sh, 4u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 4000ULL) == 0);
    hks_imu_sample_t big_s[338];
    memset(big_s, 0, sizeof(big_s));
    big_s[0].ax = 4096;
    hks_imu_batch_hdr_t big_bh = { .sample_count = 338u, .odr_hz = 200u, .temp_c_x100 = 2200, .reserved = 0u };
    /* 36 * 4088 B = 147,168 B > 140 KiB -> spans 2 blocks [6, 9] */
    for (int i = 0; i < 36; i++) {
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, 4000ULL + (uint64_t)i * 10000ULL, &big_bh, big_s) == 0);
    }
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, 400000ULL) == 0);

    /* Reboot again to verify block chain persistence across reboot */
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    /* Read Session 4 & verify CRC32 / SHA-256 */
    verify_session_stream_and_footer(&log, 4u, HKS_END_BUTTON_STOP, 0u, 0u);

    /* Delete Session 4 and verify Session 2 & Session 3 remain 100% intact */
    freed = 0u;
    ASSERT_TRUE(hks_flash_log_delete_session(&log, 4u, false, &freed) == HKS_STATUS_OK);
    ASSERT_EQ_U32(freed, 2u * HKS_NAND_BLOCK_BYTES);

    verify_session_stream_and_footer(&log, 2u, HKS_END_BUTTON_STOP, 0u, 0u);
    verify_session_stream_and_footer(&log, 3u, HKS_END_BUTTON_STOP, 0u, 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);
}

static void test_mid_session_ring_reclamation_while_recording(void) {
    hks_hal_mock_reset();
    /* Constrain ring to blocks 6, 7, 8 only */
    for (uint16_t b = 9u; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        hks_hal_mock_inject_bad_block(b, false);
    }

    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);
    hks_session_header_t sh;

    /* Session 10 (verified, block 6) and Session 11 (unverified, block 7) */
    init_test_sh(&sh, 10u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 1000ULL) == 0);
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, 2000ULL) == 0);
    verify_session_stream_and_footer(&log, 10u, HKS_END_BUTTON_STOP, 0u, 0u);

    init_test_sh(&sh, 11u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 3000ULL) == 0);
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, 4000ULL) == 0);

    /* Start Session 20 on last free block (block 8) -> ring is now 100% full */
    init_test_sh(&sh, 20u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 5000ULL) == 0);

    /* External delete API must STILL return ERR_BUSY_RECORDING while recording */
    uint32_t ext_freed = 0u;
    ASSERT_EQ_U32(hks_flash_log_delete_session(&log, 10u, false, &ext_freed), HKS_STATUS_ERR_BUSY_RECORDING);

    /* Grow Session 20 past 1 block (> 131 KiB) -> must reclaim verified Session 10 mid-recording */
    hks_imu_sample_t big_s[338];
    memset(big_s, 0, sizeof(big_s));
    big_s[0].ax = 2048;
    hks_imu_batch_hdr_t big_bh = { .sample_count = 338u, .odr_hz = 200u, .temp_c_x100 = 2150, .reserved = 0u };
    for (int i = 0; i < 34; i++) { /* 34 * 4088 = 138,992 B > 131,072 B */
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, 5000ULL + (uint64_t)i * 10000ULL, &big_bh, big_s) == 0);
        ASSERT_EQ_U32(log.state, HKS_FLASH_STATE_RECORDING);
    }
    ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, 500000ULL) == 0);
    ASSERT_EQ_U32(log.state, HKS_FLASH_STATE_IDLE);

    verify_session_stream_and_footer(&log, 20u, HKS_END_BUTTON_STOP, 0u, 0u);
    verify_session_stream_and_footer(&log, 11u, HKS_END_BUTTON_STOP, 0u, 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);
}

static void test_multipage_flash_full_headroom_and_dropped_counters(void) {
    /* Subtest A: 3,104 B (N=512) HIGH_G_BATCH at cur_pg=62, page_fill=1000 on 1-block ring */
    hks_hal_mock_reset();
    for (uint16_t b = 7u; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        hks_hal_mock_inject_bad_block(b, false);
    }

    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);
    hks_session_header_t sh;
    init_test_sh(&sh, 30u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 1000ULL) == 0);

    /* Target byte offset: 62 * 2048 + 1000 = 127,976 B.
     * Currently written: 104 + 312 = 416 B. Need 127,560 B more. */
    hks_imu_sample_t imu_s[338];
    memset(imu_s, 0, sizeof(imu_s));
    hks_imu_batch_hdr_t imu_bh = { .sample_count = 338u, .odr_hz = 200u, .temp_c_x100 = 2000, .reserved = 0u };
    /* 31 frames of 4088 B = 126,728 B -> remaining to reach 127,976 is 832 B */
    for (int i = 0; i < 31; i++) {
        ASSERT_TRUE(hks_flash_log_append_imu_batch(&log, 0u, 2000ULL + (uint64_t)i * 1000ULL, &imu_bh, imu_s) == 0);
    }
    /* Remaining 832 B: one 832 B IMU batch (832 - 32 = 800 B -> 800 / 12 = 66.67).
     * Use 16 AUX_ENV frames (16 * 52 = 832 B)! */
    hks_aux_env_t aux;
    memset(&aux, 0, sizeof(aux));
    for (int i = 0; i < 16; i++) {
        ASSERT_TRUE(hks_flash_log_append_aux_env(&log, 0u, 40000ULL + (uint64_t)i * 100ULL, &aux) == 0);
    }
    ASSERT_EQ_U32(log.rec_cur_page, 62u);
    ASSERT_EQ_U32(log.page_fill, 1000u);

    /* Append 3,104 B (N=512) HIGH_G_BATCH: 3104 + 96 = 3200 > 3096 remaining in block */
    hks_high_g_sample_t hg_s[512];
    memset(hg_s, 0, sizeof(hg_s));
    hks_high_g_batch_hdr_t hg_bh = { .sample_count = 512u, .odr_hz = 3200u, .trigger_mode = HKS_HIGH_G_IMPACT_BURST, .reserved0 = 0u, .reserved1 = 0u };
    ASSERT_TRUE(hks_flash_log_append_high_g_batch(&log, HKS_FLAG_HIGH_G_ACTIVE, 50000ULL, &hg_bh, hg_s) == -2);
    ASSERT_EQ_U32(log.state, HKS_FLASH_STATE_IDLE);
    verify_session_stream_and_footer(&log, 30u, HKS_END_FLASH_FULL, 0u, 512u);

    /* Subtest B: 0x02 IMU_BATCH FLASH_FULL reports dropped_imu_samples == 20 */
    hks_hal_mock_reset();
    for (uint16_t b = 7u; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        hks_hal_mock_inject_bad_block(b, false);
    }
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);
    init_test_sh(&sh, 31u);
    ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, 1000ULL) == 0);

    hks_imu_batch_hdr_t bh20 = { .sample_count = 20u, .odr_hz = 200u, .temp_c_x100 = 2000, .reserved = 0u };
    int rc = 0;
    uint64_t ts = 2000ULL;
    while (rc == 0) {
        rc = hks_flash_log_append_imu_batch(&log, 0u, ts, &bh20, imu_s);
        ts += 5000ULL;
    }
    ASSERT_TRUE(rc == -2);
    ASSERT_EQ_U32(log.state, HKS_FLASH_STATE_IDLE);
    verify_session_stream_and_footer(&log, 31u, HKS_END_FLASH_FULL, 20u, 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);
}

static void test_session_index_compaction_64_sessions_and_page63_reboot(void) {
    hks_hal_mock_reset();
    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_cal_block_t cal;
    hks_cal_block_init_default(&cal, 1000ULL);
    hks_session_header_t sh;

    /* Fill all 64 active sessions (128 index records = 2 index blocks) */
    for (uint64_t sid = 1u; sid <= HKS_FLASH_MAX_SESSIONS; sid++) {
        init_test_sh(&sh, sid);
        ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, sid * 1000ULL) == 0);
        ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, sid * 1000ULL + 1000ULL) == 0);
    }
    ASSERT_EQ_U32(log.session_count, HKS_FLASH_MAX_SESSIONS);

    /* Verify all 64 sessions -> wraps across all 4 SESSION_INDEX blocks (2..5) with 64 active! */
    for (uint64_t sid = 1u; sid <= HKS_FLASH_MAX_SESSIONS; sid++) {
        verify_session_stream_and_footer(&log, sid, HKS_END_BUTTON_STOP, 0u, 0u);
    }

    /* Drive index_cur_page to 64 (last.page == 63) by doing verify/delete+re-create cycles */
    uint64_t next_sid = 100u;
    while (log.index_cur_page != 64u) {
        const uint64_t victim = (next_sid - 100u) % HKS_FLASH_MAX_SESSIONS + 1u;
        uint32_t freed = 0u;
        ASSERT_TRUE(hks_flash_log_delete_session(&log, victim, true, &freed) == HKS_STATUS_OK);
        init_test_sh(&sh, next_sid);
        ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, next_sid * 1000ULL) == 0);
        ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, next_sid * 1000ULL + 1000ULL) == 0);
        /* Replace back with original sid? No, keep 64 active sessions while advancing index_cur_page */
        ASSERT_TRUE(hks_flash_log_delete_session(&log, next_sid, true, &freed) == HKS_STATUS_OK);
        init_test_sh(&sh, victim);
        ASSERT_TRUE(hks_flash_log_start_session(&log, &sh, &cal, 0u, next_sid * 1000ULL + 500ULL) == 0);
        ASSERT_TRUE(hks_flash_log_stop_session(&log, HKS_END_BUTTON_STOP, 0u, next_sid * 1000ULL + 1500ULL) == 0);
        next_sid++;
        if (log.index_cur_page >= HKS_NAND_PAGES_PER_BLOCK) {
            break;
        }
    }

    /* Reboot right at page == 63 (index_cur_page == 64), then update & wrap further */
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);
    ASSERT_EQ_U32(log.session_count, HKS_FLASH_MAX_SESSIONS);

    for (uint64_t sid = 1u; sid <= HKS_FLASH_MAX_SESSIONS; sid++) {
        verify_session_stream_and_footer(&log, sid, HKS_END_BUTTON_STOP, 0u, 0u);
    }

    /* Reboot once more and confirm all 64 sessions survive intact */
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);
    ASSERT_EQ_U32(log.session_count, HKS_FLASH_MAX_SESSIONS);
    hks_session_list_entry_t list[HKS_FLASH_MAX_SESSIONS];
    size_t cnt = 0u;
    ASSERT_TRUE(hks_flash_log_list_sessions(&log, list, HKS_FLASH_MAX_SESSIONS, &cnt) == HKS_STATUS_OK);
    ASSERT_EQ_U32(cnt, HKS_FLASH_MAX_SESSIONS);
    ASSERT_EQ_U32(hks_hal_mock_get_nop_violations(), 0u);
    ASSERT_EQ_U32(hks_hal_mock_get_bit_violations(), 0u);
}

static void test_protocol_oversized_resync_and_no_memcpy_overlap(void) {
    hks_hal_mock_reset();
    static hks_flash_log_t log;
    ASSERT_TRUE(hks_flash_log_init(&log) == 0);

    hks_protocol_t proto;
    hks_protocol_init(&proto, &log, 0x01020304u, 0x0100u);

    /* Build oversized command (payload_len = 129) containing embedded valid LIST_SESSIONS frame */
    uint8_t embedded_list[16];
    const size_t elen = hks_protocol_encode_cmd(embedded_list, sizeof(embedded_list),
                                                HKS_CMD_LIST_SESSIONS, 77u, NULL, 0u);
    ASSERT_EQ_U32(elen, 10u);

    uint8_t oversize_pl[129];
    memset(oversize_pl, 0x22, sizeof(oversize_pl));
    memcpy(oversize_pl + 10u, embedded_list, elen);

    uint8_t oversize_cmd[256];
    const size_t olen = hks_protocol_encode_cmd(oversize_cmd, sizeof(oversize_cmd),
                                                HKS_CMD_GET_HEADER, 9u, oversize_pl, 129u);
    ASSERT_EQ_U32(olen, 6u + 129u + 4u);

    uint8_t rsp[HKS_PROTO_MAX_TX_FRAME];
    const size_t rlen = hks_protocol_feed_bytes(&proto, oversize_cmd, olen, rsp, sizeof(rsp));
    ASSERT_EQ_U32(rlen, HKS_PROTO_RSP_OVERHEAD_BYTES); /* ONLY 12 B ERR_INVALID_PARAM */
    ASSERT_EQ_U32(rsp[2], HKS_CMD_GET_HEADER);
    ASSERT_EQ_U32(rsp[3], HKS_STATUS_ERR_INVALID_PARAM);
    ASSERT_EQ_U32(rsp[4], 9u);

    /* Next valid command immediately after drained oversized frame succeeds */
    size_t rlen2 = 0u;
    send_cmd_expect(&proto, HKS_CMD_GET_DEVICE_STATUS, 10u, NULL, 0u, HKS_STATUS_OK, rsp, &rlen2);

    /* Also exercise hks_protocol_poll_usb() (tx_buf output path under ASan/UBSan) */
    uint8_t cmd_tx[32];
    const size_t clen = hks_protocol_encode_cmd(cmd_tx, sizeof(cmd_tx), HKS_CMD_LIST_SESSIONS, 11u, NULL, 0u);
    ASSERT_EQ_U32(hks_hal_mock_usb_push_rx(cmd_tx, clen), clen);
    ASSERT_TRUE(hks_protocol_poll_usb(&proto) >= HKS_PROTO_RSP_OVERHEAD_BYTES);
    uint8_t usb_out[64];
    const size_t ulen = hks_hal_mock_usb_pop_tx(usb_out, sizeof(usb_out));
    ASSERT_TRUE(ulen >= HKS_PROTO_RSP_OVERHEAD_BYTES);
    ASSERT_EQ_U32(usb_out[3], HKS_STATUS_OK);
}

static void test_attitude_and_contract_edge_cases(void) {
    /* 1. Invalid msg_type rejection in hks_frame_decode_hdr */
    uint8_t bad_frame[32];
    ASSERT_TRUE(hks_frame_encode(bad_frame, sizeof(bad_frame), 0x00u, 0u, 0u, 1000ULL, NULL, 0u) == 24u);
    hks_frame_hdr_t fh;
    ASSERT_TRUE(!hks_frame_decode_hdr(bad_frame, sizeof(bad_frame), &fh));
    ASSERT_TRUE(hks_frame_encode(bad_frame, sizeof(bad_frame), 0x08u, 0u, 0u, 1000ULL, NULL, 0u) == 24u);
    ASSERT_TRUE(!hks_frame_decode_hdr(bad_frame, sizeof(bad_frame), &fh));

    /* 2. +Inf / -Inf / NaN quaternion normalization resets cleanly to [1,0,0,0] */
    float q_inf[4] = { INFINITY, 0.0f, 0.0f, 0.0f };
    hks_quat_normalize(q_inf);
    ASSERT_NEAR(q_inf[0], 1.0f, 1e-6);
    ASSERT_NEAR(q_inf[1], 0.0f, 1e-6);
    ASSERT_NEAR(q_inf[2], 0.0f, 1e-6);
    ASSERT_NEAR(q_inf[3], 0.0f, 1e-6);

    /* 3. Stale last_high_g_b must NOT be reused on subsequent > 14.5 g event with hg_sample_opt == NULL */
    hks_attitude_t att;
    hks_attitude_init(&att, NULL);
    hks_attitude_step_out_t out;
    const int16_t icm_146 = (int16_t)lrintf(14.6f * 2048.0f);
    const int16_t icm_119 = (int16_t)lrintf(11.9f * 2048.0f);
    hks_imu_sample_t s146 = { .ax = 2048, .ay = 0, .az = icm_146, .gx = 0, .gy = 0, .gz = 0 };
    hks_imu_sample_t s119 = { .ax = 2048, .ay = 0, .az = icm_119, .gx = 0, .gy = 0, .gz = 0 };
    hks_high_g_sample_t hg200 = { .ax = 200, .ay = 0, .az = 0 }; /* +Z_B = +196.133 m/s^2 */

    hks_attitude_step(&att, &s146, &hg200, 5000ULL, 0.005f, &out);
    ASSERT_TRUE(att.high_g_active);
    ASSERT_NEAR(att.f_b[2], 200.0f * HKS_HIGH_G_SCALE_MPS2_PER_LSB, 1e-2);

    for (int k = 0; k < 4; k++) {
        hks_attitude_step(&att, &s119, NULL, 10000ULL + (uint64_t)k * 5000ULL, 0.005f, &out);
    }
    ASSERT_TRUE(!att.high_g_active);
    ASSERT_TRUE(!att.has_high_g_sample);

    /* New > 14.5 g event with hg_sample_opt == NULL must use live ICM vector (+X_B ~ 143.18 m/s^2), not stale hg200 */
    hks_attitude_step(&att, &s146, NULL, 35000ULL, 0.005f, &out);
    ASSERT_TRUE(att.high_g_active);
    ASSERT_NEAR(att.f_b[0], (float)icm_146 * HKS_ACCEL_SCALE_MPS2_PER_LSB, 1e-2);
    ASSERT_NEAR(att.f_b[2], HKS_GRAVITY_MPS2_F, 1e-2);

    /* 4. 1.5 s burst at 0.50 g RMS must NOT falsely emit SHIFT_HINT_ON; >= 2.0 s skating MUST emit */
    hks_attitude_init(&att, NULL);
    hks_imu_sample_t upright = { .ax = 2048, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0 };
    uint64_t ts = 0ULL;
    bool saw_false_on = false;
    for (int k = 0; k < 400; k++) { /* 2.0 s quiet bench */
        ts += 5000ULL;
        hks_attitude_step(&att, &upright, NULL, ts, 0.005f, &out);
    }
    for (int k = 0; k < 300; k++) { /* 1.5 s burst at 0.50 g RMS 2.0 Hz */
        const float t = (float)k * 0.005f;
        const float dyn = 0.50f * 1.41421356f * HKS_GRAVITY_MPS2_F * sinf(2.0f * 3.14159265f * 2.0f * t);
        hks_imu_sample_t burst = {
            .ax = 2048,
            .ay = (int16_t)lrintf(dyn / HKS_ACCEL_SCALE_MPS2_PER_LSB),
            .az = 0, .gx = 0, .gy = 0, .gz = 0
        };
        ts += 5000ULL;
        hks_attitude_step(&att, &burst, NULL, ts, 0.005f, &out);
        if (out.emit_shift_hint && out.shift_event.event_code == HKS_EVT_SHIFT_HINT_ON) {
            saw_false_on = true;
        }
    }
    for (int k = 0; k < 600; k++) { /* 3.0 s quiet bench */
        ts += 5000ULL;
        hks_attitude_step(&att, &upright, NULL, ts, 0.005f, &out);
        if (out.emit_shift_hint && out.shift_event.event_code == HKS_EVT_SHIFT_HINT_ON) {
            saw_false_on = true;
        }
    }
    ASSERT_TRUE(!saw_false_on);

    /* 5. >= 2.0 s skating (2.25 s at 0.25 g RMS 1.5 Hz) emits SHIFT_HINT_ON */
    bool saw_real_on = false;
    for (int k = 0; k < 500; k++) {
        const float t = (float)k * 0.005f;
        const float dyn = 0.25f * 1.41421356f * HKS_GRAVITY_MPS2_F * sinf(2.0f * 3.14159265f * 1.5f * t);
        hks_imu_sample_t skate = {
            .ax = 2048,
            .ay = (int16_t)lrintf(dyn / HKS_ACCEL_SCALE_MPS2_PER_LSB),
            .az = 0, .gx = 0, .gy = 0, .gz = 0
        };
        ts += 5000ULL;
        hks_attitude_step(&att, &skate, NULL, ts, 0.005f, &out);
        if (out.emit_shift_hint && out.shift_event.event_code == HKS_EVT_SHIFT_HINT_ON) {
            saw_real_on = true;
        }
    }
    ASSERT_TRUE(saw_real_on);

    /* 6. Mahony ki gating prevents integral windup when ||err|| > 0.15 rad */
    hks_attitude_init(&att, NULL);
    const float deg15 = 15.0f * 3.14159265f / 180.0f;
    const float f_s_15[3] = { HKS_GRAVITY_MPS2_F * cosf(deg15), 0.0f, -HKS_GRAVITY_MPS2_F * sinf(deg15) };
    hks_imu_sample_t tilt15 = {
        .ax = (int16_t)lrintf(f_s_15[0] / HKS_ACCEL_SCALE_MPS2_PER_LSB),
        .ay = 0,
        .az = (int16_t)lrintf(f_s_15[2] / HKS_ACCEL_SCALE_MPS2_PER_LSB),
        .gx = 0, .gy = 0, .gz = 0
    };
    for (int k = 0; k < 100; k++) { /* first 0.5 s while err_norm > 0.15 rad: ki gated */
        hks_attitude_step(&att, &tilt15, NULL, (uint64_t)(k + 1) * 5000ULL, 0.005f, &out);
    }
    ASSERT_NEAR(att.integral_err_b[1], 0.0f, 1e-6);
    for (int k = 100; k < 2000; k++) { /* settle to 15 deg pitch */
        hks_attitude_step(&att, &tilt15, NULL, (uint64_t)(k + 1) * 5000ULL, 0.005f, &out);
    }
    ASSERT_TRUE(fabsf(att.integral_err_b[1]) < 0.003f);
    ASSERT_NEAR(att.a_lin_w[0], 0.0f, 0.05);
    ASSERT_NEAR(att.a_lin_w[2], 0.0f, 0.05);

    /* 7. U4 (ADXL372) DNP mode: hks_hal_highg_is_present() == false, read_fifo returns 0 samples,
     * and hks_attitude_step() tracks >12 g impacts cleanly using U3 alone. */
    hks_hal_mock_reset();
    hks_hal_mock_set_highg_present(false);
    ASSERT_TRUE(!hks_hal_highg_is_present());
    const int16_t dummy_lj = (int16_t)(400 << 4);
    hks_hal_mock_push_highg_raw16(&dummy_lj, &dummy_lj, &dummy_lj, 1u);
    hks_high_g_sample_t hg_dnp_out[4];
    size_t hg_dnp_cnt = 99u;
    ASSERT_TRUE(hks_hal_highg_read_fifo(hg_dnp_out, 4u, &hg_dnp_cnt) == 0);
    ASSERT_EQ_U32(hg_dnp_cnt, 0u);
    hks_attitude_init(&att, NULL);
    hks_attitude_step(&att, &s146, NULL, 10000ULL, 0.005f, &out);
    ASSERT_TRUE(att.high_g_active);
    ASSERT_TRUE(!att.has_high_g_sample);
    hks_attitude_step(&att, &upright, NULL, 270000ULL, 0.005f, &out);
    ASSERT_TRUE(out.emit_impact);
    ASSERT_EQ_U32(out.impact_event.event_code, HKS_EVT_IMPACT_SPIKE);
}

int main(void) {
    printf("Running hockey_sensor M3 firmware test suite...\n");
    RUN_TEST(test_crc32_and_sha256_vectors);
    RUN_TEST(test_adxl372_unpack_and_misaligned_serdes);
    RUN_TEST(test_nand_nop4_and_bad_block_skip);
    RUN_TEST(test_watchdog_recover_torn_frame);
    RUN_TEST(test_usb_protocol_all_commands_and_10_statuses);
    RUN_TEST(test_ring_reclamation_verified_vs_unverified);
    RUN_TEST(test_r_bs_dual_imu_attitude_and_detectors);
    RUN_TEST(test_noncontiguous_session_block_chain_reboot);
    RUN_TEST(test_mid_session_ring_reclamation_while_recording);
    RUN_TEST(test_multipage_flash_full_headroom_and_dropped_counters);
    RUN_TEST(test_session_index_compaction_64_sessions_and_page63_reboot);
    RUN_TEST(test_protocol_oversized_resync_and_no_memcpy_overlap);
    RUN_TEST(test_attitude_and_contract_edge_cases);
    printf("ALL %d M3 TESTS PASSED\n", g_tests_run);
    return 0;
}
