#ifndef HKS_CONTRACT_H
#define HKS_CONTRACT_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * ABI & Contract Constants (INTERFACE_CONTRACT.json v1.0.0)
 * ========================================================================= */

#define HKS_CONTRACT_VERSION_MAJOR 1u
#define HKS_CONTRACT_VERSION_MINOR 0u
#define HKS_CONTRACT_VERSION_PATCH 0u

#define HKS_SYNC_MARKER            0xAA55u
#define HKS_RSP_SYNC_MARKER        0x55AAu
#define HKS_SESSION_MAGIC_0        'H'
#define HKS_SESSION_MAGIC_1        'K'
#define HKS_SESSION_MAGIC_2        'S'
#define HKS_SESSION_MAGIC_3        '1'
#define HKS_FORMAT_VERSION         1u
#define HKS_HW_REV_DEFAULT         0x0100u
#define HKS_FW_VERSION_DEFAULT     0x01000000u

#define HKS_GRAVITY_MPS2           9.80665
#define HKS_GRAVITY_MPS2_F         9.80665f

/* Exact sensor scales locked in INTERFACE_CONTRACT.json & docs/architecture.md */
#define HKS_ACCEL_SCALE_MPS2_PER_LSB   (9.80665f / 2048.0f)
#define HKS_GYRO_SCALE_RADS_PER_LSB    0.001064225153655079f
#define HKS_HIGH_G_SCALE_MPS2_PER_LSB  0.980665f
#define HKS_MAG_SCALE_UT_PER_LSB       (100.0f / 16384.0f)
#define HKS_PRESSURE_PA_PER_LSB        0.015625

/* Binary flash frame dimensions */
#define HKS_FRAME_HDR_BYTES            20u
#define HKS_FRAME_CRC_BYTES            4u
#define HKS_FRAME_OVERHEAD_BYTES       (HKS_FRAME_HDR_BYTES + HKS_FRAME_CRC_BYTES)
#define HKS_MAX_PAYLOAD_BYTES          4072u
#define HKS_MAX_FRAME_BYTES            4096u

/* Exact wire payload sizes */
#define HKS_PAYLOAD_SESSION_HEADER_BYTES   80u
#define HKS_FRAME_SESSION_HEADER_BYTES     (HKS_FRAME_OVERHEAD_BYTES + HKS_PAYLOAD_SESSION_HEADER_BYTES)

#define HKS_PAYLOAD_IMU_BATCH_HDR_BYTES    8u
#define HKS_IMU_SAMPLE_BYTES               12u
#define HKS_IMU_MAX_SAMPLES                338u

#define HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES 8u
#define HKS_HIGH_G_SAMPLE_BYTES            6u
#define HKS_HIGH_G_MAX_SAMPLES             677u

#define HKS_PAYLOAD_AUX_ENV_BYTES          28u
#define HKS_FRAME_AUX_ENV_BYTES            (HKS_FRAME_OVERHEAD_BYTES + HKS_PAYLOAD_AUX_ENV_BYTES)

#define HKS_PAYLOAD_EVENT_MARKER_BYTES     16u
#define HKS_FRAME_EVENT_MARKER_BYTES       (HKS_FRAME_OVERHEAD_BYTES + HKS_PAYLOAD_EVENT_MARKER_BYTES)

#define HKS_PAYLOAD_CAL_BLOCK_BYTES        288u
#define HKS_CAL_BLOCK_CRC_BYTES            284u
#define HKS_FRAME_CAL_BLOCK_BYTES          (HKS_FRAME_OVERHEAD_BYTES + HKS_PAYLOAD_CAL_BLOCK_BYTES)

#define HKS_PAYLOAD_SESSION_FOOTER_BYTES   72u
#define HKS_FRAME_SESSION_FOOTER_BYTES     (HKS_FRAME_OVERHEAD_BYTES + HKS_PAYLOAD_SESSION_FOOTER_BYTES)

/* USB CDC-ACM / BLE download protocol sizes */
#define HKS_PROTO_CMD_HDR_BYTES            6u
#define HKS_PROTO_CMD_OVERHEAD_BYTES       10u
#define HKS_PROTO_RSP_HDR_BYTES            8u
#define HKS_PROTO_RSP_OVERHEAD_BYTES       12u
#define HKS_PROTO_MAX_CHUNK_BYTES          4096u

#define HKS_LIST_SESSIONS_HDR_BYTES        4u
#define HKS_LIST_SESSIONS_ENTRY_BYTES      32u
#define HKS_GET_HEADER_REQ_BYTES           8u
#define HKS_GET_HEADER_RSP_BYTES           (HKS_FRAME_SESSION_HEADER_BYTES + HKS_FRAME_CAL_BLOCK_BYTES)
#define HKS_READ_CHUNK_REQ_BYTES           14u
#define HKS_READ_CHUNK_RSP_HDR_BYTES       6u
#define HKS_ACK_VERIFY_REQ_BYTES           44u
#define HKS_ACK_VERIFY_REQ_MIN_BYTES       12u
#define HKS_ACK_VERIFY_RSP_BYTES           2u
#define HKS_DELETE_SESSION_REQ_BYTES       9u
#define HKS_DELETE_SESSION_RSP_BYTES       4u
#define HKS_GET_DEVICE_STATUS_RSP_BYTES    24u
#define HKS_SET_UTC_TIME_BYTES             8u

/* W25N01JW 1 Gbit (128 MiB) SLC QSPI NAND geometry & partition map */
#define HKS_NAND_PAGE_BYTES                2048u
#define HKS_NAND_OOB_BYTES                 64u
#define HKS_NAND_PAGES_PER_BLOCK           64u
#define HKS_NAND_BLOCK_BYTES               (HKS_NAND_PAGE_BYTES * HKS_NAND_PAGES_PER_BLOCK)
#define HKS_NAND_TOTAL_BLOCKS              1024u
#define HKS_NAND_MAX_NOP                   4u

#define HKS_PART_FACTORY_CAL_START         0u
#define HKS_PART_FACTORY_CAL_BLOCKS        2u    /* Blocks 0..1 */
#define HKS_PART_SESSION_INDEX_START       2u
#define HKS_PART_SESSION_INDEX_BLOCKS      4u    /* Blocks 2..5 */
#define HKS_PART_RING_START                6u
#define HKS_PART_RING_BLOCKS               998u  /* Blocks 6..1003 */
#define HKS_PART_BBM_POOL_START            1004u
#define HKS_PART_BBM_POOL_BLOCKS           20u   /* Blocks 1004..1023 */

/* =========================================================================
 * Hashing Boundary Contract Note (M3 <-> M4 Parity)
 * -------------------------------------------------------------------------
 * 1. 0x07 SESSION_FOOTER.pre_footer_sha256[32]:
 *    Computed incrementally over session bytes [0 .. total_bytes - 96),
 *    covering all frames strictly preceding the 96-byte 0x07 footer frame.
 * 2. 0x04 ACK_VERIFY.computed_sha256[32] & computed_file_crc32:
 *    Computed over the COMPLETE downloaded .hks session [0 .. total_bytes),
 *    including the 96-byte 0x07 footer frame.
 * ========================================================================= */

/* =========================================================================
 * Enumerations
 * ========================================================================= */

typedef enum {
    HKS_MSG_SESSION_HEADER    = 0x01,
    HKS_MSG_IMU_BATCH         = 0x02,
    HKS_MSG_HIGH_G_BATCH      = 0x03,
    HKS_MSG_AUX_ENV           = 0x04,
    HKS_MSG_EVENT_MARKER      = 0x05,
    HKS_MSG_CALIBRATION_BLOCK = 0x06,
    HKS_MSG_SESSION_FOOTER    = 0x07
} hks_msg_type_t;

typedef enum {
    HKS_FLAG_ACCEL_SAT     = (1u << 0),
    HKS_FLAG_GYRO_SAT      = (1u << 1),
    HKS_FLAG_MAG_ANOM      = (1u << 2),
    HKS_FLAG_LOW_BATT      = (1u << 3),
    HKS_FLAG_HIGH_G_ACTIVE = (1u << 4),
    HKS_FLAG_UTC_SYNCED    = (1u << 5)
} hks_frame_flags_t;

typedef enum {
    HKS_RINK_NHL_200X85FT = 0,
    HKS_RINK_IIHF_60X30M  = 1,
    HKS_RINK_CUSTOM       = 2
} hks_rink_preset_t;

typedef enum {
    HKS_MOUNT_STERNUM_POCKET = 0,
    HKS_MOUNT_CHEST_STRAP    = 1
} hks_mount_location_t;

typedef enum {
    HKS_HIGH_G_CONTINUOUS   = 0,
    HKS_HIGH_G_IMPACT_BURST = 1
} hks_high_g_trigger_t;

typedef enum {
    HKS_AUX_MAG_FRESH     = (1u << 0),
    HKS_AUX_BARO_FRESH    = (1u << 1),
    HKS_AUX_PMIC_FRESH    = (1u << 2),
    HKS_AUX_MAG_SET_RESET = (1u << 3)
} hks_aux_validity_t;

typedef enum {
    HKS_EVT_SESSION_START   = 1,
    HKS_EVT_BUTTON_MARK     = 2,
    HKS_EVT_IMPACT_SPIKE    = 3,
    HKS_EVT_SHIFT_HINT_ON   = 4,
    HKS_EVT_SHIFT_HINT_OFF  = 5,
    HKS_EVT_LOW_BATT        = 6,
    HKS_EVT_BOARD_COLLISION = 7,
    HKS_EVT_SESSION_END     = 8
} hks_event_code_t;

typedef enum {
    HKS_SEV_INFO     = 0,
    HKS_SEV_NOTICE   = 1,
    HKS_SEV_WARNING  = 2,
    HKS_SEV_CRITICAL = 3
} hks_event_severity_t;

typedef enum {
    HKS_END_BUTTON_STOP      = 0,
    HKS_END_USB_DOCK         = 1,
    HKS_END_LOW_BATT         = 2,
    HKS_END_FLASH_FULL       = 3,
    HKS_END_WATCHDOG_RECOVER = 4
} hks_end_reason_t;

typedef enum {
    HKS_CMD_LIST_SESSIONS     = 0x01,
    HKS_CMD_GET_HEADER        = 0x02,
    HKS_CMD_READ_CHUNK        = 0x03,
    HKS_CMD_ACK_VERIFY        = 0x04,
    HKS_CMD_DELETE_SESSION    = 0x05,
    HKS_CMD_GET_DEVICE_STATUS = 0x06,
    HKS_CMD_SET_UTC_TIME      = 0x07
} hks_cmd_id_t;

/* Authoritative status codes from INTERFACE_CONTRACT.json (0x00..0x09) */
typedef enum {
    HKS_STATUS_OK                   = 0x00,
    HKS_STATUS_ERR_BAD_CRC          = 0x01,
    HKS_STATUS_ERR_UNKNOWN_CMD      = 0x02,
    HKS_STATUS_ERR_SESSION_NOT_FOUND = 0x03,
    HKS_STATUS_ERR_OFFSET_OUT_OF_RANGE = 0x04,
    HKS_STATUS_ERR_NOT_VERIFIED     = 0x05,
    HKS_STATUS_ERR_FLASH_IO         = 0x06,
    HKS_STATUS_ERR_BUSY_RECORDING   = 0x07,
    HKS_STATUS_ERR_VERIFY_MISMATCH  = 0x08,
    HKS_STATUS_ERR_INVALID_PARAM    = 0x09
} hks_status_code_t;

/* nPM1300 primary status flags in bits [7:0] (shared by AUX_ENV and GET_DEVICE_STATUS) */
typedef enum {
    HKS_PMIC_VBUS_PRESENT = (1u << 0),
    HKS_PMIC_CHARGING     = (1u << 1),
    HKS_PMIC_CHARGE_DONE  = (1u << 2),
    HKS_PMIC_NTC_FAULT    = (1u << 3),
    HKS_PMIC_LOW_BATT     = (1u << 4),
    HKS_PMIC_THERM_WARN   = (1u << 5),
    HKS_PMIC_FG_VALID     = (1u << 6),
    HKS_PMIC_I2C_ERR      = (1u << 7)
} hks_pmic_status_bits_t;

/* =========================================================================
 * Compile-Time ABI Verification (_Static_assert)
 * ========================================================================= */

_Static_assert(CHAR_BIT == 8, "8-bit byte required");
_Static_assert(sizeof(float) == 4, "IEEE-754 float must be 4 bytes");
_Static_assert(sizeof(double) == 8, "IEEE-754 double must be 8 bytes");
_Static_assert(HKS_FRAME_OVERHEAD_BYTES == 24u, "Frame overhead must be 24 bytes");
_Static_assert(HKS_MAX_FRAME_BYTES == 4096u, "Max frame bytes must be 4096");
_Static_assert(HKS_PAYLOAD_IMU_BATCH_HDR_BYTES + (HKS_IMU_MAX_SAMPLES * HKS_IMU_SAMPLE_BYTES) <= HKS_MAX_PAYLOAD_BYTES,
               "Max IMU batch must fit in max payload");
_Static_assert(HKS_PAYLOAD_IMU_BATCH_HDR_BYTES + ((HKS_IMU_MAX_SAMPLES + 1u) * HKS_IMU_SAMPLE_BYTES) > HKS_MAX_PAYLOAD_BYTES,
               "IMU max sample limit must be tight");
_Static_assert(HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES + (HKS_HIGH_G_MAX_SAMPLES * HKS_HIGH_G_SAMPLE_BYTES) <= HKS_MAX_PAYLOAD_BYTES,
               "Max High-G batch must fit in max payload");
_Static_assert(HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES + ((HKS_HIGH_G_MAX_SAMPLES + 1u) * HKS_HIGH_G_SAMPLE_BYTES) > HKS_MAX_PAYLOAD_BYTES,
               "High-G max sample limit must be tight");
_Static_assert((uint64_t)HKS_PART_RING_BLOCKS * (uint64_t)HKS_NAND_BLOCK_BYTES >= 130000000ULL,
               "TELEMETRY_RING usable capacity must be >= 130 MB");
_Static_assert(HKS_PART_FACTORY_CAL_BLOCKS + HKS_PART_SESSION_INDEX_BLOCKS +
               HKS_PART_RING_BLOCKS + HKS_PART_BBM_POOL_BLOCKS == HKS_NAND_TOTAL_BLOCKS,
               "NAND partition blocks must sum to 1024");

#if defined(__GNUC__) || defined(__clang__)
#define HKS_PACKED_ABI_CHECK __attribute__((packed))
typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t sync;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t payload_len;
    uint16_t seq;
    uint64_t timestamp_us;
    uint32_t reserved;
} hks_wire_frame_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    char magic[4];
    uint16_t format_version;
    uint16_t hw_rev;
    uint32_t fw_version;
    uint32_t cal_crc32;
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint64_t device_id;
    uint16_t imu_odr_hz;
    uint16_t high_g_odr_hz;
    uint16_t mag_odr_hz;
    uint16_t baro_odr_hz;
    float accel_scale_mps2_per_lsb;
    float gyro_scale_rads_per_lsb;
    float high_g_scale_mps2_per_lsb;
    float mag_scale_ut_per_lsb;
    uint8_t rink_preset;
    uint8_t mount_location;
    uint16_t player_height_cm;
    float bench_gate_x_m;
    float bench_gate_y_m;
    uint32_t reserved;
} hks_wire_session_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t sample_count;
    uint16_t odr_hz;
    int16_t temp_c_x100;
    uint16_t reserved;
} hks_wire_imu_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} hks_wire_imu_sample_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t sample_count;
    uint16_t odr_hz;
    uint8_t trigger_mode;
    uint8_t reserved0;
    uint16_t reserved1;
} hks_wire_high_g_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    int16_t ax;
    int16_t ay;
    int16_t az;
} hks_wire_high_g_sample_t;

typedef struct HKS_PACKED_ABI_CHECK {
    int32_t mx_raw;
    int32_t my_raw;
    int32_t mz_raw;
    uint32_t pressure_pa_x64;
    int16_t baro_temp_c_x100;
    uint16_t vbatt_mv;
    int16_t batt_temp_c_x100;
    uint16_t soc_pct_x10;
    uint16_t pmic_status;
    uint16_t validity_flags;
} hks_wire_aux_env_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t event_code;
    uint16_t severity;
    float value_f32;
    uint64_t detail_u64;
} hks_wire_event_marker_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t cal_version;
    uint16_t flags;
    uint64_t cal_utc_ms;
    float sensor_to_body_rotation_matrix[9];
    float accel_misalign_scale[9];
    float accel_bias_mps2[3];
    float gyro_misalign_scale[9];
    float gyro_bias_rads[3];
    float high_g_to_body_rotation_matrix[9];
    float high_g_misalign_scale[9];
    float high_g_bias_mps2[3];
    float mag_hard_iron_ut[3];
    float mag_soft_iron_matrix[9];
    float baseline_mag_norm_ut;
    float baseline_mag_dip_deg;
    uint32_t cal_crc32;
} hks_wire_cal_block_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint64_t end_utc_ms;
    uint32_t total_imu_samples;
    uint32_t dropped_imu_samples;
    uint32_t total_high_g_samples;
    uint32_t dropped_high_g_samples;
    uint32_t total_aux_samples;
    uint32_t end_reason;
    uint64_t total_bytes;
    uint8_t pre_footer_sha256[32];
} hks_wire_session_footer_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t sync;
    uint8_t cmd_id;
    uint8_t seq;
    uint16_t payload_len;
} hks_wire_cmd_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint16_t sync;
    uint8_t cmd_id;
    uint8_t status;
    uint8_t seq;
    uint8_t reserved;
    uint16_t payload_len;
} hks_wire_rsp_hdr_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint32_t duration_ms;
    uint32_t byte_size;
    uint32_t sample_count;
    uint32_t flags;
} hks_wire_list_entry_t;

typedef struct HKS_PACKED_ABI_CHECK {
    uint32_t fw_version;
    uint16_t hw_rev;
    uint16_t vbatt_mv;
    uint16_t soc_pct_x10;
    uint8_t recording_state;
    uint8_t pmic_status;
    uint32_t flash_free_kb;
    uint64_t utc_ms;
} hks_wire_dev_status_t;

_Static_assert(sizeof(hks_wire_frame_hdr_t) == HKS_FRAME_HDR_BYTES, "Frame header wire size mismatch");
_Static_assert(sizeof(hks_wire_session_hdr_t) == HKS_PAYLOAD_SESSION_HEADER_BYTES, "0x01 wire size mismatch");
_Static_assert(sizeof(hks_wire_imu_hdr_t) == HKS_PAYLOAD_IMU_BATCH_HDR_BYTES, "0x02 hdr wire size mismatch");
_Static_assert(sizeof(hks_wire_imu_sample_t) == HKS_IMU_SAMPLE_BYTES, "0x02 sample wire size mismatch");
_Static_assert(sizeof(hks_wire_high_g_hdr_t) == HKS_PAYLOAD_HIGH_G_BATCH_HDR_BYTES, "0x03 hdr wire size mismatch");
_Static_assert(sizeof(hks_wire_high_g_sample_t) == HKS_HIGH_G_SAMPLE_BYTES, "0x03 sample wire size mismatch");
_Static_assert(sizeof(hks_wire_aux_env_t) == HKS_PAYLOAD_AUX_ENV_BYTES, "0x04 wire size mismatch");
_Static_assert(sizeof(hks_wire_event_marker_t) == HKS_PAYLOAD_EVENT_MARKER_BYTES, "0x05 wire size mismatch");
_Static_assert(sizeof(hks_wire_cal_block_t) == HKS_PAYLOAD_CAL_BLOCK_BYTES, "0x06 wire size mismatch");
_Static_assert(sizeof(hks_wire_session_footer_t) == HKS_PAYLOAD_SESSION_FOOTER_BYTES, "0x07 wire size mismatch");
_Static_assert(sizeof(hks_wire_cmd_hdr_t) == HKS_PROTO_CMD_HDR_BYTES, "Cmd hdr wire size mismatch");
_Static_assert(sizeof(hks_wire_rsp_hdr_t) == HKS_PROTO_RSP_HDR_BYTES, "Rsp hdr wire size mismatch");
_Static_assert(sizeof(hks_wire_list_entry_t) == HKS_LIST_SESSIONS_ENTRY_BYTES, "LIST_SESSIONS entry wire size mismatch");
_Static_assert(sizeof(hks_wire_dev_status_t) == HKS_GET_DEVICE_STATUS_RSP_BYTES, "GET_DEVICE_STATUS wire size mismatch");
#endif

/* =========================================================================
 * Naturally Aligned In-Memory C11 Data Structures (Never cast packed ptrs!)
 * ========================================================================= */

typedef struct {
    uint16_t sync;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t payload_len;
    uint16_t seq;
    uint64_t timestamp_us;
    uint32_t reserved;
} hks_frame_hdr_t;

typedef struct {
    char magic[4];
    uint16_t format_version;
    uint16_t hw_rev;
    uint32_t fw_version;
    uint32_t cal_crc32;
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint64_t device_id;
    uint16_t imu_odr_hz;
    uint16_t high_g_odr_hz;
    uint16_t mag_odr_hz;
    uint16_t baro_odr_hz;
    float accel_scale_mps2_per_lsb;
    float gyro_scale_rads_per_lsb;
    float high_g_scale_mps2_per_lsb;
    float mag_scale_ut_per_lsb;
    uint8_t rink_preset;
    uint8_t mount_location;
    uint16_t player_height_cm;
    float bench_gate_x_m;
    float bench_gate_y_m;
    uint32_t reserved;
} hks_session_header_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} hks_imu_sample_t;

typedef struct {
    uint16_t sample_count;
    uint16_t odr_hz;
    int16_t temp_c_x100;
    uint16_t reserved;
} hks_imu_batch_hdr_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
} hks_high_g_sample_t;

typedef struct {
    uint16_t sample_count;
    uint16_t odr_hz;
    uint8_t trigger_mode;
    uint8_t reserved0;
    uint16_t reserved1;
} hks_high_g_batch_hdr_t;

typedef struct {
    int32_t mx_raw;
    int32_t my_raw;
    int32_t mz_raw;
    uint32_t pressure_pa_x64;
    int16_t baro_temp_c_x100;
    uint16_t vbatt_mv;
    int16_t batt_temp_c_x100;
    uint16_t soc_pct_x10;
    uint16_t pmic_status;
    uint16_t validity_flags;
} hks_aux_env_t;

typedef struct {
    uint16_t event_code;
    uint16_t severity;
    float value_f32;
    uint64_t detail_u64;
} hks_event_marker_t;

typedef struct {
    uint16_t cal_version;
    uint16_t flags;
    uint64_t cal_utc_ms;
    float sensor_to_body_rotation_matrix[9];
    float accel_misalign_scale[9];
    float accel_bias_mps2[3];
    float gyro_misalign_scale[9];
    float gyro_bias_rads[3];
    float high_g_to_body_rotation_matrix[9];
    float high_g_misalign_scale[9];
    float high_g_bias_mps2[3];
    float mag_hard_iron_ut[3];
    float mag_soft_iron_matrix[9];
    float baseline_mag_norm_ut;
    float baseline_mag_dip_deg;
    uint32_t cal_crc32;
} hks_cal_block_t;

typedef struct {
    uint64_t end_utc_ms;
    uint32_t total_imu_samples;
    uint32_t dropped_imu_samples;
    uint32_t total_high_g_samples;
    uint32_t dropped_high_g_samples;
    uint32_t total_aux_samples;
    uint32_t end_reason;
    uint64_t total_bytes;
    uint8_t pre_footer_sha256[32];
} hks_session_footer_t;

typedef struct {
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint32_t duration_ms;
    uint32_t byte_size;
    uint32_t sample_count;
    uint32_t flags;
} hks_session_list_entry_t;

typedef struct {
    uint32_t fw_version;
    uint16_t hw_rev;
    uint16_t vbatt_mv;
    uint16_t soc_pct_x10;
    uint8_t recording_state;
    uint8_t pmic_status;
    uint32_t flash_free_kb;
    uint64_t utc_ms;
} hks_device_status_t;

/* =========================================================================
 * Portable memcpy-Safe Little-Endian Serdes Helpers
 * ========================================================================= */

static inline void hks_put_u16_le(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void hks_put_u32_le(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline void hks_put_u64_le(uint8_t *dst, uint64_t v) {
    for (size_t i = 0u; i < 8u; i++) {
        dst[i] = (uint8_t)((v >> (8u * i)) & 0xFFu);
    }
}

static inline void hks_put_i16_le(uint8_t *dst, int16_t v) {
    uint16_t u = 0u;
    memcpy(&u, &v, sizeof(u));
    hks_put_u16_le(dst, u);
}

static inline void hks_put_i32_le(uint8_t *dst, int32_t v) {
    uint32_t u = 0u;
    memcpy(&u, &v, sizeof(u));
    hks_put_u32_le(dst, u);
}

static inline void hks_put_f32_le(uint8_t *dst, float v) {
    uint32_t u = 0u;
    memcpy(&u, &v, sizeof(u));
    hks_put_u32_le(dst, u);
}

static inline uint16_t hks_get_u16_le(const uint8_t *src) {
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static inline uint32_t hks_get_u32_le(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static inline uint64_t hks_get_u64_le(const uint8_t *src) {
    uint64_t v = 0u;
    for (size_t i = 0u; i < 8u; i++) {
        v |= ((uint64_t)src[i] << (8u * i));
    }
    return v;
}

static inline int16_t hks_get_i16_le(const uint8_t *src) {
    uint16_t u = hks_get_u16_le(src);
    int16_t v = 0;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static inline int32_t hks_get_i32_le(const uint8_t *src) {
    uint32_t u = hks_get_u32_le(src);
    int32_t v = 0;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static inline float hks_get_f32_le(const uint8_t *src) {
    uint32_t u = hks_get_u32_le(src);
    float v = 0.0f;
    memcpy(&v, &u, sizeof(v));
    return v;
}

/**
 * Portable ISO C11 unpacking of ADXL372 12-bit left-justified two's-complement
 * register/FIFO word (bits [15:4]) into right-aligned sign-extended int16_t
 * in [-2048, +2047] at 0.980665 (m/s^2)/LSB.
 * Avoids implementation-defined signed right-shift in C11.
 */
static inline int16_t hks_adxl372_unpack_i12(uint16_t raw_left_justified) {
    uint16_t u = (uint16_t)(raw_left_justified >> 4);
    if ((u & 0x0800u) != 0u) {
        u |= 0xF000u;
    }
    int16_t val = 0;
    memcpy(&val, &u, sizeof(val));
    return val;
}

/* =========================================================================
 * Frame & Payload Encode / Decode API (src/hks_contract.c)
 * ========================================================================= */

/**
 * Encode a complete .hks frame (20 B header + payload + 4 B CRC32-IEEE).
 * Returns total frame bytes (24 + payload_len), or 0 on error.
 */
size_t hks_frame_encode(uint8_t *out,
                        size_t out_cap,
                        uint8_t msg_type,
                        uint8_t flags,
                        uint16_t seq,
                        uint64_t timestamp_us,
                        const uint8_t *payload,
                        uint16_t payload_len);

/**
 * Decode a 20-byte frame header without assuming alignment.
 * Returns true if sync == 0xAA55 and payload_len <= HKS_MAX_PAYLOAD_BYTES.
 */
bool hks_frame_decode_hdr(const uint8_t *buf, size_t len, hks_frame_hdr_t *hdr);

/**
 * Validate a complete frame in buf[0 .. total_len-1] (checks header + CRC32-IEEE).
 */
bool hks_frame_verify(const uint8_t *buf, size_t len, hks_frame_hdr_t *hdr_out);

/* Typed payload encoders/decoders (return bytes written, or 0 on invalid input) */
size_t hks_encode_session_header(uint8_t *out, size_t out_cap, const hks_session_header_t *hdr);
bool   hks_decode_session_header(const uint8_t *in, size_t in_len, hks_session_header_t *hdr);

size_t hks_encode_imu_batch(uint8_t *out, size_t out_cap,
                            const hks_imu_batch_hdr_t *hdr,
                            const hks_imu_sample_t *samples);
bool   hks_decode_imu_batch(const uint8_t *in, size_t in_len,
                            hks_imu_batch_hdr_t *hdr,
                            hks_imu_sample_t *samples_out,
                            size_t max_samples);

size_t hks_encode_high_g_batch(uint8_t *out, size_t out_cap,
                               const hks_high_g_batch_hdr_t *hdr,
                               const hks_high_g_sample_t *samples);
bool   hks_decode_high_g_batch(const uint8_t *in, size_t in_len,
                               hks_high_g_batch_hdr_t *hdr,
                               hks_high_g_sample_t *samples_out,
                               size_t max_samples);

size_t hks_encode_aux_env(uint8_t *out, size_t out_cap, const hks_aux_env_t *aux);
bool   hks_decode_aux_env(const uint8_t *in, size_t in_len, hks_aux_env_t *aux);

size_t hks_encode_event_marker(uint8_t *out, size_t out_cap, const hks_event_marker_t *evt);
bool   hks_decode_event_marker(const uint8_t *in, size_t in_len, hks_event_marker_t *evt);

size_t hks_encode_cal_block(uint8_t *out, size_t out_cap, const hks_cal_block_t *cal, bool recompute_crc);
bool   hks_decode_cal_block(const uint8_t *in, size_t in_len, hks_cal_block_t *cal, bool verify_crc);
void   hks_cal_block_init_default(hks_cal_block_t *cal, uint64_t cal_utc_ms);

size_t hks_encode_session_footer(uint8_t *out, size_t out_cap, const hks_session_footer_t *ftr);
bool   hks_decode_session_footer(const uint8_t *in, size_t in_len, hks_session_footer_t *ftr);

/* Download protocol helper codecs */
size_t hks_encode_list_entry(uint8_t *out, size_t out_cap, const hks_session_list_entry_t *entry);
size_t hks_encode_device_status(uint8_t *out, size_t out_cap, const hks_device_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* HKS_CONTRACT_H */
