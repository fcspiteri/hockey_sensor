#ifndef HKS_FLASH_LOG_H
#define HKS_FLASH_LOG_H

#include "hks_contract.h"
#include "hks_sha256.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HKS_FLASH_MAX_SESSIONS 64u

typedef enum {
    HKS_FLASH_STATE_UNINIT    = 0,
    HKS_FLASH_STATE_IDLE      = 1,
    HKS_FLASH_STATE_RECORDING = 2,
    HKS_FLASH_STATE_ERROR     = 3
} hks_flash_state_t;

typedef enum {
    HKS_SESS_FLAG_CLOSED    = (1u << 0),
    HKS_SESS_FLAG_VERIFIED  = (1u << 1),
    HKS_SESS_FLAG_RECOVERED = (1u << 2)
} hks_session_index_flags_t;

typedef struct {
    bool     valid;
    bool     closed;
    bool     verified;
    bool     recovered;
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint32_t duration_ms;
    uint32_t byte_size;
    uint32_t sample_count;
    uint32_t session_flags;
    uint16_t start_block;
    uint16_t block_count;
    uint8_t  whole_file_sha256[HKS_SHA256_DIGEST_BYTES];
} hks_flash_session_meta_t;

typedef struct {
    hks_flash_state_t state;
    bool     bad_blocks[HKS_NAND_TOTAL_BLOCKS];
    uint16_t bad_block_count;

    /* Calibration loaded/stored in FACTORY_CAL (blocks 0..1) */
    hks_cal_block_t active_cal;
    bool            has_cal;

    /* SESSION_INDEX (blocks 2..5) append-only page journal state */
    uint32_t index_seq;
    uint16_t index_cur_block;
    uint8_t  index_cur_page;

    /* Known sessions in TELEMETRY_RING (blocks 6..1003) */
    hks_flash_session_meta_t sessions[HKS_FLASH_MAX_SESSIONS];
    size_t                   session_count;
    uint16_t                 ring_cursor_block;

    /* Explicit ordered physical block chain tracking */
    uint16_t block_next[HKS_NAND_TOTAL_BLOCKS];
    uint8_t  block_owner[HKS_NAND_TOTAL_BLOCKS]; /* 0xFF = free, 0..63 = slot, 0xFE = reserved */
    uint16_t session_tail_block[HKS_FLASH_MAX_SESSIONS];
    bool     index_has_unckpt_ops;

    /* Active recording state */
    uint8_t          rec_session_slot;
    uint16_t         rec_reserved_block;
    uint64_t         rec_session_id;
    uint64_t         rec_start_utc_ms;
    uint64_t         rec_start_ts_us;
    uint64_t         rec_last_ts_us;
    uint16_t         rec_seq;
    uint16_t         rec_start_block;
    uint16_t         rec_cur_block;
    uint8_t          rec_cur_page;
    uint16_t         rec_block_count;
    uint32_t         rec_bytes_written;
    uint32_t         rec_imu_samples;
    uint32_t         rec_dropped_imu_samples;
    uint32_t         rec_high_g_samples;
    uint32_t         rec_dropped_high_g_samples;
    uint32_t         rec_aux_samples;
    uint8_t          rec_last_flags;
    uint8_t          page_buf[HKS_NAND_PAGE_BYTES];
    size_t           page_fill;
    hks_sha256_ctx_t pre_footer_sha_ctx;
    uint32_t         file_crc_state;
} hks_flash_log_t;

/* Initialize NAND, scan BBM markers, load FACTORY_CAL, replay SESSION_INDEX,
 * and recover any unclosed OPEN session with WATCHDOG_RECOVER. */
int hks_flash_log_init(hks_flash_log_t *log);

/* Calibration persistence in FACTORY_CAL (blocks 0..1, Slot A/B) */
int hks_flash_log_write_factory_cal(hks_flash_log_t *log, const hks_cal_block_t *cal);
int hks_flash_log_read_factory_cal(hks_flash_log_t *log, hks_cal_block_t *cal_out);

/* Session recording lifecycle */
int hks_flash_log_start_session(hks_flash_log_t *log,
                                const hks_session_header_t *hdr,
                                const hks_cal_block_t *cal_opt,
                                uint8_t frame_flags,
                                uint64_t start_ts_us);

int hks_flash_log_append_frame(hks_flash_log_t *log,
                               uint8_t msg_type,
                               uint8_t flags,
                               uint64_t timestamp_us,
                               const uint8_t *payload,
                               uint16_t payload_len);

int hks_flash_log_append_imu_batch(hks_flash_log_t *log,
                                   uint8_t flags,
                                   uint64_t timestamp_us,
                                   const hks_imu_batch_hdr_t *batch_hdr,
                                   const hks_imu_sample_t *samples);

int hks_flash_log_append_high_g_batch(hks_flash_log_t *log,
                                      uint8_t flags,
                                      uint64_t timestamp_us,
                                      const hks_high_g_batch_hdr_t *batch_hdr,
                                      const hks_high_g_sample_t *samples);

int hks_flash_log_append_aux_env(hks_flash_log_t *log,
                                 uint8_t flags,
                                 uint64_t timestamp_us,
                                 const hks_aux_env_t *aux);

int hks_flash_log_append_event(hks_flash_log_t *log,
                               uint8_t flags,
                               uint64_t timestamp_us,
                               const hks_event_marker_t *evt);

int hks_flash_log_stop_session(hks_flash_log_t *log,
                               hks_end_reason_t reason,
                               uint64_t end_utc_ms,
                               uint64_t end_ts_us);

/* Read / query / verify / delete (returns hks_status_code_t) */
hks_status_code_t hks_flash_log_list_sessions(const hks_flash_log_t *log,
                                              hks_session_list_entry_t *out,
                                              size_t max_entries,
                                              size_t *out_count);

hks_status_code_t hks_flash_log_read_bytes(const hks_flash_log_t *log,
                                           uint64_t session_id,
                                           uint32_t offset_bytes,
                                           uint16_t length_bytes,
                                           uint8_t *out,
                                           uint16_t *actual_len);

hks_status_code_t hks_flash_log_get_header(const hks_flash_log_t *log,
                                           uint64_t session_id,
                                           uint8_t out_416[HKS_GET_HEADER_RSP_BYTES]);

hks_status_code_t hks_flash_log_verify_session(hks_flash_log_t *log,
                                               uint64_t session_id,
                                               uint32_t expected_file_crc32,
                                               const uint8_t *expected_sha256_opt,
                                               bool check_sha256);

hks_status_code_t hks_flash_log_delete_session(hks_flash_log_t *log,
                                               uint64_t session_id,
                                               bool force_flag,
                                               uint32_t *freed_bytes_out);

uint32_t hks_flash_log_free_kb(const hks_flash_log_t *log);

#ifdef __cplusplus
}
#endif

#endif /* HKS_FLASH_LOG_H */
