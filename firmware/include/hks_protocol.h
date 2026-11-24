#ifndef HKS_PROTOCOL_H
#define HKS_PROTOCOL_H

#include "hks_contract.h"
#include "hks_flash_log.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * USB CDC-ACM / BLE Binary Download Protocol Parser & State Machine
 * -------------------------------------------------------------------------
 * Hashing boundary parity (M3 <-> M4):
 * - ACK_VERIFY (0x04) computed_file_crc32 and computed_sha256[32] cover the
 *   COMPLETE session file [0 .. byte_size) including the 96 B 0x07 footer.
 * - hash_mode = 1 (CRC32-only) is selected when computed_sha256[32] is
 *   all-zero (or omitted in 12 B form); hash_mode = 2 (CRC32+SHA256) when
 *   computed_sha256 is non-zero.
 * - 0x07 SESSION_FOOTER.pre_footer_sha256[32] covers [0 .. byte_size - 96).
 * ========================================================================= */

#define HKS_PROTO_MAX_RX_PAYLOAD 128u
#define HKS_PROTO_MAX_TX_FRAME   (HKS_PROTO_RSP_OVERHEAD_BYTES + HKS_READ_CHUNK_RSP_HDR_BYTES + HKS_PROTO_MAX_CHUNK_BYTES)

typedef enum {
    HKS_RX_WAIT_SYNC0      = 0,
    HKS_RX_WAIT_SYNC1      = 1,
    HKS_RX_READ_HDR        = 2,
    HKS_RX_READ_PAYLOAD    = 3,
    HKS_RX_READ_CRC        = 4,
    HKS_RX_DRAIN_OVERSIZE  = 5
} hks_proto_rx_state_t;

typedef struct {
    hks_flash_log_t     *flash;
    uint32_t             fw_version;
    uint16_t             hw_rev;

    /* UTC epoch tracking */
    bool                 utc_synced;
    uint64_t             utc_base_ms;
    uint64_t             utc_sync_mono_us;

    /* Latest PMIC status snapshot (for GET_DEVICE_STATUS parity with AUX_ENV) */
    uint16_t             vbatt_mv;
    uint16_t             soc_pct_x10;
    uint8_t              pmic_status; /* bits [7:0] */

    /* Stream parser state */
    hks_proto_rx_state_t rx_state;
    uint8_t              rx_hdr[HKS_PROTO_CMD_HDR_BYTES];
    size_t               rx_hdr_pos;
    uint8_t              rx_cmd_id;
    uint8_t              rx_seq;
    uint16_t             rx_payload_len;
    uint8_t              rx_payload[HKS_PROTO_MAX_RX_PAYLOAD];
    size_t               rx_payload_pos;
    uint8_t              rx_crc[4];
    size_t               rx_crc_pos;
    size_t               rx_drain_rem;
    uint8_t              rsp_scratch[HKS_READ_CHUNK_RSP_HDR_BYTES + HKS_PROTO_MAX_CHUNK_BYTES];
    uint8_t              tx_buf[HKS_PROTO_MAX_TX_FRAME];
} hks_protocol_t;

void     hks_protocol_init(hks_protocol_t *proto, hks_flash_log_t *flash,
                           uint32_t fw_version, uint16_t hw_rev);

void     hks_protocol_update_pmic_cache(hks_protocol_t *proto,
                                        uint16_t vbatt_mv,
                                        uint16_t soc_pct_x10,
                                        uint16_t aux_pmic_status);

uint64_t hks_protocol_now_utc_ms(const hks_protocol_t *proto);

/* Process raw command bytes in-memory; writes response envelope into rsp_out.
 * Returns response byte count (0 if incomplete frame buffered). */
size_t   hks_protocol_feed_bytes(hks_protocol_t *proto,
                                 const uint8_t *rx_bytes,
                                 size_t rx_len,
                                 uint8_t *rsp_out,
                                 size_t rsp_cap);

/* Poll hks_hal_usb_cdc_read() and emit responses via hks_hal_usb_cdc_write(). */
size_t   hks_protocol_poll_usb(hks_protocol_t *proto);

/* Helper to build a valid command packet into dst (for CLI / tests). */
size_t   hks_protocol_encode_cmd(uint8_t *dst, size_t dst_cap,
                                 uint8_t cmd_id, uint8_t seq,
                                 const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* HKS_PROTOCOL_H */
