#include "hks_protocol.h"
#include "hks_crc32.h"
#include "hks_hal.h"

#include <string.h>

static bool sha256_is_all_zero(const uint8_t sha[HKS_SHA256_DIGEST_BYTES]) {
    for (size_t i = 0u; i < HKS_SHA256_DIGEST_BYTES; i++) {
        if (sha[i] != 0u) {
            return false;
        }
    }
    return true;
}

static size_t build_response(uint8_t *out,
                             size_t out_cap,
                             uint8_t cmd_id,
                             hks_status_code_t status,
                             uint8_t seq,
                             const uint8_t *payload,
                             uint16_t payload_len) {
    const size_t total = (size_t)HKS_PROTO_RSP_OVERHEAD_BYTES + (size_t)payload_len;
    if (out == NULL || out_cap < total) {
        return 0u;
    }
    hks_put_u16_le(out + 0u, HKS_RSP_SYNC_MARKER);
    out[2] = cmd_id;
    out[3] = (uint8_t)status;
    out[4] = seq;
    out[5] = 0u;
    hks_put_u16_le(out + 6u, payload_len);
    if (payload_len > 0u && payload != NULL && (out + HKS_PROTO_RSP_HDR_BYTES) != payload) {
        memmove(out + HKS_PROTO_RSP_HDR_BYTES, payload, payload_len);
    }
    const uint32_t crc = hks_crc32(out, (size_t)HKS_PROTO_RSP_HDR_BYTES + (size_t)payload_len);
    hks_put_u32_le(out + HKS_PROTO_RSP_HDR_BYTES + payload_len, crc);
    return total;
}

void hks_protocol_init(hks_protocol_t *proto, hks_flash_log_t *flash,
                       uint32_t fw_version, uint16_t hw_rev) {
    if (proto == NULL) {
        return;
    }
    memset(proto, 0, sizeof(*proto));
    proto->flash = flash;
    proto->fw_version = (fw_version != 0u) ? fw_version : HKS_FW_VERSION_DEFAULT;
    proto->hw_rev = (hw_rev != 0u) ? hw_rev : HKS_HW_REV_DEFAULT;
    proto->vbatt_mv = 3900u;
    proto->soc_pct_x10 = 850u;
    proto->pmic_status = HKS_PMIC_FG_VALID;
    proto->rx_state = HKS_RX_WAIT_SYNC0;
}

void hks_protocol_update_pmic_cache(hks_protocol_t *proto,
                                    uint16_t vbatt_mv,
                                    uint16_t soc_pct_x10,
                                    uint16_t aux_pmic_status) {
    if (proto == NULL) {
        return;
    }
    proto->vbatt_mv = vbatt_mv;
    proto->soc_pct_x10 = soc_pct_x10;
    proto->pmic_status = (uint8_t)(aux_pmic_status & 0xFFu);
}

uint64_t hks_protocol_now_utc_ms(const hks_protocol_t *proto) {
    if (proto == NULL || !proto->utc_synced) {
        return 0u;
    }
    const uint64_t now_us = hks_hal_time_us();
    const uint64_t delta_ms = (now_us >= proto->utc_sync_mono_us)
                                  ? ((now_us - proto->utc_sync_mono_us) / 1000ULL)
                                  : 0ULL;
    return proto->utc_base_ms + delta_ms;
}

static size_t dispatch_command(hks_protocol_t *proto, uint8_t *rsp_out, size_t rsp_cap) {
    const uint8_t cmd_id = proto->rx_cmd_id;
    const uint8_t seq = proto->rx_seq;
    const uint16_t plen = proto->rx_payload_len;
    const uint8_t *p = proto->rx_payload;

    /* Verify command envelope CRC32 over hdr[0..5] || payload[0..plen-1] */
    uint32_t crc_state = hks_crc32_init();
    crc_state = hks_crc32_update(crc_state, proto->rx_hdr, HKS_PROTO_CMD_HDR_BYTES);
    if (plen > 0u) {
        crc_state = hks_crc32_update(crc_state, p, plen);
    }
    const uint32_t expected_crc = hks_crc32_final(crc_state);
    const uint32_t actual_crc = hks_get_u32_le(proto->rx_crc);
    if (expected_crc != actual_crc) {
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_BAD_CRC, seq, NULL, 0u);
    }

    if (cmd_id < HKS_CMD_LIST_SESSIONS || cmd_id > HKS_CMD_SET_UTC_TIME) {
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_UNKNOWN_CMD, seq, NULL, 0u);
    }

    if (proto->flash == NULL) {
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_FLASH_IO, seq, NULL, 0u);
    }

    const bool recording = (proto->flash->state == HKS_FLASH_STATE_RECORDING);
    if (recording && cmd_id != HKS_CMD_GET_DEVICE_STATUS) {
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_BUSY_RECORDING, seq, NULL, 0u);
    }

    uint8_t *rsp_payload = proto->rsp_scratch;

    switch ((hks_cmd_id_t)cmd_id) {
    case HKS_CMD_LIST_SESSIONS: {
        if (plen != 0u) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        hks_session_list_entry_t entries[HKS_FLASH_MAX_SESSIONS];
        size_t count = 0u;
        const hks_status_code_t st = hks_flash_log_list_sessions(proto->flash, entries,
                                                                 HKS_FLASH_MAX_SESSIONS, &count);
        if (st != HKS_STATUS_OK) {
            return build_response(rsp_out, rsp_cap, cmd_id, st, seq, NULL, 0u);
        }
        if (count > 127u) {
            count = 127u; /* fits within 4096 B chunk payload */
        }
        hks_put_u16_le(rsp_payload + 0u, (uint16_t)count);
        hks_put_u16_le(rsp_payload + 2u, 0u);
        for (size_t i = 0u; i < count; i++) {
            (void)hks_encode_list_entry(rsp_payload + HKS_LIST_SESSIONS_HDR_BYTES +
                                            i * HKS_LIST_SESSIONS_ENTRY_BYTES,
                                        HKS_LIST_SESSIONS_ENTRY_BYTES, &entries[i]);
        }
        const uint16_t out_len = (uint16_t)(HKS_LIST_SESSIONS_HDR_BYTES +
                                            count * HKS_LIST_SESSIONS_ENTRY_BYTES);
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq, rsp_payload, out_len);
    }

    case HKS_CMD_GET_HEADER: {
        if (plen != HKS_GET_HEADER_REQ_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const uint64_t session_id = hks_get_u64_le(p + 0u);
        if (session_id == 0u) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const hks_status_code_t st = hks_flash_log_get_header(proto->flash, session_id, rsp_payload);
        if (st != HKS_STATUS_OK) {
            return build_response(rsp_out, rsp_cap, cmd_id, st, seq, NULL, 0u);
        }
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, HKS_GET_HEADER_RSP_BYTES);
    }

    case HKS_CMD_READ_CHUNK: {
        if (plen != HKS_READ_CHUNK_REQ_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const uint64_t session_id = hks_get_u64_le(p + 0u);
        const uint32_t offset_bytes = hks_get_u32_le(p + 8u);
        const uint16_t length_bytes = hks_get_u16_le(p + 12u);
        if (session_id == 0u || length_bytes == 0u || length_bytes > HKS_PROTO_MAX_CHUNK_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        uint16_t actual_len = 0u;
        const hks_status_code_t st = hks_flash_log_read_bytes(
            proto->flash, session_id, offset_bytes, length_bytes,
            rsp_payload + HKS_READ_CHUNK_RSP_HDR_BYTES, &actual_len);
        if (st != HKS_STATUS_OK) {
            return build_response(rsp_out, rsp_cap, cmd_id, st, seq, NULL, 0u);
        }
        hks_put_u32_le(rsp_payload + 0u, offset_bytes);
        hks_put_u16_le(rsp_payload + 4u, actual_len);
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, (uint16_t)(HKS_READ_CHUNK_RSP_HDR_BYTES + actual_len));
    }

    case HKS_CMD_ACK_VERIFY: {
        if (plen != HKS_ACK_VERIFY_REQ_BYTES && plen != HKS_ACK_VERIFY_REQ_MIN_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const uint64_t session_id = hks_get_u64_le(p + 0u);
        const uint32_t expected_crc32 = hks_get_u32_le(p + 8u);
        if (session_id == 0u) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        uint8_t sha[HKS_SHA256_DIGEST_BYTES];
        memset(sha, 0, sizeof(sha));
        if (plen == HKS_ACK_VERIFY_REQ_BYTES) {
            memcpy(sha, p + 12u, HKS_SHA256_DIGEST_BYTES);
        }
        const bool check_sha = !sha256_is_all_zero(sha);
        const hks_status_code_t st = hks_flash_log_verify_session(
            proto->flash, session_id, expected_crc32, sha, check_sha);
        if (st != HKS_STATUS_OK) {
            return build_response(rsp_out, rsp_cap, cmd_id, st, seq, NULL, 0u);
        }
        rsp_payload[0] = 1u; /* verified_status = OK */
        rsp_payload[1] = check_sha ? 2u : 1u;
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, HKS_ACK_VERIFY_RSP_BYTES);
    }

    case HKS_CMD_DELETE_SESSION: {
        if (plen != HKS_DELETE_SESSION_REQ_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const uint64_t session_id = hks_get_u64_le(p + 0u);
        const uint8_t force_flag = p[8];
        if (session_id == 0u || force_flag > 1u) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        uint32_t freed_bytes = 0u;
        const hks_status_code_t st = hks_flash_log_delete_session(
            proto->flash, session_id, force_flag == 1u, &freed_bytes);
        if (st != HKS_STATUS_OK) {
            return build_response(rsp_out, rsp_cap, cmd_id, st, seq, NULL, 0u);
        }
        hks_put_u32_le(rsp_payload + 0u, freed_bytes);
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, HKS_DELETE_SESSION_RSP_BYTES);
    }

    case HKS_CMD_GET_DEVICE_STATUS: {
        if (plen != 0u) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        hks_hal_pmic_info_t pmic;
        if (hks_hal_pmic_read(&pmic) == 0) {
            proto->vbatt_mv = pmic.vbatt_mv;
            proto->soc_pct_x10 = pmic.soc_pct_x10;
            proto->pmic_status = pmic.pmic_status;
        }
        hks_device_status_t ds;
        ds.fw_version = proto->fw_version;
        ds.hw_rev = proto->hw_rev;
        ds.vbatt_mv = proto->vbatt_mv;
        ds.soc_pct_x10 = proto->soc_pct_x10;
        ds.recording_state = recording ? 1u : 0u;
        ds.pmic_status = proto->pmic_status;
        ds.flash_free_kb = hks_flash_log_free_kb(proto->flash);
        ds.utc_ms = hks_protocol_now_utc_ms(proto);
        (void)hks_encode_device_status(rsp_payload, HKS_GET_DEVICE_STATUS_RSP_BYTES, &ds);
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, HKS_GET_DEVICE_STATUS_RSP_BYTES);
    }

    case HKS_CMD_SET_UTC_TIME: {
        if (plen != HKS_SET_UTC_TIME_BYTES) {
            return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_INVALID_PARAM, seq, NULL, 0u);
        }
        const uint64_t utc_ms = hks_get_u64_le(p + 0u);
        proto->utc_synced = true;
        proto->utc_base_ms = utc_ms;
        proto->utc_sync_mono_us = hks_hal_time_us();
        hks_put_u64_le(rsp_payload + 0u, utc_ms);
        return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_OK, seq,
                              rsp_payload, HKS_SET_UTC_TIME_BYTES);
    }
    }

    return build_response(rsp_out, rsp_cap, cmd_id, HKS_STATUS_ERR_UNKNOWN_CMD, seq, NULL, 0u);
}

size_t hks_protocol_feed_bytes(hks_protocol_t *proto,
                               const uint8_t *rx_bytes,
                               size_t rx_len,
                               uint8_t *rsp_out,
                               size_t rsp_cap) {
    if (proto == NULL || rx_bytes == NULL || rx_len == 0u) {
        return 0u;
    }

    size_t total_rsp = 0u;
    for (size_t i = 0u; i < rx_len; i++) {
        const uint8_t b = rx_bytes[i];

        switch (proto->rx_state) {
        case HKS_RX_WAIT_SYNC0:
            if (b == (uint8_t)(HKS_SYNC_MARKER & 0xFFu)) {
                proto->rx_hdr[0] = b;
                proto->rx_state = HKS_RX_WAIT_SYNC1;
            }
            break;

        case HKS_RX_WAIT_SYNC1:
            if (b == (uint8_t)((HKS_SYNC_MARKER >> 8) & 0xFFu)) {
                proto->rx_hdr[1] = b;
                proto->rx_hdr_pos = 2u;
                proto->rx_state = HKS_RX_READ_HDR;
            } else if (b == (uint8_t)(HKS_SYNC_MARKER & 0xFFu)) {
                proto->rx_hdr[0] = b;
            } else {
                proto->rx_state = HKS_RX_WAIT_SYNC0;
            }
            break;

        case HKS_RX_READ_HDR:
            proto->rx_hdr[proto->rx_hdr_pos++] = b;
            if (proto->rx_hdr_pos == HKS_PROTO_CMD_HDR_BYTES) {
                proto->rx_cmd_id = proto->rx_hdr[2];
                proto->rx_seq = proto->rx_hdr[3];
                proto->rx_payload_len = hks_get_u16_le(proto->rx_hdr + 4u);
                if (proto->rx_payload_len > HKS_PROTO_MAX_RX_PAYLOAD) {
                    if (rsp_out != NULL && rsp_cap > total_rsp) {
                        total_rsp += build_response(rsp_out + total_rsp, rsp_cap - total_rsp,
                                                    proto->rx_cmd_id, HKS_STATUS_ERR_INVALID_PARAM,
                                                    proto->rx_seq, NULL, 0u);
                    }
                    if (proto->rx_payload_len <= HKS_MAX_FRAME_BYTES) {
                        proto->rx_drain_rem = (size_t)proto->rx_payload_len + 4u;
                        proto->rx_state = HKS_RX_DRAIN_OVERSIZE;
                    } else {
                        proto->rx_state = HKS_RX_WAIT_SYNC0;
                    }
                } else if (proto->rx_payload_len == 0u) {
                    proto->rx_crc_pos = 0u;
                    proto->rx_state = HKS_RX_READ_CRC;
                } else {
                    proto->rx_payload_pos = 0u;
                    proto->rx_state = HKS_RX_READ_PAYLOAD;
                }
            }
            break;

        case HKS_RX_READ_PAYLOAD:
            proto->rx_payload[proto->rx_payload_pos++] = b;
            if (proto->rx_payload_pos == proto->rx_payload_len) {
                proto->rx_crc_pos = 0u;
                proto->rx_state = HKS_RX_READ_CRC;
            }
            break;

        case HKS_RX_READ_CRC:
            proto->rx_crc[proto->rx_crc_pos++] = b;
            if (proto->rx_crc_pos == 4u) {
                if (rsp_out != NULL && rsp_cap > total_rsp) {
                    total_rsp += dispatch_command(proto, rsp_out + total_rsp, rsp_cap - total_rsp);
                }
                proto->rx_state = HKS_RX_WAIT_SYNC0;
            }
            break;

        case HKS_RX_DRAIN_OVERSIZE:
            if (proto->rx_drain_rem > 0u) {
                proto->rx_drain_rem--;
            }
            if (proto->rx_drain_rem == 0u) {
                proto->rx_state = HKS_RX_WAIT_SYNC0;
            }
            break;
        }
    }
    return total_rsp;
}

size_t hks_protocol_poll_usb(hks_protocol_t *proto) {
    if (proto == NULL) {
        return 0u;
    }
    uint8_t rx_chunk[256];
    size_t total_tx = 0u;
    while (hks_hal_usb_cdc_rx_available() > 0u) {
        const size_t n = hks_hal_usb_cdc_read(rx_chunk, sizeof(rx_chunk));
        if (n == 0u) {
            break;
        }
        /* Feed one byte or chunk; dispatch_command may emit up to 4114 B per cmd */
        for (size_t i = 0u; i < n; i++) {
            const size_t rsp_len = hks_protocol_feed_bytes(
                proto, &rx_chunk[i], 1u, proto->tx_buf, sizeof(proto->tx_buf));
            if (rsp_len > 0u) {
                total_tx += hks_hal_usb_cdc_write(proto->tx_buf, rsp_len);
            }
        }
    }
    return total_tx;
}

size_t hks_protocol_encode_cmd(uint8_t *dst, size_t dst_cap,
                               uint8_t cmd_id, uint8_t seq,
                               const uint8_t *payload, uint16_t payload_len) {
    const size_t total = (size_t)HKS_PROTO_CMD_OVERHEAD_BYTES + (size_t)payload_len;
    if (dst == NULL || dst_cap < total) {
        return 0u;
    }
    if (payload_len > 0u && payload == NULL) {
        return 0u;
    }
    hks_put_u16_le(dst + 0u, HKS_SYNC_MARKER);
    dst[2] = cmd_id;
    dst[3] = seq;
    hks_put_u16_le(dst + 4u, payload_len);
    if (payload_len > 0u) {
        memcpy(dst + HKS_PROTO_CMD_HDR_BYTES, payload, payload_len);
    }
    const uint32_t crc = hks_crc32(dst, (size_t)HKS_PROTO_CMD_HDR_BYTES + (size_t)payload_len);
    hks_put_u32_le(dst + HKS_PROTO_CMD_HDR_BYTES + payload_len, crc);
    return total;
}
