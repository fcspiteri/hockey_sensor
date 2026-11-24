#include "hks_flash_log.h"
#include "hks_crc32.h"
#include "hks_hal.h"

#include <string.h>

#define HKS_INDEX_MAGIC 0x49534B48u /* "HKSI" little-endian */
#define HKS_FCAL_MAGIC  0x43464B48u /* "HKFC" little-endian */

#define HKS_IDX_KIND_OP         1u
#define HKS_IDX_KIND_CKPT_META  2u
#define HKS_IDX_KIND_CKPT_CHAIN 3u

#define HKS_INDEX_OP_OPEN        1u
#define HKS_INDEX_OP_CLOSE       2u
#define HKS_INDEX_OP_VERIFY      3u
#define HKS_INDEX_OP_DELETE      4u
#define HKS_INDEX_OP_BLOCK_ALLOC 5u

#define HKS_IDX_PAGE_HDR_BYTES   16u
#define HKS_IDX_PAGE_CRC_OFF     (HKS_NAND_PAGE_BYTES - 4u) /* 2044 */
#define HKS_IDX_META_ENTRY_BYTES 72u
#define HKS_IDX_META_PER_PAGE    ((HKS_IDX_PAGE_CRC_OFF - HKS_IDX_PAGE_HDR_BYTES) / HKS_IDX_META_ENTRY_BYTES) /* 28 */
#define HKS_IDX_CHAIN_SEG_HDR    12u
#define HKS_IDX_COMPACT_THRESH   12u /* >= max snapshot pages (5..6) + 1 */

#define HKS_FCAL_REC_BYTES       312u /* 24 B header + 288 B cal_block */

typedef struct {
    uint32_t seq;
    uint32_t ckpt_seq_start;
    uint16_t block;
    uint8_t  page;
    uint8_t  kind;
    uint8_t  sub;
} index_loc_t;

typedef struct {
    uint8_t  op;
    uint8_t  flags;
    uint64_t session_id;
    uint64_t start_utc_ms;
    uint32_t duration_ms;
    uint32_t byte_size;
    uint32_t sample_count;
    uint32_t session_flags;
    uint16_t start_block;
    uint16_t block_count;
    uint16_t alloc_logical_idx;
    uint16_t alloc_phys_block;
    uint8_t  whole_file_sha256[HKS_SHA256_DIGEST_BYTES];
} index_op_t;

static bool is_buf_erased(const uint8_t *buf, size_t len) {
    for (size_t i = 0u; i < len; i++) {
        if (buf[i] != 0xFFu) {
            return false;
        }
    }
    return true;
}

static bool is_valid_ring_block(uint16_t b) {
    return (b >= HKS_PART_RING_START && b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS);
}

static void encode_meta_entry(uint8_t *out, const hks_flash_session_meta_t *m) {
    const uint8_t flg = (uint8_t)((m->closed ? HKS_SESS_FLAG_CLOSED : 0u) |
                                  (m->verified ? HKS_SESS_FLAG_VERIFIED : 0u) |
                                  (m->recovered ? HKS_SESS_FLAG_RECOVERED : 0u));
    hks_put_u64_le(out + 0u, m->session_id);
    hks_put_u64_le(out + 8u, m->start_utc_ms);
    hks_put_u32_le(out + 16u, m->duration_ms);
    hks_put_u32_le(out + 20u, m->byte_size);
    hks_put_u32_le(out + 24u, m->sample_count);
    hks_put_u32_le(out + 28u, m->session_flags);
    hks_put_u16_le(out + 32u, m->start_block);
    hks_put_u16_le(out + 34u, m->block_count);
    out[36] = flg;
    out[37] = 0u;
    out[38] = 0u;
    out[39] = 0u;
    memcpy(out + 40u, m->whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
}

static void decode_meta_entry(const uint8_t *in, hks_flash_session_meta_t *m, uint8_t *flags_out) {
    m->session_id = hks_get_u64_le(in + 0u);
    m->start_utc_ms = hks_get_u64_le(in + 8u);
    m->duration_ms = hks_get_u32_le(in + 16u);
    m->byte_size = hks_get_u32_le(in + 20u);
    m->sample_count = hks_get_u32_le(in + 24u);
    m->session_flags = hks_get_u32_le(in + 28u);
    m->start_block = hks_get_u16_le(in + 32u);
    m->block_count = hks_get_u16_le(in + 34u);
    const uint8_t flg = in[36];
    if (flags_out != NULL) {
        *flags_out = flg;
    }
    m->closed = (flg & HKS_SESS_FLAG_CLOSED) != 0u;
    m->verified = (flg & HKS_SESS_FLAG_VERIFIED) != 0u;
    m->recovered = (flg & HKS_SESS_FLAG_RECOVERED) != 0u;
    memcpy(m->whole_file_sha256, in + 40u, HKS_SHA256_DIGEST_BYTES);
}

static void finalize_index_page(uint8_t page[HKS_NAND_PAGE_BYTES],
                                uint32_t seq,
                                uint8_t kind,
                                uint8_t sub,
                                uint16_t count,
                                uint32_t ckpt_seq_start) {
    hks_put_u32_le(page + 0u, HKS_INDEX_MAGIC);
    hks_put_u32_le(page + 4u, seq);
    page[8] = kind;
    page[9] = sub;
    hks_put_u16_le(page + 10u, count);
    hks_put_u32_le(page + 12u, ckpt_seq_start);
    const uint32_t crc = hks_crc32(page, HKS_IDX_PAGE_CRC_OFF);
    hks_put_u32_le(page + HKS_IDX_PAGE_CRC_OFF, crc);
}

static bool validate_index_page_hdr(const uint8_t page[HKS_NAND_PAGE_BYTES],
                                    uint32_t *seq_out,
                                    uint8_t *kind_out,
                                    uint8_t *sub_out,
                                    uint16_t *count_out,
                                    uint32_t *ckpt_seq_start_out) {
    if (hks_get_u32_le(page + 0u) != HKS_INDEX_MAGIC) {
        return false;
    }
    const uint32_t expected_crc = hks_crc32(page, HKS_IDX_PAGE_CRC_OFF);
    const uint32_t actual_crc = hks_get_u32_le(page + HKS_IDX_PAGE_CRC_OFF);
    if (expected_crc != actual_crc) {
        return false;
    }
    if (seq_out != NULL) *seq_out = hks_get_u32_le(page + 4u);
    if (kind_out != NULL) *kind_out = page[8];
    if (sub_out != NULL) *sub_out = page[9];
    if (count_out != NULL) *count_out = hks_get_u16_le(page + 10u);
    if (ckpt_seq_start_out != NULL) *ckpt_seq_start_out = hks_get_u32_le(page + 12u);
    return true;
}

static hks_flash_session_meta_t *find_session_mut(hks_flash_log_t *log, uint64_t session_id) {
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (log->sessions[i].valid && log->sessions[i].session_id == session_id) {
            return &log->sessions[i];
        }
    }
    return NULL;
}

static const hks_flash_session_meta_t *find_session_const(const hks_flash_log_t *log, uint64_t session_id) {
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (log->sessions[i].valid && log->sessions[i].session_id == session_id) {
            return &log->sessions[i];
        }
    }
    return NULL;
}

static size_t find_session_slot(const hks_flash_log_t *log, uint64_t session_id) {
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (log->sessions[i].valid && log->sessions[i].session_id == session_id) {
            return i;
        }
    }
    return HKS_FLASH_MAX_SESSIONS;
}

static size_t alloc_session_slot(hks_flash_log_t *log) {
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (!log->sessions[i].valid) {
            memset(&log->sessions[i], 0, sizeof(log->sessions[i]));
            log->session_tail_block[i] = 0xFFFFu;
            return i;
        }
    }
    return HKS_FLASH_MAX_SESSIONS;
}

static bool session_append_block(hks_flash_log_t *log, size_t slot_idx, uint16_t phys_block) {
    if (slot_idx >= HKS_FLASH_MAX_SESSIONS || !is_valid_ring_block(phys_block)) {
        return false;
    }
    if (log->block_owner[phys_block] != 0xFFu) {
        return false;
    }
    hks_flash_session_meta_t *m = &log->sessions[slot_idx];
    if (m->block_count == 0u || log->session_tail_block[slot_idx] == 0xFFFFu) {
        m->start_block = phys_block;
        log->session_tail_block[slot_idx] = phys_block;
        m->block_count = 1u;
    } else {
        const uint16_t tail = log->session_tail_block[slot_idx];
        log->block_next[tail] = phys_block;
        log->session_tail_block[slot_idx] = phys_block;
        m->block_count++;
    }
    log->block_next[phys_block] = 0xFFFFu;
    log->block_owner[phys_block] = (uint8_t)slot_idx;
    return true;
}

static void session_clear_chain(hks_flash_log_t *log, size_t slot_idx) {
    if (slot_idx >= HKS_FLASH_MAX_SESSIONS) {
        return;
    }
    hks_flash_session_meta_t *m = &log->sessions[slot_idx];
    uint16_t b = (m->block_count > 0u) ? m->start_block : 0xFFFFu;
    for (uint16_t step = 0u; step < HKS_PART_RING_BLOCKS && b != 0xFFFFu; step++) {
        const uint16_t nxt = log->block_next[b];
        if (log->block_owner[b] == (uint8_t)slot_idx) {
            log->block_owner[b] = 0xFFu;
        }
        log->block_next[b] = 0xFFFFu;
        b = nxt;
    }
    m->start_block = 0u;
    m->block_count = 0u;
    log->session_tail_block[slot_idx] = 0xFFFFu;
}

static void session_trim_to_blocks(hks_flash_log_t *log, size_t slot_idx, uint16_t keep_count) {
    if (slot_idx >= HKS_FLASH_MAX_SESSIONS) {
        return;
    }
    hks_flash_session_meta_t *m = &log->sessions[slot_idx];
    if (keep_count >= m->block_count) {
        return;
    }
    if (keep_count == 0u) {
        session_clear_chain(log, slot_idx);
        return;
    }
    uint16_t b = m->start_block;
    for (uint16_t i = 1u; i < keep_count && b != 0xFFFFu; i++) {
        b = log->block_next[b];
    }
    if (b == 0xFFFFu) {
        return;
    }
    uint16_t drop = log->block_next[b];
    log->block_next[b] = 0xFFFFu;
    log->session_tail_block[slot_idx] = b;
    m->block_count = keep_count;
    for (uint16_t step = 0u; step < HKS_PART_RING_BLOCKS && drop != 0xFFFFu; step++) {
        const uint16_t nxt = log->block_next[drop];
        if (log->block_owner[drop] == (uint8_t)slot_idx) {
            log->block_owner[drop] = 0xFFu;
        }
        log->block_next[drop] = 0xFFFFu;
        drop = nxt;
    }
}

static uint16_t next_ring_block(const hks_flash_log_t *log, uint16_t cur_block) {
    uint16_t b = cur_block;
    for (uint16_t step = 0u; step < HKS_PART_RING_BLOCKS; step++) {
        b++;
        if (b >= HKS_PART_RING_START + HKS_PART_RING_BLOCKS) {
            b = HKS_PART_RING_START;
        }
        if (!log->bad_blocks[b]) {
            return b;
        }
    }
    return 0xFFFFu;
}

static bool get_session_block_at_idx(const hks_flash_log_t *log,
                                     const hks_flash_session_meta_t *meta,
                                     uint16_t logical_idx,
                                     uint16_t *phys_block_out) {
    if (meta == NULL || phys_block_out == NULL || logical_idx >= meta->block_count) {
        return false;
    }
    uint16_t b = meta->start_block;
    if (!is_valid_ring_block(b) || log->bad_blocks[b]) {
        return false;
    }
    for (uint16_t i = 0u; i < logical_idx; i++) {
        b = log->block_next[b];
        if (!is_valid_ring_block(b) || log->bad_blocks[b]) {
            return false;
        }
    }
    *phys_block_out = b;
    return true;
}

static bool is_block_used_by_any_session(const hks_flash_log_t *log, uint16_t phys_block) {
    if (!is_valid_ring_block(phys_block)) {
        return false;
    }
    return (log->block_owner[phys_block] != 0xFFu);
}

static int index_next_good_block(const hks_flash_log_t *log, uint16_t cur, uint16_t *out) {
    uint16_t b = cur;
    for (uint16_t i = 0u; i < HKS_PART_SESSION_INDEX_BLOCKS; i++) {
        b++;
        if (b >= HKS_PART_SESSION_INDEX_START + HKS_PART_SESSION_INDEX_BLOCKS) {
            b = HKS_PART_SESSION_INDEX_START;
        }
        if (!log->bad_blocks[b]) {
            *out = b;
            return 0;
        }
    }
    return -1;
}

static bool index_block_has_data(uint16_t block) {
    uint8_t page0[HKS_NAND_PAGE_BYTES];
    if (hks_hal_nand_read_page(block, 0u, 0u, page0, sizeof(page0)) != 0) {
        return true;
    }
    return !is_buf_erased(page0, sizeof(page0));
}

static size_t count_active_sessions(const hks_flash_log_t *log) {
    size_t n = 0u;
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (log->sessions[i].valid) {
            n++;
        }
    }
    return n;
}

static int index_prog_page(hks_flash_log_t *log, const uint8_t page[HKS_NAND_PAGE_BYTES]) {
    if (log->index_cur_page >= HKS_NAND_PAGES_PER_BLOCK) {
        return -1;
    }
    if (hks_hal_nand_prog_page(log->index_cur_block, log->index_cur_page, 0u, page, HKS_NAND_PAGE_BYTES) != 0) {
        return -1;
    }
    log->index_cur_page++;
    return 0;
}

static int index_emit_checkpoint(hks_flash_log_t *log, bool dry_run, size_t *total_pages_out) {
    const size_t active = count_active_sessions(log);
    const size_t meta_pages = (active == 0u) ? 1u : ((active + HKS_IDX_META_PER_PAGE - 1u) / HKS_IDX_META_PER_PAGE);

    size_t chain_pages = 0u;
    size_t sim_pos = HKS_IDX_PAGE_CRC_OFF;
    for (size_t s = 0u; s < HKS_FLASH_MAX_SESSIONS; s++) {
        const hks_flash_session_meta_t *m = &log->sessions[s];
        if (!m->valid || m->block_count == 0u) {
            continue;
        }
        uint16_t rem_blks = m->block_count;
        while (rem_blks > 0u) {
            if (sim_pos + HKS_IDX_CHAIN_SEG_HDR + 2u > HKS_IDX_PAGE_CRC_OFF) {
                chain_pages++;
                sim_pos = HKS_IDX_PAGE_HDR_BYTES;
            }
            const size_t max_here = (HKS_IDX_PAGE_CRC_OFF - sim_pos - HKS_IDX_CHAIN_SEG_HDR) / 2u;
            const uint16_t take = (rem_blks <= max_here) ? rem_blks : (uint16_t)max_here;
            sim_pos += HKS_IDX_CHAIN_SEG_HDR + (size_t)take * 2u;
            rem_blks = (uint16_t)(rem_blks - take);
        }
    }

    const size_t total_pages = meta_pages + chain_pages;
    if (total_pages_out != NULL) {
        *total_pages_out = total_pages;
    }
    if (dry_run) {
        return 0;
    }

    const uint32_t ckpt_seq_start = log->index_seq + 1u;
    const uint8_t ckpt_total = (uint8_t)total_pages;
    uint8_t page[HKS_NAND_PAGE_BYTES];

    /* 1. Emit CKPT_META page(s) */
    if (active == 0u) {
        memset(page, 0xFF, sizeof(page));
        finalize_index_page(page, ++log->index_seq, HKS_IDX_KIND_CKPT_META,
                            ckpt_total, 0u, ckpt_seq_start);
        if (index_prog_page(log, page) != 0) {
            return -1;
        }
    } else {
        size_t s = 0u;
        for (size_t mp = 0u; mp < meta_pages; mp++) {
            memset(page, 0xFF, sizeof(page));
            uint16_t entry_cnt = 0u;
            while (s < HKS_FLASH_MAX_SESSIONS && entry_cnt < HKS_IDX_META_PER_PAGE) {
                if (log->sessions[s].valid) {
                    encode_meta_entry(page + HKS_IDX_PAGE_HDR_BYTES +
                                          (size_t)entry_cnt * HKS_IDX_META_ENTRY_BYTES,
                                      &log->sessions[s]);
                    entry_cnt++;
                }
                s++;
            }
            finalize_index_page(page, ++log->index_seq, HKS_IDX_KIND_CKPT_META,
                                ckpt_total, entry_cnt, ckpt_seq_start);
            if (index_prog_page(log, page) != 0) {
                return -1;
            }
        }
    }

    /* 2. Emit CKPT_CHAIN page(s) */
    if (chain_pages > 0u) {
        memset(page, 0xFF, sizeof(page));
        size_t pos = HKS_IDX_PAGE_HDR_BYTES;
        uint16_t seg_cnt = 0u;

        for (size_t s = 0u; s < HKS_FLASH_MAX_SESSIONS; s++) {
            const hks_flash_session_meta_t *m = &log->sessions[s];
            if (!m->valid || m->block_count == 0u) {
                continue;
            }
            uint16_t cur_blk = m->start_block;
            uint16_t lidx = 0u;
            while (lidx < m->block_count && cur_blk != 0xFFFFu) {
                if (pos + HKS_IDX_CHAIN_SEG_HDR + 2u > HKS_IDX_PAGE_CRC_OFF) {
                    finalize_index_page(page, ++log->index_seq, HKS_IDX_KIND_CKPT_CHAIN,
                                        ckpt_total, seg_cnt, ckpt_seq_start);
                    if (index_prog_page(log, page) != 0) {
                        return -1;
                    }
                    memset(page, 0xFF, sizeof(page));
                    pos = HKS_IDX_PAGE_HDR_BYTES;
                    seg_cnt = 0u;
                }
                const size_t max_here = (HKS_IDX_PAGE_CRC_OFF - pos - HKS_IDX_CHAIN_SEG_HDR) / 2u;
                const uint16_t rem_blks = (uint16_t)(m->block_count - lidx);
                const uint16_t take = (rem_blks <= max_here) ? rem_blks : (uint16_t)max_here;

                hks_put_u64_le(page + pos + 0u, m->session_id);
                hks_put_u16_le(page + pos + 8u, lidx);
                hks_put_u16_le(page + pos + 10u, take);
                pos += HKS_IDX_CHAIN_SEG_HDR;
                for (uint16_t k = 0u; k < take; k++) {
                    hks_put_u16_le(page + pos, cur_blk);
                    pos += 2u;
                    cur_blk = (cur_blk != 0xFFFFu) ? log->block_next[cur_blk] : 0xFFFFu;
                }
                lidx = (uint16_t)(lidx + take);
                seg_cnt++;
            }
        }
        if (seg_cnt > 0u) {
            finalize_index_page(page, ++log->index_seq, HKS_IDX_KIND_CKPT_CHAIN,
                                ckpt_total, seg_cnt, ckpt_seq_start);
            if (index_prog_page(log, page) != 0) {
                return -1;
            }
        }
    }

    log->index_has_unckpt_ops = false;
    return 0;
}

static int index_prepare_space(hks_flash_log_t *log) {
    uint16_t next_b = 0u;
    if (index_next_good_block(log, log->index_cur_block, &next_b) != 0) {
        return -1;
    }

    size_t ckpt_pages = 0u;
    (void)index_emit_checkpoint(log, true, &ckpt_pages);

    if (next_b == log->index_cur_block) {
        /* Emergency: only 1 good SESSION_INDEX block remains */
        if (log->index_cur_page >= HKS_NAND_PAGES_PER_BLOCK) {
            if (hks_hal_nand_erase_block(log->index_cur_block) != 0) {
                return -1;
            }
            log->index_cur_page = 0u;
            if (index_emit_checkpoint(log, false, NULL) != 0) {
                return -1;
            }
        }
        return 0;
    }

    const size_t rem = (log->index_cur_page < HKS_NAND_PAGES_PER_BLOCK)
                           ? ((size_t)HKS_NAND_PAGES_PER_BLOCK - (size_t)log->index_cur_page)
                           : 0u;

    if (index_block_has_data(next_b)) {
        if (rem <= HKS_IDX_COMPACT_THRESH) {
            if (rem >= ckpt_pages) {
                if (index_emit_checkpoint(log, false, NULL) != 0) {
                    return -1;
                }
                if (hks_hal_nand_erase_block(next_b) != 0) {
                    log->bad_blocks[next_b] = true;
                    log->bad_block_count++;
                    (void)hks_hal_nand_mark_bad_block(next_b);
                    if (index_next_good_block(log, next_b, &next_b) != 0) {
                        return -1;
                    }
                    if (next_b != log->index_cur_block) {
                        if (index_block_has_data(next_b) && hks_hal_nand_erase_block(next_b) != 0) {
                            return -1;
                        }
                        log->index_cur_block = next_b;
                        log->index_cur_page = 0u;
                    }
                } else {
                    log->index_cur_block = next_b;
                    log->index_cur_page = 0u;
                }
            } else if (!log->index_has_unckpt_ops) {
                /* Complete checkpoint already exists in index_cur_block with no trailing ops */
                if (hks_hal_nand_erase_block(next_b) != 0) {
                    return -1;
                }
                log->index_cur_block = next_b;
                log->index_cur_page = 0u;
            } else {
                /* Post-reboot at end-of-block with dirty next_b: erase next_b & checkpoint immediately */
                if (hks_hal_nand_erase_block(next_b) != 0) {
                    return -1;
                }
                log->index_cur_block = next_b;
                log->index_cur_page = 0u;
                if (index_emit_checkpoint(log, false, NULL) != 0) {
                    return -1;
                }
            }
        }
    } else if (log->index_cur_page >= HKS_NAND_PAGES_PER_BLOCK) {
        log->index_cur_block = next_b;
        log->index_cur_page = 0u;
    }

    return 0;
}

static int index_append_op(hks_flash_log_t *log, const index_op_t *op) {
    if (index_prepare_space(log) != 0) {
        return -1;
    }
    uint8_t page[HKS_NAND_PAGE_BYTES];
    memset(page, 0xFF, sizeof(page));
    if (op->op == HKS_INDEX_OP_DELETE) {
        hks_put_u64_le(page + HKS_IDX_PAGE_HDR_BYTES, op->session_id);
    } else if (op->op == HKS_INDEX_OP_BLOCK_ALLOC) {
        hks_put_u64_le(page + HKS_IDX_PAGE_HDR_BYTES + 0u, op->session_id);
        hks_put_u16_le(page + HKS_IDX_PAGE_HDR_BYTES + 8u, op->alloc_logical_idx);
        hks_put_u16_le(page + HKS_IDX_PAGE_HDR_BYTES + 10u, op->alloc_phys_block);
    } else {
        hks_flash_session_meta_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        tmp.session_id = op->session_id;
        tmp.start_utc_ms = op->start_utc_ms;
        tmp.duration_ms = op->duration_ms;
        tmp.byte_size = op->byte_size;
        tmp.sample_count = op->sample_count;
        tmp.session_flags = op->session_flags;
        tmp.start_block = op->start_block;
        tmp.block_count = op->block_count;
        tmp.closed = (op->flags & HKS_SESS_FLAG_CLOSED) != 0u;
        tmp.verified = (op->flags & HKS_SESS_FLAG_VERIFIED) != 0u;
        tmp.recovered = (op->flags & HKS_SESS_FLAG_RECOVERED) != 0u;
        memcpy(tmp.whole_file_sha256, op->whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
        encode_meta_entry(page + HKS_IDX_PAGE_HDR_BYTES, &tmp);
    }
    finalize_index_page(page, ++log->index_seq, HKS_IDX_KIND_OP, op->op, 0u, 0u);
    if (index_prog_page(log, page) != 0) {
        return -1;
    }
    log->index_has_unckpt_ops = true;
    return 0;
}

static int index_append_block_alloc(hks_flash_log_t *log,
                                    uint64_t session_id,
                                    uint16_t logical_idx,
                                    uint16_t phys_block) {
    index_op_t op;
    memset(&op, 0, sizeof(op));
    op.op = HKS_INDEX_OP_BLOCK_ALLOC;
    op.session_id = session_id;
    op.alloc_logical_idx = logical_idx;
    op.alloc_phys_block = phys_block;
    return index_append_op(log, &op);
}

static void apply_index_op(hks_flash_log_t *log, const index_op_t *op, size_t *slot_out) {
    if (op->op == HKS_INDEX_OP_DELETE) {
        const size_t s = find_session_slot(log, op->session_id);
        if (s < HKS_FLASH_MAX_SESSIONS) {
            session_clear_chain(log, s);
            log->sessions[s].valid = false;
            if (log->session_count > 0u) {
                log->session_count--;
            }
        }
        if (slot_out != NULL) *slot_out = HKS_FLASH_MAX_SESSIONS;
        return;
    }

    if (op->op == HKS_INDEX_OP_BLOCK_ALLOC) {
        const size_t s = find_session_slot(log, op->session_id);
        if (s < HKS_FLASH_MAX_SESSIONS && op->alloc_logical_idx == log->sessions[s].block_count) {
            (void)session_append_block(log, s, op->alloc_phys_block);
        }
        if (slot_out != NULL) *slot_out = s;
        return;
    }

    size_t s = find_session_slot(log, op->session_id);
    if (s >= HKS_FLASH_MAX_SESSIONS) {
        s = alloc_session_slot(log);
        if (s >= HKS_FLASH_MAX_SESSIONS) {
            if (slot_out != NULL) *slot_out = HKS_FLASH_MAX_SESSIONS;
            return;
        }
        log->sessions[s].valid = true;
        log->session_count++;
    }

    hks_flash_session_meta_t *m = &log->sessions[s];
    m->session_id = op->session_id;
    m->start_utc_ms = op->start_utc_ms;
    m->duration_ms = op->duration_ms;
    m->byte_size = op->byte_size;
    m->sample_count = op->sample_count;
    m->session_flags = op->session_flags;
    memcpy(m->whole_file_sha256, op->whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
    m->closed = (op->flags & HKS_SESS_FLAG_CLOSED) != 0u;
    m->verified = (op->flags & HKS_SESS_FLAG_VERIFIED) != 0u;
    m->recovered = (op->flags & HKS_SESS_FLAG_RECOVERED) != 0u;

    if (op->op == HKS_INDEX_OP_OPEN) {
        if (m->block_count == 0u && is_valid_ring_block(op->start_block)) {
            if (log->block_owner[op->start_block] == 0xFEu) {
                log->block_owner[op->start_block] = 0xFFu;
            }
            (void)session_append_block(log, s, op->start_block);
        }
    } else if (op->op == HKS_INDEX_OP_CLOSE) {
        if (op->block_count > 0u && op->block_count < m->block_count) {
            session_trim_to_blocks(log, s, op->block_count);
        }
    }

    if (slot_out != NULL) *slot_out = s;
}

static void apply_index_page(hks_flash_log_t *log, const uint8_t page[HKS_NAND_PAGE_BYTES]) {
    uint32_t seq = 0u;
    uint8_t kind = 0u;
    uint8_t sub = 0u;
    uint16_t count = 0u;
    uint32_t ckpt_seq_start = 0u;
    if (!validate_index_page_hdr(page, &seq, &kind, &sub, &count, &ckpt_seq_start)) {
        return;
    }

    if (kind == HKS_IDX_KIND_OP) {
        index_op_t op;
        memset(&op, 0, sizeof(op));
        op.op = sub;
        if (op.op == HKS_INDEX_OP_DELETE) {
            op.session_id = hks_get_u64_le(page + HKS_IDX_PAGE_HDR_BYTES);
        } else if (op.op == HKS_INDEX_OP_BLOCK_ALLOC) {
            op.session_id = hks_get_u64_le(page + HKS_IDX_PAGE_HDR_BYTES + 0u);
            op.alloc_logical_idx = hks_get_u16_le(page + HKS_IDX_PAGE_HDR_BYTES + 8u);
            op.alloc_phys_block = hks_get_u16_le(page + HKS_IDX_PAGE_HDR_BYTES + 10u);
        } else {
            hks_flash_session_meta_t tmp;
            memset(&tmp, 0, sizeof(tmp));
            decode_meta_entry(page + HKS_IDX_PAGE_HDR_BYTES, &tmp, &op.flags);
            op.session_id = tmp.session_id;
            op.start_utc_ms = tmp.start_utc_ms;
            op.duration_ms = tmp.duration_ms;
            op.byte_size = tmp.byte_size;
            op.sample_count = tmp.sample_count;
            op.session_flags = tmp.session_flags;
            op.start_block = tmp.start_block;
            op.block_count = tmp.block_count;
            memcpy(op.whole_file_sha256, tmp.whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
        }
        apply_index_op(log, &op, NULL);
    } else if (kind == HKS_IDX_KIND_CKPT_META) {
        const size_t limit = (count <= HKS_IDX_META_PER_PAGE) ? (size_t)count : HKS_IDX_META_PER_PAGE;
        for (size_t i = 0u; i < limit; i++) {
            hks_flash_session_meta_t tmp;
            uint8_t flg = 0u;
            memset(&tmp, 0, sizeof(tmp));
            decode_meta_entry(page + HKS_IDX_PAGE_HDR_BYTES + i * HKS_IDX_META_ENTRY_BYTES, &tmp, &flg);
            size_t s = find_session_slot(log, tmp.session_id);
            if (s >= HKS_FLASH_MAX_SESSIONS) {
                s = alloc_session_slot(log);
                if (s >= HKS_FLASH_MAX_SESSIONS) {
                    continue;
                }
                log->sessions[s].valid = true;
                log->session_count++;
            }
            hks_flash_session_meta_t *m = &log->sessions[s];
            m->session_id = tmp.session_id;
            m->start_utc_ms = tmp.start_utc_ms;
            m->duration_ms = tmp.duration_ms;
            m->byte_size = tmp.byte_size;
            m->sample_count = tmp.sample_count;
            m->session_flags = tmp.session_flags;
            m->closed = tmp.closed;
            m->verified = tmp.verified;
            m->recovered = tmp.recovered;
            memcpy(m->whole_file_sha256, tmp.whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
        }
    } else if (kind == HKS_IDX_KIND_CKPT_CHAIN) {
        size_t pos = HKS_IDX_PAGE_HDR_BYTES;
        for (uint16_t seg = 0u; seg < count; seg++) {
            if (pos + HKS_IDX_CHAIN_SEG_HDR > HKS_IDX_PAGE_CRC_OFF) {
                break;
            }
            const uint64_t sid = hks_get_u64_le(page + pos + 0u);
            const uint16_t start_lidx = hks_get_u16_le(page + pos + 8u);
            const uint16_t seg_blks = hks_get_u16_le(page + pos + 10u);
            pos += HKS_IDX_CHAIN_SEG_HDR;
            if (pos + (size_t)seg_blks * 2u > HKS_IDX_PAGE_CRC_OFF) {
                break;
            }
            const size_t s = find_session_slot(log, sid);
            for (uint16_t k = 0u; k < seg_blks; k++) {
                const uint16_t phys = hks_get_u16_le(page + pos + (size_t)k * 2u);
                if (s < HKS_FLASH_MAX_SESSIONS &&
                    (uint32_t)start_lidx + (uint32_t)k == (uint32_t)log->sessions[s].block_count) {
                    (void)session_append_block(log, s, phys);
                }
            }
            pos += (size_t)seg_blks * 2u;
        }
    }
}

static int find_free_scratch_block(const hks_flash_log_t *log, uint16_t *blk_out) {
    for (uint16_t b = HKS_PART_RING_START; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        if (!log->bad_blocks[b] && !is_block_used_by_any_session(log, b)) {
            *blk_out = b;
            return 0;
        }
    }
    for (uint16_t b = HKS_PART_BBM_POOL_START; b < HKS_PART_BBM_POOL_START + HKS_PART_BBM_POOL_BLOCKS; b++) {
        if (!log->bad_blocks[b]) {
            *blk_out = b;
            return 0;
        }
    }
    return -1;
}

static int acquire_ring_block(hks_flash_log_t *log, uint16_t *blk_out);

static int append_footer_at_offset(hks_flash_log_t *log,
                                   size_t slot_idx,
                                   uint32_t valid_bytes,
                                   const uint8_t footer_frame[HKS_FRAME_SESSION_FOOTER_BYTES]) {
    hks_flash_session_meta_t *meta = &log->sessions[slot_idx];
    const uint16_t last_logical_blk = (uint16_t)(valid_bytes / HKS_NAND_BLOCK_BYTES);
    const uint32_t in_blk_off = valid_bytes % HKS_NAND_BLOCK_BYTES;
    const uint8_t last_pg = (uint8_t)(in_blk_off / HKS_NAND_PAGE_BYTES);
    const size_t tail = (size_t)(in_blk_off % HKS_NAND_PAGE_BYTES);

    uint16_t target_blk = 0u;
    if (last_logical_blk < meta->block_count) {
        if (!get_session_block_at_idx(log, meta, last_logical_blk, &target_blk)) {
            return -1;
        }
    } else if (last_logical_blk == meta->block_count) {
        if (acquire_ring_block(log, &target_blk) != 0) {
            return -1;
        }
        if (!session_append_block(log, slot_idx, target_blk)) {
            return -1;
        }
        if (index_append_block_alloc(log, meta->session_id, last_logical_blk, target_blk) != 0) {
            return -1;
        }
    } else {
        return -1;
    }

    uint8_t pg_buf[HKS_NAND_PAGE_BYTES];
    bool page_erased = false;
    if (tail == 0u) {
        if (last_pg == 0u && (valid_bytes == 0u || in_blk_off == 0u)) {
            if (hks_hal_nand_erase_block(target_blk) != 0) {
                return -1;
            }
            page_erased = true;
        } else {
            if (hks_hal_nand_read_page(target_blk, last_pg, 0u, pg_buf, sizeof(pg_buf)) != 0) {
                return -1;
            }
            page_erased = is_buf_erased(pg_buf, sizeof(pg_buf));
        }
    }

    if (tail == 0u && page_erased) {
        memset(pg_buf, 0xFF, sizeof(pg_buf));
        memcpy(pg_buf, footer_frame, HKS_FRAME_SESSION_FOOTER_BYTES);
        if (hks_hal_nand_prog_page(target_blk, last_pg, 0u, pg_buf, sizeof(pg_buf)) != 0) {
            return -1;
        }
        return 0;
    }

    /* Non-zero page offset or torn tail bytes: rewrite last block cleanly via scratch block */
    uint16_t scratch_blk = 0u;
    if (find_free_scratch_block(log, &scratch_blk) != 0) {
        return -1;
    }
    if (hks_hal_nand_erase_block(scratch_blk) != 0) {
        return -1;
    }

    for (uint8_t p = 0u; p < last_pg; p++) {
        if (hks_hal_nand_read_page(target_blk, p, 0u, pg_buf, sizeof(pg_buf)) != 0) {
            return -1;
        }
        if (hks_hal_nand_prog_page(scratch_blk, p, 0u, pg_buf, sizeof(pg_buf)) != 0) {
            return -1;
        }
    }

    uint8_t last_buf[HKS_NAND_PAGE_BYTES];
    memset(last_buf, 0xFF, sizeof(last_buf));
    if (tail > 0u) {
        if (hks_hal_nand_read_page(target_blk, last_pg, 0u, last_buf, tail) != 0) {
            return -1;
        }
    }

    const size_t first_part = (tail + HKS_FRAME_SESSION_FOOTER_BYTES <= HKS_NAND_PAGE_BYTES)
                                  ? HKS_FRAME_SESSION_FOOTER_BYTES
                                  : (HKS_NAND_PAGE_BYTES - tail);
    memcpy(last_buf + tail, footer_frame, first_part);
    if (hks_hal_nand_prog_page(scratch_blk, last_pg, 0u, last_buf, sizeof(last_buf)) != 0) {
        return -1;
    }

    uint8_t total_pages_in_scratch = (uint8_t)(last_pg + 1u);
    const size_t rem = HKS_FRAME_SESSION_FOOTER_BYTES - first_part;
    bool spilled_next_block = false;
    uint8_t spill_buf[HKS_NAND_PAGE_BYTES];
    if (rem > 0u) {
        memset(spill_buf, 0xFF, sizeof(spill_buf));
        memcpy(spill_buf, footer_frame + first_part, rem);
        if (last_pg + 1u < HKS_NAND_PAGES_PER_BLOCK) {
            if (hks_hal_nand_prog_page(scratch_blk, (uint8_t)(last_pg + 1u), 0u, spill_buf, sizeof(spill_buf)) != 0) {
                return -1;
            }
            total_pages_in_scratch = (uint8_t)(last_pg + 2u);
        } else {
            spilled_next_block = true;
        }
    }

    if (hks_hal_nand_erase_block(target_blk) != 0) {
        return -1;
    }
    for (uint8_t p = 0u; p < total_pages_in_scratch; p++) {
        if (hks_hal_nand_read_page(scratch_blk, p, 0u, pg_buf, sizeof(pg_buf)) != 0) {
            return -1;
        }
        if (hks_hal_nand_prog_page(target_blk, p, 0u, pg_buf, sizeof(pg_buf)) != 0) {
            return -1;
        }
    }
    (void)hks_hal_nand_erase_block(scratch_blk);

    if (spilled_next_block) {
        uint16_t spill_blk = 0u;
        const uint16_t spill_lidx = (uint16_t)(last_logical_blk + 1u);
        if (spill_lidx < meta->block_count) {
            if (!get_session_block_at_idx(log, meta, spill_lidx, &spill_blk)) {
                return -1;
            }
        } else {
            if (acquire_ring_block(log, &spill_blk) != 0) {
                return -1;
            }
            if (!session_append_block(log, slot_idx, spill_blk)) {
                return -1;
            }
            if (index_append_block_alloc(log, meta->session_id, spill_lidx, spill_blk) != 0) {
                return -1;
            }
        }
        if (hks_hal_nand_erase_block(spill_blk) != 0) {
            return -1;
        }
        if (hks_hal_nand_prog_page(spill_blk, 0u, 0u, spill_buf, sizeof(spill_buf)) != 0) {
            return -1;
        }
    }
    return 0;
}

static int recover_open_session(hks_flash_log_t *log, size_t slot_idx) {
    hks_flash_session_meta_t *meta = &log->sessions[slot_idx];
    uint8_t win[HKS_MAX_FRAME_BYTES + HKS_NAND_PAGE_BYTES];
    size_t win_len = 0u;

    uint16_t cur_logical_blk = 0u;
    uint16_t cur_phys_blk = 0xFFFFu;
    uint8_t cur_pg = 0u;
    bool eof_flash = false;

    hks_sha256_ctx_t sha_ctx;
    hks_sha256_init(&sha_ctx);

    uint32_t valid_bytes = 0u;
    uint32_t imu_samples = 0u;
    uint32_t hg_samples = 0u;
    uint32_t aux_samples = 0u;
    uint64_t first_ts_us = 0u;
    uint64_t last_ts_us = 0u;
    bool have_ts = false;
    uint16_t last_seq = 0u;
    uint8_t last_flags = 0u;
    bool saw_footer = false;
    uint64_t start_utc_ms = meta->start_utc_ms;

    while (1) {
        while (!eof_flash && win_len < HKS_MAX_FRAME_BYTES) {
            if (cur_pg == 0u) {
                if (cur_logical_blk >= meta->block_count ||
                    !get_session_block_at_idx(log, meta, cur_logical_blk, &cur_phys_blk)) {
                    eof_flash = true;
                    break;
                }
            }
            if (hks_hal_nand_read_page(cur_phys_blk, cur_pg, 0u,
                                       win + win_len, HKS_NAND_PAGE_BYTES) != 0) {
                eof_flash = true;
                break;
            }
            if (is_buf_erased(win + win_len, HKS_NAND_PAGE_BYTES)) {
                eof_flash = true;
                break;
            }
            win_len += HKS_NAND_PAGE_BYTES;
            cur_pg++;
            if (cur_pg >= HKS_NAND_PAGES_PER_BLOCK) {
                cur_pg = 0u;
                cur_logical_blk++;
            }
        }

        if (win_len < HKS_FRAME_HDR_BYTES) {
            break;
        }
        hks_frame_hdr_t hdr;
        if (!hks_frame_decode_hdr(win, win_len, &hdr)) {
            break;
        }
        const size_t frame_len = (size_t)HKS_FRAME_OVERHEAD_BYTES + (size_t)hdr.payload_len;
        if (win_len < frame_len || !hks_frame_verify(win, frame_len, &hdr)) {
            break;
        }

        const uint8_t *payload = win + HKS_FRAME_HDR_BYTES;
        if (hdr.msg_type == HKS_MSG_SESSION_HEADER) {
            hks_session_header_t sh;
            if (hks_decode_session_header(payload, hdr.payload_len, &sh)) {
                start_utc_ms = sh.start_utc_ms;
            }
        } else if (hdr.msg_type == HKS_MSG_IMU_BATCH) {
            hks_imu_batch_hdr_t bh;
            if (hks_decode_imu_batch(payload, hdr.payload_len, &bh, NULL, 0u)) {
                imu_samples += bh.sample_count;
            }
        } else if (hdr.msg_type == HKS_MSG_HIGH_G_BATCH) {
            hks_high_g_batch_hdr_t hgh;
            if (hks_decode_high_g_batch(payload, hdr.payload_len, &hgh, NULL, 0u)) {
                hg_samples += hgh.sample_count;
            }
        } else if (hdr.msg_type == HKS_MSG_AUX_ENV) {
            aux_samples++;
        }

        if (!have_ts) {
            first_ts_us = hdr.timestamp_us;
            have_ts = true;
        }
        last_ts_us = hdr.timestamp_us;
        last_seq = hdr.seq;
        last_flags = hdr.flags;

        hks_sha256_update(&sha_ctx, win, frame_len);
        valid_bytes += (uint32_t)frame_len;

        memmove(win, win + frame_len, win_len - frame_len);
        win_len -= frame_len;

        if (hdr.msg_type == HKS_MSG_SESSION_FOOTER) {
            saw_footer = true;
            break;
        }
    }

    if (valid_bytes == 0u) {
        /* Nothing valid written; discard orphan OPEN entry and free its blocks */
        const uint64_t del_sid = meta->session_id;
        session_clear_chain(log, slot_idx);
        meta->valid = false;
        if (log->session_count > 0u) {
            log->session_count--;
        }
        index_op_t del_op;
        memset(&del_op, 0, sizeof(del_op));
        del_op.op = HKS_INDEX_OP_DELETE;
        del_op.session_id = del_sid;
        (void)index_append_op(log, &del_op);
        return 0;
    }

    if (!saw_footer) {
        hks_sha256_ctx_t pre_ctx = sha_ctx;
        hks_session_footer_t ftr;
        memset(&ftr, 0, sizeof(ftr));
        const uint64_t dur_ms = (last_ts_us >= first_ts_us) ? ((last_ts_us - first_ts_us) / 1000ULL) : 0ULL;
        ftr.end_utc_ms = (start_utc_ms > 0u) ? (start_utc_ms + dur_ms) : 0u;
        ftr.total_imu_samples = imu_samples;
        ftr.total_high_g_samples = hg_samples;
        ftr.total_aux_samples = aux_samples;
        ftr.end_reason = HKS_END_WATCHDOG_RECOVER;
        ftr.total_bytes = (uint64_t)valid_bytes + (uint64_t)HKS_FRAME_SESSION_FOOTER_BYTES;
        hks_sha256_final(&pre_ctx, ftr.pre_footer_sha256);

        uint8_t ftr_payload[HKS_PAYLOAD_SESSION_FOOTER_BYTES];
        uint8_t ftr_frame[HKS_FRAME_SESSION_FOOTER_BYTES];
        (void)hks_encode_session_footer(ftr_payload, sizeof(ftr_payload), &ftr);
        (void)hks_frame_encode(ftr_frame, sizeof(ftr_frame),
                               HKS_MSG_SESSION_FOOTER, last_flags,
                               (uint16_t)(last_seq + 1u), last_ts_us,
                               ftr_payload, HKS_PAYLOAD_SESSION_FOOTER_BYTES);

        if (append_footer_at_offset(log, slot_idx, valid_bytes, ftr_frame) != 0) {
            return -1;
        }
        hks_sha256_update(&sha_ctx, ftr_frame, sizeof(ftr_frame));
        valid_bytes += HKS_FRAME_SESSION_FOOTER_BYTES;
    }

    uint16_t final_blocks_used = (uint16_t)((valid_bytes + HKS_NAND_BLOCK_BYTES - 1u) / HKS_NAND_BLOCK_BYTES);
    if (final_blocks_used == 0u) {
        final_blocks_used = 1u;
    }
    if (meta->block_count > final_blocks_used) {
        session_trim_to_blocks(log, slot_idx, final_blocks_used);
    }

    uint8_t whole_sha[HKS_SHA256_DIGEST_BYTES];
    hks_sha256_final(&sha_ctx, whole_sha);

    meta->start_utc_ms = start_utc_ms;
    meta->duration_ms = (last_ts_us >= first_ts_us)
                            ? (uint32_t)((last_ts_us - first_ts_us) / 1000ULL)
                            : 0u;
    meta->byte_size = valid_bytes;
    meta->sample_count = imu_samples;
    meta->closed = true;
    meta->recovered = true;
    meta->session_flags |= HKS_SESS_FLAG_RECOVERED;
    memcpy(meta->whole_file_sha256, whole_sha, HKS_SHA256_DIGEST_BYTES);

    index_op_t close_op;
    memset(&close_op, 0, sizeof(close_op));
    close_op.op = HKS_INDEX_OP_CLOSE;
    close_op.flags = (uint8_t)(HKS_SESS_FLAG_CLOSED | HKS_SESS_FLAG_RECOVERED);
    close_op.session_id = meta->session_id;
    close_op.start_utc_ms = meta->start_utc_ms;
    close_op.duration_ms = meta->duration_ms;
    close_op.byte_size = meta->byte_size;
    close_op.sample_count = meta->sample_count;
    close_op.session_flags = meta->session_flags;
    close_op.start_block = meta->start_block;
    close_op.block_count = meta->block_count;
    memcpy(close_op.whole_file_sha256, meta->whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
    return index_append_op(log, &close_op);
}

int hks_flash_log_init(hks_flash_log_t *log) {
    if (log == NULL) {
        return -1;
    }
    memset(log, 0, sizeof(*log));
    log->state = HKS_FLASH_STATE_UNINIT;
    log->ring_cursor_block = HKS_PART_RING_START;
    for (uint16_t b = 0u; b < HKS_NAND_TOTAL_BLOCKS; b++) {
        log->block_next[b] = 0xFFFFu;
        log->block_owner[b] = 0xFFu;
    }
    for (size_t s = 0u; s < HKS_FLASH_MAX_SESSIONS; s++) {
        log->session_tail_block[s] = 0xFFFFu;
    }
    log->rec_session_slot = 0xFFu;
    log->rec_reserved_block = 0xFFFFu;

    if (hks_hal_nand_init() != 0) {
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }

    for (uint16_t b = 0u; b < HKS_NAND_TOTAL_BLOCKS; b++) {
        bool bad = false;
        if (hks_hal_nand_is_bad_block(b, &bad) != 0) {
            bad = true;
        }
        log->bad_blocks[b] = bad;
        if (bad) {
            log->bad_block_count++;
        }
    }

    /* Load FACTORY_CAL if present (blocks 0..1) */
    hks_cal_block_t cal_tmp;
    if (hks_flash_log_read_factory_cal(log, &cal_tmp) == 0) {
        log->active_cal = cal_tmp;
        log->has_cal = true;
    } else {
        hks_cal_block_init_default(&log->active_cal, 0u);
        log->has_cal = false;
    }

    /* Scan SESSION_INDEX (blocks 2..5) for all valid pages */
    index_loc_t locs[HKS_PART_SESSION_INDEX_BLOCKS * HKS_NAND_PAGES_PER_BLOCK];
    size_t loc_count = 0u;
    uint16_t first_good_idx_blk = 0xFFFFu;

    for (uint16_t b = HKS_PART_SESSION_INDEX_START;
         b < HKS_PART_SESSION_INDEX_START + HKS_PART_SESSION_INDEX_BLOCKS; b++) {
        if (log->bad_blocks[b]) {
            continue;
        }
        if (first_good_idx_blk == 0xFFFFu) {
            first_good_idx_blk = b;
        }
        for (uint8_t p = 0u; p < HKS_NAND_PAGES_PER_BLOCK; p++) {
            uint8_t page_buf[HKS_NAND_PAGE_BYTES];
            if (hks_hal_nand_read_page(b, p, 0u, page_buf, sizeof(page_buf)) != 0) {
                continue;
            }
            uint32_t seq = 0u;
            uint8_t kind = 0u;
            uint8_t sub = 0u;
            uint32_t ckpt_s0 = 0u;
            if (validate_index_page_hdr(page_buf, &seq, &kind, &sub, NULL, &ckpt_s0)) {
                locs[loc_count].seq = seq;
                locs[loc_count].ckpt_seq_start = ckpt_s0;
                locs[loc_count].block = b;
                locs[loc_count].page = p;
                locs[loc_count].kind = kind;
                locs[loc_count].sub = sub;
                loc_count++;
            }
        }
    }

    if (first_good_idx_blk == 0xFFFFu) {
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }

    /* Insertion sort by ascending seq */
    for (size_t i = 1u; i < loc_count; i++) {
        index_loc_t key = locs[i];
        size_t j = i;
        while (j > 0u && locs[j - 1u].seq > key.seq) {
            locs[j] = locs[j - 1u];
            j--;
        }
        locs[j] = key;
    }

    bool have_complete_ckpt = false;
    size_t complete_ckpt_start = 0u;
    size_t complete_ckpt_pages = 0u;
    for (size_t i = 0u; i < loc_count; i++) {
        if (locs[i].kind == HKS_IDX_KIND_CKPT_META &&
            locs[i].seq == locs[i].ckpt_seq_start &&
            locs[i].sub >= 1u) {
            const size_t total_p = (size_t)locs[i].sub;
            if (i + total_p <= loc_count) {
                bool ok = true;
                for (size_t k = 0u; k < total_p; k++) {
                    const index_loc_t *pl = &locs[i + k];
                    if (pl->seq != locs[i].seq + (uint32_t)k ||
                        (pl->kind != HKS_IDX_KIND_CKPT_META &&
                         pl->kind != HKS_IDX_KIND_CKPT_CHAIN) ||
                        pl->ckpt_seq_start != locs[i].ckpt_seq_start ||
                        pl->sub != locs[i].sub) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    have_complete_ckpt = true;
                    complete_ckpt_start = i;
                    complete_ckpt_pages = total_p;
                }
            }
        }
    }

    const size_t replay_from = have_complete_ckpt ? complete_ckpt_start : 0u;
    const size_t ckpt_end = have_complete_ckpt ? (complete_ckpt_start + complete_ckpt_pages) : 0u;
    log->index_has_unckpt_ops = false;
    for (size_t i = replay_from; i < loc_count; i++) {
        if (locs[i].kind != HKS_IDX_KIND_OP && (!have_complete_ckpt || i >= ckpt_end)) {
            continue;
        }
        uint8_t page_buf[HKS_NAND_PAGE_BYTES];
        if (hks_hal_nand_read_page(locs[i].block, locs[i].page, 0u, page_buf, sizeof(page_buf)) == 0) {
            apply_index_page(log, page_buf);
            if (locs[i].kind == HKS_IDX_KIND_OP) {
                log->index_has_unckpt_ops = true;
            }
        }
    }

    if (loc_count > 0u) {
        const index_loc_t last = locs[loc_count - 1u];
        log->index_seq = last.seq;
        log->index_cur_block = last.block;
        log->index_cur_page = (uint8_t)(last.page + 1u);
    } else {
        log->index_seq = 0u;
        log->index_cur_block = first_good_idx_blk;
        log->index_cur_page = 0u;
    }

    /* Advance ring cursor past any occupied blocks */
    for (uint16_t step = 0u; step < HKS_PART_RING_BLOCKS; step++) {
        const uint16_t cand = (uint16_t)(HKS_PART_RING_START + step);
        if (!log->bad_blocks[cand] && !is_block_used_by_any_session(log, cand)) {
            log->ring_cursor_block = cand;
            break;
        }
    }

    /* Recover any unclosed OPEN sessions via WATCHDOG_RECOVER */
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        if (log->sessions[i].valid && !log->sessions[i].closed) {
            if (recover_open_session(log, i) != 0) {
                log->state = HKS_FLASH_STATE_ERROR;
                return -1;
            }
        }
    }

    log->state = HKS_FLASH_STATE_IDLE;
    return 0;
}

int hks_flash_log_write_factory_cal(hks_flash_log_t *log, const hks_cal_block_t *cal) {
    if (log == NULL || cal == NULL) {
        return -1;
    }
    uint8_t page[HKS_NAND_PAGE_BYTES];
    memset(page, 0xFF, sizeof(page));
    hks_put_u32_le(page + 0u, HKS_FCAL_MAGIC);
    hks_put_u16_le(page + 4u, 1u); /* version */
    hks_put_u16_le(page + 6u, 0u);
    hks_put_u64_le(page + 8u, hks_hal_device_id());
    hks_put_u32_le(page + 16u, 0u);
    hks_put_u32_le(page + 20u, 0u);
    if (hks_encode_cal_block(page + 24u, HKS_PAYLOAD_CAL_BLOCK_BYTES, cal, true) == 0u) {
        return -1;
    }
    const uint32_t rec_crc = hks_crc32(page, HKS_FCAL_REC_BYTES);
    hks_put_u32_le(page + 20u, rec_crc);

    size_t wrote = 0u;
    for (uint16_t b = HKS_PART_FACTORY_CAL_START;
         b < HKS_PART_FACTORY_CAL_START + HKS_PART_FACTORY_CAL_BLOCKS; b++) {
        if (log->bad_blocks[b]) {
            continue;
        }
        if (hks_hal_nand_erase_block(b) != 0 ||
            hks_hal_nand_prog_page(b, 0u, 0u, page, sizeof(page)) != 0) {
            log->bad_blocks[b] = true;
            (void)hks_hal_nand_mark_bad_block(b);
            continue;
        }
        wrote++;
    }
    if (wrote == 0u) {
        return -1;
    }
    log->active_cal = *cal;
    log->has_cal = true;
    return 0;
}

int hks_flash_log_read_factory_cal(hks_flash_log_t *log, hks_cal_block_t *cal_out) {
    if (log == NULL || cal_out == NULL) {
        return -1;
    }
    uint8_t page[HKS_FCAL_REC_BYTES];
    for (uint16_t b = HKS_PART_FACTORY_CAL_START;
         b < HKS_PART_FACTORY_CAL_START + HKS_PART_FACTORY_CAL_BLOCKS; b++) {
        if (log->bad_blocks[b]) {
            continue;
        }
        if (hks_hal_nand_read_page(b, 0u, 0u, page, sizeof(page)) != 0) {
            continue;
        }
        if (hks_get_u32_le(page + 0u) != HKS_FCAL_MAGIC) {
            continue;
        }
        const uint32_t stored_crc = hks_get_u32_le(page + 20u);
        uint8_t chk[HKS_FCAL_REC_BYTES];
        memcpy(chk, page, sizeof(chk));
        memset(chk + 20u, 0, 4u);
        if (hks_crc32(chk, sizeof(chk)) != stored_crc) {
            continue;
        }
        if (hks_decode_cal_block(page + 24u, HKS_PAYLOAD_CAL_BLOCK_BYTES, cal_out, true)) {
            return 0;
        }
    }
    return -1;
}

static hks_status_code_t delete_session_internal(hks_flash_log_t *log,
                                                 uint64_t session_id,
                                                 bool force_flag,
                                                 uint32_t *freed_bytes_out);

static int reclaim_oldest_verified_session(hks_flash_log_t *log) {
    hks_flash_session_meta_t *oldest = NULL;
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        hks_flash_session_meta_t *m = &log->sessions[i];
        if (!m->valid || !m->closed || !m->verified) {
            continue;
        }
        if (oldest == NULL || m->session_id < oldest->session_id) {
            oldest = m;
        }
    }
    if (oldest == NULL) {
        return -1;
    }
    uint32_t freed = 0u;
    if (delete_session_internal(log, oldest->session_id, false, &freed) != HKS_STATUS_OK) {
        return -1;
    }
    return 0;
}

static int acquire_ring_block(hks_flash_log_t *log, uint16_t *blk_out) {
    for (int attempt = 0; attempt < 2; attempt++) {
        uint16_t b = log->ring_cursor_block;
        if (!is_valid_ring_block(b)) {
            b = HKS_PART_RING_START;
        }
        for (uint16_t step = 0u; step < HKS_PART_RING_BLOCKS; step++) {
            if (!log->bad_blocks[b] && !is_block_used_by_any_session(log, b)) {
                if (hks_hal_nand_erase_block(b) != 0) {
                    log->bad_blocks[b] = true;
                    log->bad_block_count++;
                    (void)hks_hal_nand_mark_bad_block(b);
                } else {
                    uint16_t next_b = next_ring_block(log, b);
                    log->ring_cursor_block = (next_b != 0xFFFFu) ? next_b : HKS_PART_RING_START;
                    *blk_out = b;
                    return 0;
                }
            }
            b++;
            if (b >= HKS_PART_RING_START + HKS_PART_RING_BLOCKS) {
                b = HKS_PART_RING_START;
            }
        }
        if (reclaim_oldest_verified_session(log) != 0) {
            break;
        }
    }
    return -1;
}

static int advance_recording_block(hks_flash_log_t *log) {
    uint16_t next_b = 0xFFFFu;
    if (log->rec_reserved_block != 0xFFFFu) {
        next_b = log->rec_reserved_block;
        log->rec_reserved_block = 0xFFFFu;
        log->block_owner[next_b] = 0xFFu;
    } else {
        if (acquire_ring_block(log, &next_b) != 0) {
            return -2;
        }
    }
    if (log->rec_session_slot >= HKS_FLASH_MAX_SESSIONS) {
        return -1;
    }
    const uint16_t logical_idx = log->sessions[log->rec_session_slot].block_count;
    if (!session_append_block(log, log->rec_session_slot, next_b)) {
        return -1;
    }
    log->rec_cur_block = next_b;
    log->rec_cur_page = 0u;
    log->rec_block_count = log->sessions[log->rec_session_slot].block_count;

    if (index_append_block_alloc(log, log->rec_session_id, logical_idx, next_b) != 0) {
        return -1;
    }
    return 0;
}

static int flush_page_buffer(hks_flash_log_t *log, bool pad_ff) {
    if (log->page_fill == 0u) {
        return 0;
    }
    if (pad_ff && log->page_fill < HKS_NAND_PAGE_BYTES) {
        memset(log->page_buf + log->page_fill, 0xFF, HKS_NAND_PAGE_BYTES - log->page_fill);
    }

    if (log->rec_cur_page >= HKS_NAND_PAGES_PER_BLOCK) {
        const int arc = advance_recording_block(log);
        if (arc != 0) {
            return arc;
        }
    }

    if (hks_hal_nand_prog_page(log->rec_cur_block, log->rec_cur_page, 0u,
                               log->page_buf, HKS_NAND_PAGE_BYTES) != 0) {
        return -1;
    }
    log->rec_cur_page++;
    log->page_fill = 0u;
    return 0;
}

static int write_raw_frame_bytes(hks_flash_log_t *log, const uint8_t *frame, size_t len, bool is_footer) {
    if (!is_footer) {
        /* Reserve headroom across entire current block so 96 B 0x07 footer always fits on FLASH_FULL,
         * and pre-acquire next block BEFORE mutating SHA-256 / CRC32 / page buffer state. */
        const size_t blk_used = (size_t)log->rec_cur_page * HKS_NAND_PAGE_BYTES + log->page_fill;
        const size_t blk_rem = (blk_used < HKS_NAND_BLOCK_BYTES) ? (HKS_NAND_BLOCK_BYTES - blk_used) : 0u;
        if (blk_rem < len + HKS_FRAME_SESSION_FOOTER_BYTES && log->rec_reserved_block == 0xFFFFu) {
            uint16_t res_b = 0u;
            if (acquire_ring_block(log, &res_b) != 0) {
                return -2;
            }
            log->block_owner[res_b] = 0xFEu;
            log->rec_reserved_block = res_b;
        }
        hks_sha256_update(&log->pre_footer_sha_ctx, frame, len);
    }
    log->file_crc_state = hks_crc32_update(log->file_crc_state, frame, len);

    size_t off = 0u;
    while (off < len) {
        if (log->rec_cur_page >= HKS_NAND_PAGES_PER_BLOCK && log->page_fill == 0u) {
            const int arc = advance_recording_block(log);
            if (arc != 0) {
                return arc;
            }
        }
        const size_t space = HKS_NAND_PAGE_BYTES - log->page_fill;
        const size_t chunk = (len - off < space) ? (len - off) : space;
        memcpy(log->page_buf + log->page_fill, frame + off, chunk);
        log->page_fill += chunk;
        off += chunk;

        if (log->page_fill == HKS_NAND_PAGE_BYTES) {
            const int rc = flush_page_buffer(log, false);
            if (rc != 0) {
                return rc;
            }
        }
    }

    log->rec_bytes_written += (uint32_t)len;
    return 0;
}

int hks_flash_log_start_session(hks_flash_log_t *log,
                                const hks_session_header_t *hdr,
                                const hks_cal_block_t *cal_opt,
                                uint8_t frame_flags,
                                uint64_t start_ts_us) {
    if (log == NULL || hdr == NULL || log->state != HKS_FLASH_STATE_IDLE) {
        return -1;
    }
    if (find_session_const(log, hdr->session_id) != NULL) {
        return -1;
    }
    if (count_active_sessions(log) >= HKS_FLASH_MAX_SESSIONS) {
        if (reclaim_oldest_verified_session(log) != 0) {
            return -1;
        }
    }

    uint16_t start_blk = 0u;
    if (acquire_ring_block(log, &start_blk) != 0) {
        return -2;
    }
    log->block_owner[start_blk] = 0xFEu;
    log->block_next[start_blk] = 0xFFFFu;

    hks_cal_block_t cal;
    if (cal_opt != NULL) {
        cal = *cal_opt;
    } else if (log->has_cal) {
        cal = log->active_cal;
    } else {
        hks_cal_block_init_default(&cal, hdr->start_utc_ms);
    }

    uint8_t cal_payload[HKS_PAYLOAD_CAL_BLOCK_BYTES];
    (void)hks_encode_cal_block(cal_payload, sizeof(cal_payload), &cal, true);
    cal.cal_crc32 = hks_get_u32_le(cal_payload + HKS_CAL_BLOCK_CRC_BYTES);

    hks_session_header_t sh = *hdr;
    sh.cal_crc32 = cal.cal_crc32;

    uint8_t sh_payload[HKS_PAYLOAD_SESSION_HEADER_BYTES];
    uint8_t sh_frame[HKS_FRAME_SESSION_HEADER_BYTES];
    (void)hks_encode_session_header(sh_payload, sizeof(sh_payload), &sh);
    (void)hks_frame_encode(sh_frame, sizeof(sh_frame),
                           HKS_MSG_SESSION_HEADER, frame_flags, 0u, start_ts_us,
                           sh_payload, HKS_PAYLOAD_SESSION_HEADER_BYTES);

    uint8_t cal_frame[HKS_FRAME_CAL_BLOCK_BYTES];
    (void)hks_frame_encode(cal_frame, sizeof(cal_frame),
                           HKS_MSG_CALIBRATION_BLOCK, frame_flags, 1u, start_ts_us,
                           cal_payload, HKS_PAYLOAD_CAL_BLOCK_BYTES);

    log->rec_session_slot = 0xFFu;
    log->rec_reserved_block = 0xFFFFu;
    log->rec_session_id = sh.session_id;
    log->rec_start_utc_ms = sh.start_utc_ms;
    log->rec_start_ts_us = start_ts_us;
    log->rec_last_ts_us = start_ts_us;
    log->rec_seq = 2u;
    log->rec_start_block = start_blk;
    log->rec_cur_block = start_blk;
    log->rec_cur_page = 0u;
    log->rec_block_count = 1u;
    log->rec_bytes_written = 0u;
    log->rec_imu_samples = 0u;
    log->rec_dropped_imu_samples = 0u;
    log->rec_high_g_samples = 0u;
    log->rec_dropped_high_g_samples = 0u;
    log->rec_aux_samples = 0u;
    log->rec_last_flags = frame_flags;
    log->page_fill = 0u;
    hks_sha256_init(&log->pre_footer_sha_ctx);
    log->file_crc_state = hks_crc32_init();
    log->state = HKS_FLASH_STATE_RECORDING;

    /* Record OPEN in SESSION_INDEX first for power-loss recovery */
    index_op_t open_op;
    memset(&open_op, 0, sizeof(open_op));
    open_op.op = HKS_INDEX_OP_OPEN;
    open_op.flags = 0u;
    open_op.session_id = sh.session_id;
    open_op.start_utc_ms = sh.start_utc_ms;
    open_op.start_block = start_blk;
    open_op.block_count = 1u;
    if (index_append_op(log, &open_op) != 0) {
        log->block_owner[start_blk] = 0xFFu;
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }
    size_t rec_slot = HKS_FLASH_MAX_SESSIONS;
    apply_index_op(log, &open_op, &rec_slot);
    if (rec_slot >= HKS_FLASH_MAX_SESSIONS) {
        log->block_owner[start_blk] = 0xFFu;
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }
    log->rec_session_slot = (uint8_t)rec_slot;

    if (write_raw_frame_bytes(log, sh_frame, sizeof(sh_frame), false) != 0 ||
        write_raw_frame_bytes(log, cal_frame, sizeof(cal_frame), false) != 0) {
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }

    return 0;
}

static int append_frame_with_drop_counts(hks_flash_log_t *log,
                                         uint8_t msg_type,
                                         uint8_t flags,
                                         uint64_t timestamp_us,
                                         const uint8_t *payload,
                                         uint16_t payload_len,
                                         uint32_t dropped_imu_if_full,
                                         uint32_t dropped_high_g_if_full) {
    if (log == NULL || log->state != HKS_FLASH_STATE_RECORDING) {
        return -1;
    }
    uint8_t frame_buf[HKS_MAX_FRAME_BYTES];
    const size_t flen = hks_frame_encode(frame_buf, sizeof(frame_buf),
                                         msg_type, flags, log->rec_seq,
                                         timestamp_us, payload, payload_len);
    if (flen == 0u) {
        return -1;
    }
    const int rc = write_raw_frame_bytes(log, frame_buf, flen, false);
    if (rc == -2) {
        /* Increment dropped counters BEFORE sealing 0x07 FLASH_FULL footer */
        log->rec_dropped_imu_samples += dropped_imu_if_full;
        log->rec_dropped_high_g_samples += dropped_high_g_if_full;
        const uint64_t dur_ms = (timestamp_us >= log->rec_start_ts_us)
                                    ? ((timestamp_us - log->rec_start_ts_us) / 1000ULL)
                                    : 0ULL;
        const uint64_t end_utc = (log->rec_start_utc_ms > 0u) ? (log->rec_start_utc_ms + dur_ms) : 0u;
        (void)hks_flash_log_stop_session(log, HKS_END_FLASH_FULL, end_utc, timestamp_us);
        return -2;
    }
    if (rc != 0) {
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }

    log->rec_seq++;
    log->rec_last_ts_us = timestamp_us;
    log->rec_last_flags = flags;
    return 0;
}

int hks_flash_log_append_frame(hks_flash_log_t *log,
                               uint8_t msg_type,
                               uint8_t flags,
                               uint64_t timestamp_us,
                               const uint8_t *payload,
                               uint16_t payload_len) {
    return append_frame_with_drop_counts(log, msg_type, flags, timestamp_us,
                                         payload, payload_len, 0u, 0u);
}

int hks_flash_log_append_imu_batch(hks_flash_log_t *log,
                                   uint8_t flags,
                                   uint64_t timestamp_us,
                                   const hks_imu_batch_hdr_t *batch_hdr,
                                   const hks_imu_sample_t *samples) {
    if (log == NULL || batch_hdr == NULL || samples == NULL) {
        return -1;
    }
    uint8_t payload[HKS_MAX_PAYLOAD_BYTES];
    const size_t plen = hks_encode_imu_batch(payload, sizeof(payload), batch_hdr, samples);
    if (plen == 0u) {
        return -1;
    }
    const int rc = append_frame_with_drop_counts(log, HKS_MSG_IMU_BATCH, flags,
                                                 timestamp_us, payload, (uint16_t)plen,
                                                 batch_hdr->sample_count, 0u);
    if (rc == 0) {
        log->rec_imu_samples += batch_hdr->sample_count;
    }
    return rc;
}

int hks_flash_log_append_high_g_batch(hks_flash_log_t *log,
                                      uint8_t flags,
                                      uint64_t timestamp_us,
                                      const hks_high_g_batch_hdr_t *batch_hdr,
                                      const hks_high_g_sample_t *samples) {
    if (log == NULL || batch_hdr == NULL || samples == NULL) {
        return -1;
    }
    uint8_t payload[HKS_MAX_PAYLOAD_BYTES];
    const size_t plen = hks_encode_high_g_batch(payload, sizeof(payload), batch_hdr, samples);
    if (plen == 0u) {
        return -1;
    }
    const int rc = append_frame_with_drop_counts(log, HKS_MSG_HIGH_G_BATCH, flags,
                                                 timestamp_us, payload, (uint16_t)plen,
                                                 0u, batch_hdr->sample_count);
    if (rc == 0) {
        log->rec_high_g_samples += batch_hdr->sample_count;
    }
    return rc;
}

int hks_flash_log_append_aux_env(hks_flash_log_t *log,
                                 uint8_t flags,
                                 uint64_t timestamp_us,
                                 const hks_aux_env_t *aux) {
    if (log == NULL || aux == NULL) {
        return -1;
    }
    uint8_t payload[HKS_PAYLOAD_AUX_ENV_BYTES];
    const size_t plen = hks_encode_aux_env(payload, sizeof(payload), aux);
    if (plen == 0u) {
        return -1;
    }
    const int rc = hks_flash_log_append_frame(log, HKS_MSG_AUX_ENV, flags,
                                              timestamp_us, payload, (uint16_t)plen);
    if (rc == 0) {
        log->rec_aux_samples++;
    }
    return rc;
}

int hks_flash_log_append_event(hks_flash_log_t *log,
                               uint8_t flags,
                               uint64_t timestamp_us,
                               const hks_event_marker_t *evt) {
    if (log == NULL || evt == NULL) {
        return -1;
    }
    uint8_t payload[HKS_PAYLOAD_EVENT_MARKER_BYTES];
    const size_t plen = hks_encode_event_marker(payload, sizeof(payload), evt);
    if (plen == 0u) {
        return -1;
    }
    return hks_flash_log_append_frame(log, HKS_MSG_EVENT_MARKER, flags,
                                      timestamp_us, payload, (uint16_t)plen);
}

int hks_flash_log_stop_session(hks_flash_log_t *log,
                               hks_end_reason_t reason,
                               uint64_t end_utc_ms,
                               uint64_t end_ts_us) {
    if (log == NULL || log->state != HKS_FLASH_STATE_RECORDING) {
        return -1;
    }

    if (end_ts_us < log->rec_last_ts_us) {
        end_ts_us = log->rec_last_ts_us;
    }

    hks_sha256_ctx_t whole_ctx = log->pre_footer_sha_ctx;

    hks_session_footer_t ftr;
    memset(&ftr, 0, sizeof(ftr));
    ftr.end_utc_ms = end_utc_ms;
    ftr.total_imu_samples = log->rec_imu_samples;
    ftr.dropped_imu_samples = log->rec_dropped_imu_samples;
    ftr.total_high_g_samples = log->rec_high_g_samples;
    ftr.dropped_high_g_samples = log->rec_dropped_high_g_samples;
    ftr.total_aux_samples = log->rec_aux_samples;
    ftr.end_reason = (uint32_t)reason;
    ftr.total_bytes = (uint64_t)log->rec_bytes_written + (uint64_t)HKS_FRAME_SESSION_FOOTER_BYTES;
    hks_sha256_final(&log->pre_footer_sha_ctx, ftr.pre_footer_sha256);

    uint8_t ftr_payload[HKS_PAYLOAD_SESSION_FOOTER_BYTES];
    uint8_t ftr_frame[HKS_FRAME_SESSION_FOOTER_BYTES];
    (void)hks_encode_session_footer(ftr_payload, sizeof(ftr_payload), &ftr);
    (void)hks_frame_encode(ftr_frame, sizeof(ftr_frame),
                           HKS_MSG_SESSION_FOOTER, log->rec_last_flags,
                           log->rec_seq, end_ts_us,
                           ftr_payload, HKS_PAYLOAD_SESSION_FOOTER_BYTES);

    hks_sha256_update(&whole_ctx, ftr_frame, sizeof(ftr_frame));
    uint8_t whole_sha256[HKS_SHA256_DIGEST_BYTES];
    hks_sha256_final(&whole_ctx, whole_sha256);

    if (write_raw_frame_bytes(log, ftr_frame, sizeof(ftr_frame), true) != 0) {
        if (log->rec_reserved_block != 0xFFFFu) {
            log->block_owner[log->rec_reserved_block] = 0xFFu;
            log->rec_reserved_block = 0xFFFFu;
        }
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }
    if (flush_page_buffer(log, true) != 0) {
        if (log->rec_reserved_block != 0xFFFFu) {
            log->block_owner[log->rec_reserved_block] = 0xFFu;
            log->rec_reserved_block = 0xFFFFu;
        }
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }
    if (log->rec_reserved_block != 0xFFFFu) {
        log->block_owner[log->rec_reserved_block] = 0xFFu;
        log->rec_reserved_block = 0xFFFFu;
    }
    log->rec_session_slot = 0xFFu;

    const uint32_t dur_ms = (end_ts_us >= log->rec_start_ts_us)
                                ? (uint32_t)((end_ts_us - log->rec_start_ts_us) / 1000ULL)
                                : 0u;

    index_op_t close_op;
    memset(&close_op, 0, sizeof(close_op));
    close_op.op = HKS_INDEX_OP_CLOSE;
    close_op.flags = HKS_SESS_FLAG_CLOSED;
    close_op.session_id = log->rec_session_id;
    close_op.start_utc_ms = log->rec_start_utc_ms;
    close_op.duration_ms = dur_ms;
    close_op.byte_size = log->rec_bytes_written;
    close_op.sample_count = log->rec_imu_samples;
    close_op.session_flags = 0u;
    close_op.start_block = log->rec_start_block;
    close_op.block_count = log->rec_block_count;
    memcpy(close_op.whole_file_sha256, whole_sha256, HKS_SHA256_DIGEST_BYTES);

    log->state = HKS_FLASH_STATE_IDLE;
    if (index_append_op(log, &close_op) != 0) {
        log->state = HKS_FLASH_STATE_ERROR;
        return -1;
    }
    apply_index_op(log, &close_op, NULL);
    return 0;
}

hks_status_code_t hks_flash_log_list_sessions(const hks_flash_log_t *log,
                                              hks_session_list_entry_t *out,
                                              size_t max_entries,
                                              size_t *out_count) {
    if (log == NULL || out_count == NULL) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    if (log->state == HKS_FLASH_STATE_RECORDING) {
        return HKS_STATUS_ERR_BUSY_RECORDING;
    }
    size_t count = 0u;
    for (size_t i = 0u; i < HKS_FLASH_MAX_SESSIONS; i++) {
        const hks_flash_session_meta_t *m = &log->sessions[i];
        if (!m->valid || !m->closed) {
            continue;
        }
        if (out != NULL && count < max_entries) {
            out[count].session_id = m->session_id;
            out[count].start_utc_ms = m->start_utc_ms;
            out[count].duration_ms = m->duration_ms;
            out[count].byte_size = m->byte_size;
            out[count].sample_count = m->sample_count;
            out[count].flags = m->session_flags;
        }
        count++;
    }
    *out_count = count;
    return HKS_STATUS_OK;
}

hks_status_code_t hks_flash_log_read_bytes(const hks_flash_log_t *log,
                                           uint64_t session_id,
                                           uint32_t offset_bytes,
                                           uint16_t length_bytes,
                                           uint8_t *out,
                                           uint16_t *actual_len) {
    if (log == NULL || out == NULL || actual_len == NULL ||
        length_bytes == 0u || length_bytes > HKS_PROTO_MAX_CHUNK_BYTES) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    if (log->state == HKS_FLASH_STATE_RECORDING) {
        return HKS_STATUS_ERR_BUSY_RECORDING;
    }
    const hks_flash_session_meta_t *m = find_session_const(log, session_id);
    if (m == NULL || !m->closed) {
        return HKS_STATUS_ERR_SESSION_NOT_FOUND;
    }
    if (offset_bytes >= m->byte_size) {
        return HKS_STATUS_ERR_OFFSET_OUT_OF_RANGE;
    }

    const uint32_t avail = m->byte_size - offset_bytes;
    const uint16_t to_read = (avail < (uint32_t)length_bytes) ? (uint16_t)avail : length_bytes;

    uint32_t pos = offset_bytes;
    uint16_t done = 0u;
    while (done < to_read) {
        const uint16_t lblk = (uint16_t)(pos / HKS_NAND_BLOCK_BYTES);
        const uint32_t in_blk = pos % HKS_NAND_BLOCK_BYTES;
        const uint8_t pg = (uint8_t)(in_blk / HKS_NAND_PAGE_BYTES);
        const uint16_t col = (uint16_t)(in_blk % HKS_NAND_PAGE_BYTES);

        uint16_t phys_blk = 0u;
        if (!get_session_block_at_idx(log, m, lblk, &phys_blk)) {
            return HKS_STATUS_ERR_FLASH_IO;
        }
        const uint16_t pg_rem = (uint16_t)(HKS_NAND_PAGE_BYTES - col);
        const uint16_t chunk = ((uint16_t)(to_read - done) < pg_rem) ? (uint16_t)(to_read - done) : pg_rem;
        if (hks_hal_nand_read_page(phys_blk, pg, col, out + done, chunk) != 0) {
            return HKS_STATUS_ERR_FLASH_IO;
        }
        done = (uint16_t)(done + chunk);
        pos += chunk;
    }

    *actual_len = to_read;
    return HKS_STATUS_OK;
}

hks_status_code_t hks_flash_log_get_header(const hks_flash_log_t *log,
                                           uint64_t session_id,
                                           uint8_t out_416[HKS_GET_HEADER_RSP_BYTES]) {
    if (log == NULL || out_416 == NULL) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    if (log->state == HKS_FLASH_STATE_RECORDING) {
        return HKS_STATUS_ERR_BUSY_RECORDING;
    }
    const hks_flash_session_meta_t *m = find_session_const(log, session_id);
    if (m == NULL || !m->closed) {
        return HKS_STATUS_ERR_SESSION_NOT_FOUND;
    }
    if (m->byte_size < HKS_GET_HEADER_RSP_BYTES) {
        return HKS_STATUS_ERR_FLASH_IO;
    }
    uint16_t got = 0u;
    const hks_status_code_t st = hks_flash_log_read_bytes(log, session_id, 0u,
                                                          HKS_GET_HEADER_RSP_BYTES,
                                                          out_416, &got);
    if (st != HKS_STATUS_OK) {
        return st;
    }
    if (got != HKS_GET_HEADER_RSP_BYTES) {
        return HKS_STATUS_ERR_FLASH_IO;
    }
    return HKS_STATUS_OK;
}

hks_status_code_t hks_flash_log_verify_session(hks_flash_log_t *log,
                                               uint64_t session_id,
                                               uint32_t expected_file_crc32,
                                               const uint8_t *expected_sha256_opt,
                                               bool check_sha256) {
    if (log == NULL) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    if (log->state == HKS_FLASH_STATE_RECORDING) {
        return HKS_STATUS_ERR_BUSY_RECORDING;
    }
    hks_flash_session_meta_t *m = find_session_mut(log, session_id);
    if (m == NULL || !m->closed) {
        return HKS_STATUS_ERR_SESSION_NOT_FOUND;
    }

    uint32_t crc_state = hks_crc32_init();
    hks_sha256_ctx_t sha_ctx;
    hks_sha256_init(&sha_ctx);

    uint8_t buf[HKS_NAND_PAGE_BYTES];
    uint32_t off = 0u;
    while (off < m->byte_size) {
        const uint32_t rem = m->byte_size - off;
        const uint16_t want = (rem < HKS_NAND_PAGE_BYTES) ? (uint16_t)rem : (uint16_t)HKS_NAND_PAGE_BYTES;
        uint16_t got = 0u;
        const hks_status_code_t st = hks_flash_log_read_bytes(log, session_id, off, want, buf, &got);
        if (st != HKS_STATUS_OK || got != want) {
            return HKS_STATUS_ERR_FLASH_IO;
        }
        crc_state = hks_crc32_update(crc_state, buf, got);
        hks_sha256_update(&sha_ctx, buf, got);
        off += got;
    }

    const uint32_t actual_crc = hks_crc32_final(crc_state);
    uint8_t actual_sha[HKS_SHA256_DIGEST_BYTES];
    hks_sha256_final(&sha_ctx, actual_sha);

    if (actual_crc != expected_file_crc32) {
        return HKS_STATUS_ERR_VERIFY_MISMATCH;
    }
    if (check_sha256) {
        if (expected_sha256_opt == NULL ||
            memcmp(actual_sha, expected_sha256_opt, HKS_SHA256_DIGEST_BYTES) != 0) {
            return HKS_STATUS_ERR_VERIFY_MISMATCH;
        }
    }

    if (!m->verified) {
        m->verified = true;
        m->session_flags |= HKS_SESS_FLAG_VERIFIED;
        memcpy(m->whole_file_sha256, actual_sha, HKS_SHA256_DIGEST_BYTES);

        index_op_t ver_op;
        memset(&ver_op, 0, sizeof(ver_op));
        ver_op.op = HKS_INDEX_OP_VERIFY;
        ver_op.flags = (uint8_t)(HKS_SESS_FLAG_CLOSED | HKS_SESS_FLAG_VERIFIED |
                                 (m->recovered ? HKS_SESS_FLAG_RECOVERED : 0u));
        ver_op.session_id = m->session_id;
        ver_op.start_utc_ms = m->start_utc_ms;
        ver_op.duration_ms = m->duration_ms;
        ver_op.byte_size = m->byte_size;
        ver_op.sample_count = m->sample_count;
        ver_op.session_flags = m->session_flags;
        ver_op.start_block = m->start_block;
        ver_op.block_count = m->block_count;
        memcpy(ver_op.whole_file_sha256, m->whole_file_sha256, HKS_SHA256_DIGEST_BYTES);
        if (index_append_op(log, &ver_op) != 0) {
            return HKS_STATUS_ERR_FLASH_IO;
        }
    }
    return HKS_STATUS_OK;
}

static hks_status_code_t delete_session_internal(hks_flash_log_t *log,
                                                 uint64_t session_id,
                                                 bool force_flag,
                                                 uint32_t *freed_bytes_out) {
    if (log == NULL) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    const size_t slot_idx = find_session_slot(log, session_id);
    if (slot_idx >= HKS_FLASH_MAX_SESSIONS) {
        return HKS_STATUS_ERR_SESSION_NOT_FOUND;
    }
    hks_flash_session_meta_t *m = &log->sessions[slot_idx];
    if (!m->closed) {
        return HKS_STATUS_ERR_SESSION_NOT_FOUND;
    }
    if (!m->verified && !force_flag) {
        return HKS_STATUS_ERR_NOT_VERIFIED;
    }

    uint16_t blocks_to_erase[HKS_PART_RING_BLOCKS];
    uint16_t erase_cnt = 0u;
    uint16_t cur_b = (m->block_count > 0u) ? m->start_block : 0xFFFFu;
    for (uint16_t i = 0u; i < m->block_count && cur_b != 0xFFFFu; i++) {
        if (erase_cnt < HKS_PART_RING_BLOCKS) {
            blocks_to_erase[erase_cnt++] = cur_b;
        }
        cur_b = log->block_next[cur_b];
    }

    index_op_t del_op;
    memset(&del_op, 0, sizeof(del_op));
    del_op.op = HKS_INDEX_OP_DELETE;
    del_op.session_id = session_id;
    if (index_append_op(log, &del_op) != 0) {
        return HKS_STATUS_ERR_FLASH_IO;
    }

    session_clear_chain(log, slot_idx);
    m->valid = false;
    if (log->session_count > 0u) {
        log->session_count--;
    }

    uint32_t erased_blocks = 0u;
    for (uint16_t i = 0u; i < erase_cnt; i++) {
        const uint16_t b = blocks_to_erase[i];
        if (is_valid_ring_block(b) && !log->bad_blocks[b]) {
            if (hks_hal_nand_erase_block(b) != 0) {
                log->bad_blocks[b] = true;
                log->bad_block_count++;
                (void)hks_hal_nand_mark_bad_block(b);
            } else {
                erased_blocks++;
            }
        }
    }

    if (freed_bytes_out != NULL) {
        *freed_bytes_out = erased_blocks * HKS_NAND_BLOCK_BYTES;
    }
    return HKS_STATUS_OK;
}

hks_status_code_t hks_flash_log_delete_session(hks_flash_log_t *log,
                                               uint64_t session_id,
                                               bool force_flag,
                                               uint32_t *freed_bytes_out) {
    if (log == NULL) {
        return HKS_STATUS_ERR_INVALID_PARAM;
    }
    if (log->state == HKS_FLASH_STATE_RECORDING) {
        return HKS_STATUS_ERR_BUSY_RECORDING;
    }
    return delete_session_internal(log, session_id, force_flag, freed_bytes_out);
}

uint32_t hks_flash_log_free_kb(const hks_flash_log_t *log) {
    if (log == NULL) {
        return 0u;
    }
    uint32_t free_blocks = 0u;
    for (uint16_t b = HKS_PART_RING_START; b < HKS_PART_RING_START + HKS_PART_RING_BLOCKS; b++) {
        if (!log->bad_blocks[b] && !is_block_used_by_any_session(log, b)) {
            free_blocks++;
        }
    }
    return (free_blocks * HKS_NAND_BLOCK_BYTES) / 1024u;
}
