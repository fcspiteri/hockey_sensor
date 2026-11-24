#define _POSIX_C_SOURCE 200809L
#include "hks_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOCK_BLOCK_BYTES (HKS_NAND_PAGES_PER_BLOCK * (HKS_NAND_PAGE_BYTES + HKS_NAND_OOB_BYTES))
#define MOCK_USB_BUF_BYTES 8192u
#define MOCK_SENSOR_QUEUE_CAP 1024u

typedef struct {
    uint8_t *blocks[HKS_NAND_TOTAL_BLOCKS]; /* lazily allocated erased 0xFF blocks */
    uint8_t  nop_count[HKS_NAND_TOTAL_BLOCKS][HKS_NAND_PAGES_PER_BLOCK];
    bool     runtime_io_fail[HKS_NAND_TOTAL_BLOCKS];
    uint32_t nop_violations;
    uint32_t bit_violations;
    FILE    *backing_file;

    uint64_t time_us;
    uint64_t device_id;

    hks_hal_pmic_info_t pmic;
    int32_t             mx_raw;
    int32_t             my_raw;
    int32_t             mz_raw;
    bool                mag_set_reset;
    uint32_t            pressure_pa_x64;
    int16_t             baro_temp_c_x100;

    hks_imu_sample_t imu_q[MOCK_SENSOR_QUEUE_CAP];
    size_t           imu_q_head;
    size_t           imu_q_count;
    int16_t          imu_temp_c_x100;

    /* Raw 12-bit left-justified ADXL372 words to exercise >> 4 unpack path */
    uint16_t hg_raw_q[MOCK_SENSOR_QUEUE_CAP][3];
    size_t   hg_q_head;
    size_t   hg_q_count;

    uint8_t usb_rx_buf[MOCK_USB_BUF_BYTES];
    size_t  usb_rx_head;
    size_t  usb_rx_tail;
    size_t  usb_rx_count;

    uint8_t usb_tx_buf[MOCK_USB_BUF_BYTES];
    size_t  usb_tx_head;
    size_t  usb_tx_tail;
    size_t  usb_tx_count;

    bool    vbus_connected;
    bool    button_pressed;
    uint8_t led_mask;
    bool    initialized;
} mock_state_t;

static mock_state_t g_mock;

static uint8_t *ensure_block(uint16_t block) {
    if (block >= HKS_NAND_TOTAL_BLOCKS) {
        return NULL;
    }
    if (g_mock.blocks[block] == NULL) {
        g_mock.blocks[block] = (uint8_t *)malloc(MOCK_BLOCK_BYTES);
        if (g_mock.blocks[block] != NULL) {
            memset(g_mock.blocks[block], 0xFF, MOCK_BLOCK_BYTES);
        }
    }
    return g_mock.blocks[block];
}

static size_t page_data_offset(uint8_t page) {
    return (size_t)page * (HKS_NAND_PAGE_BYTES + HKS_NAND_OOB_BYTES);
}

static size_t page_oob_offset(uint8_t page) {
    return page_data_offset(page) + HKS_NAND_PAGE_BYTES;
}

static void sync_block_to_file(uint16_t block) {
    if (g_mock.backing_file == NULL || g_mock.blocks[block] == NULL) {
        return;
    }
    const long off = (long)block * (long)MOCK_BLOCK_BYTES;
    if (fseek(g_mock.backing_file, off, SEEK_SET) == 0) {
        (void)fwrite(g_mock.blocks[block], 1u, MOCK_BLOCK_BYTES, g_mock.backing_file);
        (void)fflush(g_mock.backing_file);
    }
}

void hks_hal_mock_reset(void) {
    if (g_mock.backing_file != NULL) {
        fclose(g_mock.backing_file);
        g_mock.backing_file = NULL;
    }
    for (uint16_t b = 0u; b < HKS_NAND_TOTAL_BLOCKS; b++) {
        free(g_mock.blocks[b]);
        g_mock.blocks[b] = NULL;
    }
    memset(&g_mock, 0, sizeof(g_mock));
    g_mock.device_id = 102030405060708ULL;
    g_mock.pmic.vbatt_mv = 3920u;
    g_mock.pmic.batt_temp_c_x100 = 1850;
    g_mock.pmic.soc_pct_x10 = 845u;
    g_mock.pmic.pmic_status = HKS_PMIC_FG_VALID;
    g_mock.mx_raw = 3200;
    g_mock.my_raw = -1400;
    g_mock.mz_raw = 7300;
    g_mock.mag_set_reset = true;
    g_mock.pressure_pa_x64 = 6484800u;
    g_mock.baro_temp_c_x100 = 2140;
    g_mock.imu_temp_c_x100 = 2150;
    g_mock.initialized = true;
}

int hks_hal_mock_nand_attach_file(const char *path, bool create_erased) {
    if (path == NULL) {
        return -1;
    }
    if (g_mock.backing_file != NULL) {
        fclose(g_mock.backing_file);
        g_mock.backing_file = NULL;
    }
    FILE *fp = fopen(path, create_erased ? "w+b" : "r+b");
    if (fp == NULL) {
        return -1;
    }
    if (create_erased) {
        uint8_t erased_blk[MOCK_BLOCK_BYTES];
        memset(erased_blk, 0xFF, sizeof(erased_blk));
        for (uint16_t b = 0u; b < HKS_NAND_TOTAL_BLOCKS; b++) {
            if (fwrite(erased_blk, 1u, sizeof(erased_blk), fp) != sizeof(erased_blk)) {
                fclose(fp);
                return -1;
            }
        }
        (void)fflush(fp);
    } else {
        for (uint16_t b = 0u; b < HKS_NAND_TOTAL_BLOCKS; b++) {
            uint8_t *blk = ensure_block(b);
            if (blk == NULL) {
                fclose(fp);
                return -1;
            }
            if (fread(blk, 1u, MOCK_BLOCK_BYTES, fp) != MOCK_BLOCK_BYTES) {
                memset(blk, 0xFF, MOCK_BLOCK_BYTES);
            }
        }
    }
    g_mock.backing_file = fp;
    return 0;
}

void hks_hal_mock_nand_detach_file(void) {
    if (g_mock.backing_file != NULL) {
        fclose(g_mock.backing_file);
        g_mock.backing_file = NULL;
    }
}

uint32_t hks_hal_mock_get_nop_violations(void) {
    return g_mock.nop_violations;
}

uint32_t hks_hal_mock_get_bit_violations(void) {
    return g_mock.bit_violations;
}

uint8_t hks_hal_mock_get_page_nop(uint16_t block, uint8_t page) {
    if (block >= HKS_NAND_TOTAL_BLOCKS || page >= HKS_NAND_PAGES_PER_BLOCK) {
        return 0u;
    }
    return g_mock.nop_count[block][page];
}

void hks_hal_mock_inject_bad_block(uint16_t block, bool runtime_io_fail) {
    if (block >= HKS_NAND_TOTAL_BLOCKS) {
        return;
    }
    if (runtime_io_fail) {
        g_mock.runtime_io_fail[block] = true;
    } else {
        uint8_t *blk = ensure_block(block);
        if (blk != NULL) {
            blk[page_oob_offset(0u) + 0u] = 0x00u;
            sync_block_to_file(block);
        }
    }
}

void hks_hal_mock_set_time_us(uint64_t time_us) {
    g_mock.time_us = time_us;
}

void hks_hal_mock_advance_time_us(uint64_t delta_us) {
    g_mock.time_us += delta_us;
}

void hks_hal_mock_set_device_id(uint64_t device_id) {
    g_mock.device_id = device_id;
}

void hks_hal_mock_set_pmic(const hks_hal_pmic_info_t *info) {
    if (info != NULL) {
        g_mock.pmic = *info;
    }
}

void hks_hal_mock_set_mag(int32_t mx_raw, int32_t my_raw, int32_t mz_raw, bool set_reset_done) {
    g_mock.mx_raw = mx_raw;
    g_mock.my_raw = my_raw;
    g_mock.mz_raw = mz_raw;
    g_mock.mag_set_reset = set_reset_done;
}

void hks_hal_mock_set_baro(uint32_t pressure_pa_x64, int16_t temp_c_x100) {
    g_mock.pressure_pa_x64 = pressure_pa_x64;
    g_mock.baro_temp_c_x100 = temp_c_x100;
}

void hks_hal_mock_push_imu(const hks_imu_sample_t *samples, size_t count, int16_t temp_c_x100) {
    if (samples == NULL) {
        return;
    }
    g_mock.imu_temp_c_x100 = temp_c_x100;
    for (size_t i = 0u; i < count && g_mock.imu_q_count < MOCK_SENSOR_QUEUE_CAP; i++) {
        const size_t idx = (g_mock.imu_q_head + g_mock.imu_q_count) % MOCK_SENSOR_QUEUE_CAP;
        g_mock.imu_q[idx] = samples[i];
        g_mock.imu_q_count++;
    }
}

void hks_hal_mock_push_highg_raw16(const int16_t *ax_lj12,
                                   const int16_t *ay_lj12,
                                   const int16_t *az_lj12,
                                   size_t count) {
    if (ax_lj12 == NULL || ay_lj12 == NULL || az_lj12 == NULL) {
        return;
    }
    for (size_t i = 0u; i < count && g_mock.hg_q_count < MOCK_SENSOR_QUEUE_CAP; i++) {
        const size_t idx = (g_mock.hg_q_head + g_mock.hg_q_count) % MOCK_SENSOR_QUEUE_CAP;
        uint16_t ux = 0u, uy = 0u, uz = 0u;
        memcpy(&ux, &ax_lj12[i], sizeof(ux));
        memcpy(&uy, &ay_lj12[i], sizeof(uy));
        memcpy(&uz, &az_lj12[i], sizeof(uz));
        g_mock.hg_raw_q[idx][0] = ux;
        g_mock.hg_raw_q[idx][1] = uy;
        g_mock.hg_raw_q[idx][2] = uz;
        g_mock.hg_q_count++;
    }
}

size_t hks_hal_mock_usb_push_rx(const uint8_t *buf, size_t len) {
    if (buf == NULL) {
        return 0u;
    }
    size_t pushed = 0u;
    while (pushed < len && g_mock.usb_rx_count < MOCK_USB_BUF_BYTES) {
        g_mock.usb_rx_buf[g_mock.usb_rx_tail] = buf[pushed++];
        g_mock.usb_rx_tail = (g_mock.usb_rx_tail + 1u) % MOCK_USB_BUF_BYTES;
        g_mock.usb_rx_count++;
    }
    return pushed;
}

size_t hks_hal_mock_usb_pop_tx(uint8_t *buf, size_t max_len) {
    if (buf == NULL) {
        return 0u;
    }
    size_t popped = 0u;
    while (popped < max_len && g_mock.usb_tx_count > 0u) {
        buf[popped++] = g_mock.usb_tx_buf[g_mock.usb_tx_head];
        g_mock.usb_tx_head = (g_mock.usb_tx_head + 1u) % MOCK_USB_BUF_BYTES;
        g_mock.usb_tx_count--;
    }
    return popped;
}

size_t hks_hal_mock_usb_tx_available(void) {
    return g_mock.usb_tx_count;
}

void hks_hal_mock_set_vbus(bool connected) {
    g_mock.vbus_connected = connected;
}

void hks_hal_mock_set_button(bool pressed) {
    g_mock.button_pressed = pressed;
}

/* =========================================================================
 * Link-Time HAL Implementation
 * ========================================================================= */

uint64_t hks_hal_time_us(void) {
    return g_mock.time_us;
}

void hks_hal_delay_us(uint32_t us) {
    g_mock.time_us += (uint64_t)us;
}

uint64_t hks_hal_device_id(void) {
    if (!g_mock.initialized) {
        return 102030405060708ULL;
    }
    return g_mock.device_id;
}

int hks_hal_spi_xfer(hks_hal_spi_bus_t bus, const uint8_t *tx, uint8_t *rx, size_t len) {
    (void)bus;
    if (rx != NULL && len > 0u) {
        if (tx != NULL) {
            memcpy(rx, tx, len);
        } else {
            memset(rx, 0, len);
        }
    }
    return 0;
}

int hks_hal_i2c_write_read(hks_hal_i2c_bus_t bus, uint8_t addr7,
                           const uint8_t *tx, size_t tx_len,
                           uint8_t *rx, size_t rx_len) {
    (void)bus;
    (void)addr7;
    (void)tx;
    (void)tx_len;
    if (rx != NULL && rx_len > 0u) {
        memset(rx, 0, rx_len);
    }
    return 0;
}

int hks_hal_imu_read_fifo(hks_imu_sample_t *out, size_t max_samples,
                          size_t *out_count, int16_t *temp_c_x100) {
    if (out == NULL || out_count == NULL) {
        return -1;
    }
    size_t n = 0u;
    while (n < max_samples && g_mock.imu_q_count > 0u) {
        out[n++] = g_mock.imu_q[g_mock.imu_q_head];
        g_mock.imu_q_head = (g_mock.imu_q_head + 1u) % MOCK_SENSOR_QUEUE_CAP;
        g_mock.imu_q_count--;
    }
    *out_count = n;
    if (temp_c_x100 != NULL) {
        *temp_c_x100 = g_mock.imu_temp_c_x100;
    }
    return 0;
}

int hks_hal_highg_read_fifo(hks_high_g_sample_t *out, size_t max_samples,
                            size_t *out_count) {
    if (out == NULL || out_count == NULL) {
        return -1;
    }
    size_t n = 0u;
    while (n < max_samples && g_mock.hg_q_count > 0u) {
        out[n].ax = hks_adxl372_unpack_i12(g_mock.hg_raw_q[g_mock.hg_q_head][0]);
        out[n].ay = hks_adxl372_unpack_i12(g_mock.hg_raw_q[g_mock.hg_q_head][1]);
        out[n].az = hks_adxl372_unpack_i12(g_mock.hg_raw_q[g_mock.hg_q_head][2]);
        g_mock.hg_q_head = (g_mock.hg_q_head + 1u) % MOCK_SENSOR_QUEUE_CAP;
        g_mock.hg_q_count--;
        n++;
    }
    *out_count = n;
    return 0;
}

int hks_hal_mag_read(int32_t *mx_raw, int32_t *my_raw, int32_t *mz_raw,
                     bool *set_reset_done) {
    if (mx_raw != NULL) *mx_raw = g_mock.mx_raw;
    if (my_raw != NULL) *my_raw = g_mock.my_raw;
    if (mz_raw != NULL) *mz_raw = g_mock.mz_raw;
    if (set_reset_done != NULL) *set_reset_done = g_mock.mag_set_reset;
    return 0;
}

int hks_hal_baro_read(uint32_t *pressure_pa_x64, int16_t *temp_c_x100) {
    if (pressure_pa_x64 != NULL) *pressure_pa_x64 = g_mock.pressure_pa_x64;
    if (temp_c_x100 != NULL) *temp_c_x100 = g_mock.baro_temp_c_x100;
    return 0;
}

int hks_hal_pmic_read(hks_hal_pmic_info_t *info) {
    if (info == NULL) {
        return -1;
    }
    *info = g_mock.pmic;
    return 0;
}

int hks_hal_nand_init(void) {
    if (!g_mock.initialized) {
        hks_hal_mock_reset();
    }
    return 0;
}

int hks_hal_nand_read_page(uint16_t block, uint8_t page, uint16_t col,
                           uint8_t *buf, size_t len) {
    if (buf == NULL || block >= HKS_NAND_TOTAL_BLOCKS ||
        page >= HKS_NAND_PAGES_PER_BLOCK ||
        (size_t)col + len > HKS_NAND_PAGE_BYTES) {
        return -1;
    }
    if (g_mock.runtime_io_fail[block]) {
        return -1;
    }
    if (g_mock.blocks[block] == NULL) {
        memset(buf, 0xFF, len);
        return 0;
    }
    memcpy(buf, g_mock.blocks[block] + page_data_offset(page) + col, len);
    return 0;
}

int hks_hal_nand_prog_page(uint16_t block, uint8_t page, uint16_t col,
                           const uint8_t *buf, size_t len) {
    if (buf == NULL || block >= HKS_NAND_TOTAL_BLOCKS ||
        page >= HKS_NAND_PAGES_PER_BLOCK ||
        (size_t)col + len > HKS_NAND_PAGE_BYTES) {
        return -1;
    }
    if (g_mock.runtime_io_fail[block]) {
        return -1;
    }
    uint8_t *blk = ensure_block(block);
    if (blk == NULL) {
        return -1;
    }
    if (g_mock.nop_count[block][page] >= HKS_NAND_MAX_NOP) {
        g_mock.nop_violations++;
        return -1;
    }
    g_mock.nop_count[block][page]++;

    uint8_t *dst = blk + page_data_offset(page) + col;
    for (size_t i = 0u; i < len; i++) {
        /* Real NAND can only clear 1 -> 0 bits; 0 -> 1 without erase is a violation */
        if ((dst[i] & buf[i]) != buf[i]) {
            g_mock.bit_violations++;
        }
        dst[i] &= buf[i];
    }
    sync_block_to_file(block);
    return 0;
}

int hks_hal_nand_erase_block(uint16_t block) {
    if (block >= HKS_NAND_TOTAL_BLOCKS) {
        return -1;
    }
    if (g_mock.runtime_io_fail[block]) {
        return -1;
    }
    if (g_mock.blocks[block] != NULL) {
        memset(g_mock.blocks[block], 0xFF, MOCK_BLOCK_BYTES);
        sync_block_to_file(block);
    }
    memset(g_mock.nop_count[block], 0, sizeof(g_mock.nop_count[block]));
    return 0;
}

int hks_hal_nand_read_oob(uint16_t block, uint8_t page, uint8_t *oob, size_t len) {
    if (oob == NULL || block >= HKS_NAND_TOTAL_BLOCKS ||
        page >= HKS_NAND_PAGES_PER_BLOCK || len > HKS_NAND_OOB_BYTES) {
        return -1;
    }
    if (g_mock.blocks[block] == NULL) {
        memset(oob, 0xFF, len);
        return 0;
    }
    memcpy(oob, g_mock.blocks[block] + page_oob_offset(page), len);
    return 0;
}

int hks_hal_nand_write_oob(uint16_t block, uint8_t page, const uint8_t *oob, size_t len) {
    if (oob == NULL || block >= HKS_NAND_TOTAL_BLOCKS ||
        page >= HKS_NAND_PAGES_PER_BLOCK || len > HKS_NAND_OOB_BYTES) {
        return -1;
    }
    uint8_t *blk = ensure_block(block);
    if (blk == NULL) {
        return -1;
    }
    uint8_t *dst = blk + page_oob_offset(page);
    for (size_t i = 0u; i < len; i++) {
        dst[i] &= oob[i];
    }
    sync_block_to_file(block);
    return 0;
}

int hks_hal_nand_is_bad_block(uint16_t block, bool *is_bad) {
    if (is_bad == NULL || block >= HKS_NAND_TOTAL_BLOCKS) {
        return -1;
    }
    if (g_mock.blocks[block] == NULL) {
        *is_bad = false;
        return 0;
    }
    const uint8_t oob0 = g_mock.blocks[block][page_oob_offset(0u)];
    const uint8_t oob1 = g_mock.blocks[block][page_oob_offset(1u)];
    *is_bad = (oob0 != 0xFFu || oob1 != 0xFFu);
    return 0;
}

int hks_hal_nand_mark_bad_block(uint16_t block) {
    if (block >= HKS_NAND_TOTAL_BLOCKS) {
        return -1;
    }
    uint8_t zero = 0x00u;
    return hks_hal_nand_write_oob(block, 0u, &zero, 1u);
}

size_t hks_hal_usb_cdc_rx_available(void) {
    return g_mock.usb_rx_count;
}

size_t hks_hal_usb_cdc_read(uint8_t *buf, size_t max_len) {
    if (buf == NULL) {
        return 0u;
    }
    size_t n = 0u;
    while (n < max_len && g_mock.usb_rx_count > 0u) {
        buf[n++] = g_mock.usb_rx_buf[g_mock.usb_rx_head];
        g_mock.usb_rx_head = (g_mock.usb_rx_head + 1u) % MOCK_USB_BUF_BYTES;
        g_mock.usb_rx_count--;
    }
    return n;
}

size_t hks_hal_usb_cdc_write(const uint8_t *buf, size_t len) {
    if (buf == NULL) {
        return 0u;
    }
    size_t n = 0u;
    while (n < len && g_mock.usb_tx_count < MOCK_USB_BUF_BYTES) {
        g_mock.usb_tx_buf[g_mock.usb_tx_tail] = buf[n++];
        g_mock.usb_tx_tail = (g_mock.usb_tx_tail + 1u) % MOCK_USB_BUF_BYTES;
        g_mock.usb_tx_count++;
    }
    return n;
}

bool hks_hal_usb_vbus_connected(void) {
    return g_mock.vbus_connected;
}

bool hks_hal_button_pressed(void) {
    return g_mock.button_pressed;
}

void hks_hal_led_set(uint8_t color_mask) {
    g_mock.led_mask = color_mask;
}
