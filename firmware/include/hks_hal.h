#ifndef HKS_HAL_H
#define HKS_HAL_H

#include "hks_contract.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Link-Time Hardware Abstraction Layer (nRF5340-QKAA / Host SIL Mock)
 * ========================================================================= */

typedef enum {
    HKS_SPI_BUS_SPIM4_IMU   = 0, /* 16 MHz SPI Mode 0/3: ICM-42688-P */
    HKS_SPI_BUS_SPIM1_HIGHG = 1  /*  8 MHz SPI Mode 0 (<=10 MHz max): ADXL372 */
} hks_hal_spi_bus_t;

typedef enum {
    HKS_I2C_BUS_TWIM0_SENS = 0,  /* 400 kHz I2C0: MMC5983MA (0x30), BMP581 (0x47) */
    HKS_I2C_BUS_TWIM1_PMIC = 1,  /* 400 kHz TWIM2 (contract key 'TWIM1' alias, ID 11): nPM1300 (0x6B) */
    HKS_I2C_BUS_TWIM2_PMIC = HKS_I2C_BUS_TWIM1_PMIC
} hks_hal_i2c_bus_t;

typedef enum {
    HKS_LED_OFF   = 0u,
    HKS_LED_GREEN = (1u << 0), /* LD1 Green (P1.07 -> Q1A Gate 1) */
    HKS_LED_AMBER = (1u << 1), /* Dual-LED bi-color Amber (LD1 Green + LD2 Red ON simultaneously) */
    HKS_LED_RED   = (1u << 2)  /* LD2 Red (P1.08 -> Q1B Gate 2) */
} hks_hal_led_mask_t;

typedef struct {
    uint16_t vbatt_mv;
    int16_t batt_temp_c_x100;
    uint16_t soc_pct_x10;
    uint8_t pmic_status; /* bits [7:0] per hks_pmic_status_bits_t */
} hks_hal_pmic_info_t;

/* Monotonic 64-bit time & device identity */
uint64_t hks_hal_time_us(void);
void     hks_hal_delay_us(uint32_t us);
uint64_t hks_hal_device_id(void);

/* Raw bus transactions */
int hks_hal_spi_xfer(hks_hal_spi_bus_t bus, const uint8_t *tx, uint8_t *rx, size_t len);
int hks_hal_i2c_write_read(hks_hal_i2c_bus_t bus, uint8_t addr7,
                           const uint8_t *tx, size_t tx_len,
                           uint8_t *rx, size_t rx_len);

/* Sensor & PMIC typed read primitives (return 0 on success, <0 on error) */
int hks_hal_imu_read_fifo(hks_imu_sample_t *out, size_t max_samples,
                          size_t *out_count, int16_t *temp_c_x100);
/* Reads ADXL372 FIFO and applies portable (>> 4) right-alignment into [-2048, +2047] */
int hks_hal_highg_read_fifo(hks_high_g_sample_t *out, size_t max_samples,
                            size_t *out_count);
int hks_hal_mag_read(int32_t *mx_raw, int32_t *my_raw, int32_t *mz_raw,
                     bool *set_reset_done);
int hks_hal_baro_read(uint32_t *pressure_pa_x64, int16_t *temp_c_x100);
int hks_hal_pmic_read(hks_hal_pmic_info_t *info);

/* QSPI W25N01JW SLC NAND primitives (2048 B page, 64 pages/block, 1024 blocks) */
int hks_hal_nand_init(void);
int hks_hal_nand_read_page(uint16_t block, uint8_t page, uint16_t col,
                           uint8_t *buf, size_t len);
int hks_hal_nand_prog_page(uint16_t block, uint8_t page, uint16_t col,
                           const uint8_t *buf, size_t len);
int hks_hal_nand_erase_block(uint16_t block);
int hks_hal_nand_read_oob(uint16_t block, uint8_t page, uint8_t *oob, size_t len);
int hks_hal_nand_write_oob(uint16_t block, uint8_t page, const uint8_t *oob, size_t len);
int hks_hal_nand_is_bad_block(uint16_t block, bool *is_bad);
int hks_hal_nand_mark_bad_block(uint16_t block);

/* USB 2.0 FS CDC-ACM & Board GPIO */
size_t hks_hal_usb_cdc_rx_available(void);
size_t hks_hal_usb_cdc_read(uint8_t *buf, size_t max_len);
size_t hks_hal_usb_cdc_write(const uint8_t *buf, size_t len);
bool   hks_hal_usb_vbus_connected(void);
bool   hks_hal_button_pressed(void);
void   hks_hal_led_set(uint8_t color_mask);

/* =========================================================================
 * Host SIL / Unit-Test Mock Controls (implemented only in hal/hal_mock.c)
 * ========================================================================= */
void     hks_hal_mock_reset(void);
int      hks_hal_mock_nand_attach_file(const char *path, bool create_erased);
void     hks_hal_mock_nand_detach_file(void);
uint32_t hks_hal_mock_get_nop_violations(void);
uint32_t hks_hal_mock_get_bit_violations(void);
uint8_t  hks_hal_mock_get_page_nop(uint16_t block, uint8_t page);
void     hks_hal_mock_inject_bad_block(uint16_t block, bool runtime_io_fail);

void     hks_hal_mock_set_time_us(uint64_t time_us);
void     hks_hal_mock_advance_time_us(uint64_t delta_us);
void     hks_hal_mock_set_device_id(uint64_t device_id);

void     hks_hal_mock_set_pmic(const hks_hal_pmic_info_t *info);
void     hks_hal_mock_set_mag(int32_t mx_raw, int32_t my_raw, int32_t mz_raw, bool set_reset_done);
void     hks_hal_mock_set_baro(uint32_t pressure_pa_x64, int16_t temp_c_x100);
void     hks_hal_mock_push_imu(const hks_imu_sample_t *samples, size_t count, int16_t temp_c_x100);
/* Push raw 12-bit left-justified ADXL372 words (bits [15:4]) to verify >> 4 unpack path */
void     hks_hal_mock_push_highg_raw16(const int16_t *ax_lj12,
                                       const int16_t *ay_lj12,
                                       const int16_t *az_lj12,
                                       size_t count);

size_t   hks_hal_mock_usb_push_rx(const uint8_t *buf, size_t len);
size_t   hks_hal_mock_usb_pop_tx(uint8_t *buf, size_t max_len);
size_t   hks_hal_mock_usb_tx_available(void);
void     hks_hal_mock_set_vbus(bool connected);
void     hks_hal_mock_set_button(bool pressed);

#ifdef __cplusplus
}
#endif

#endif /* HKS_HAL_H */
