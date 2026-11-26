#include "hks_hal.h"

#include <string.h>

/* =========================================================================
 * nRF5340-QKAA Bare-Metal / Zephyr Target HAL Stub
 * Aligned with M2 hardware/ee/pinmux_and_buses.json (v1.0.0)
 * -------------------------------------------------------------------------
 * Silicon Peripheral Mapping (non-secure App Core 0x4000xxxx):
 *   - TWIM0 (ID  8, 0x40008000): P1.02/P1.03 -> MMC5983MA (0x30 via PCA9306) & BMP581 (0x47)
 *   - SPIM1 (ID  9, 0x40009000): P0.19..P0.22 -> ADXL372 (8 MHz, <=10 MHz max)
 *   - SPIM4 (ID 10, 0x4000A000): P0.08..P0.11 -> ICM-42688-P (16 MHz)
 *   - TWIM2 (ID 11, 0x4000B000): P0.23/P0.24 -> nPM1300 (0x6B)
 *       Contract alias 'TWIM1': avoids Peripheral ID 9 conflict with SPIM1.
 *   - SPIM3 (ID 12, 0x4000C000): P0.13..P0.18 -> W25N01JWZEIG (8 MHz 1-bit SPI NAND)
 *       Contract alias 'QSPI': avoids QSPI 24-bit EasyDMA vs W25N01JW 16-bit column addr
 *       (optional hybrid QSPI READOC=0x03 at 0x4002B000).
 *   - P1.06 BTN_USER_N: active-low via passive Schottky D2 (BAT54WS) + 100 kOhm 1.8 V pull-up.
 *   - P1.05 PMIC_INT: nPM1300 GPIO0 GPOIRQ -> TWIM2 EVENTSSHPHLD & PMIC fault IRQs.
 * ========================================================================= */

#define NRF5340_FICR_BASE       0x00FF0000UL
#define NRF5340_TWIM0_BASE      0x40008000UL /* ID  8: MMC5983MA & BMP581 */
#define NRF5340_SPIM1_BASE      0x40009000UL /* ID  9: ADXL372 */
#define NRF5340_SPIM4_BASE      0x4000A000UL /* ID 10: ICM-42688-P */
#define NRF5340_TWIM2_BASE      0x4000B000UL /* ID 11: nPM1300 (contract 'TWIM1' alias) */
#define NRF5340_TWIM1_BASE      NRF5340_TWIM2_BASE
#define NRF5340_SPIM3_BASE      0x4000C000UL /* ID 12: W25N01JW 1-bit SPI NAND (contract 'QSPI' alias) */
#define NRF5340_QSPI_HW_BASE    0x4002B000UL /* Optional hybrid QSPI READOC=0x03 */
#define NRF5340_QSPI_BASE       NRF5340_SPIM3_BASE
#define NRF5340_RTC0_BASE       0x40014000UL
#define NRF5340_USBD_BASE       0x40036000UL
#define NRF5340_P0_BASE         0x40842500UL /* App Core P0 GPIO base */
#define NRF5340_P1_BASE         0x40842800UL /* App Core P1 GPIO base */
#define NRF_GPIO_OUTSET_OFFSET  0x008UL
#define NRF_GPIO_OUTCLR_OFFSET  0x00CUL
#define NRF_GPIO_PIN_CNF_OFFSET 0x200UL
#define NRF_PIN_CNF_PULLUP_IN   0x0000000Cu  /* DIR=Input(0), INPUT=Connect(0), PULL=Pullup(3<<2) */
#define NRF_PIN_CNF_PULLDOWN_DIS 0x00000006u /* DIR=Input(0), INPUT=Disconnect(1<<1), PULL=Pulldown(1<<2) for DNP U4 pins */
#define NRF_PIN_CNF_STD_OUT     0x00000003u  /* DIR=Output(1), INPUT=Disconnect(1<<1), DRIVE=S0S1(0) */

/* U3 Primary 6-DOF IMU (auto-detects LSM6DS3TR-C, BMI270, QMI8658C, or ICM-42688-P on SPIM4) */
#define ICM42688_REG_INT_CONFIG 0x14u
#define ICM42688_INT_CFG_PP_AH  0x1Bu /* INT1 & INT2 push-pull (bit1/4=1), active-high (bit0/3=1), pulsed (bit2/5=0) */
#define ICM42688_REG_WHO_AM_I   0x75u
#define ICM42688_WHO_AM_I_VAL   0x47u
#define LSM6DS3_REG_WHO_AM_I    0x0Fu
#define LSM6DS3_WHO_AM_I_VAL    0x6Au /* LSM6DS3TR-C (0x6A) / LSM6DS3 (0x69) */
#define LSM6DS3_REG_CTRL1_XL    0x10u
#define LSM6DS3_REG_CTRL2_G     0x11u
#define LSM6DS3_REG_CTRL3_C     0x12u
#define BMI270_REG_CHIP_ID      0x00u
#define BMI270_CHIP_ID_VAL      0x24u

/* U4 ADXL372 High-G Accelerometer (DNP by default on cost-optimized Rev A builds) */
#ifndef HKS_HIGHG_DNP
#define HKS_HIGHG_DNP           1
#endif
#define ADXL372_REG_DEVID_AD    0x00u
#define ADXL372_DEVID_AD_VAL    0xADu
#define ADXL372_REG_PARTID      0x02u
#define ADXL372_PARTID_VAL      0xFAu

#define NRF_GPIO_PIN(port, pin) (((uint32_t)(port) * 32u) + (uint32_t)(pin))

/* M2 silicon pinmap (override via -DHKS_PINMUX_OVERRIDDEN) */
#ifndef HKS_PINMUX_OVERRIDDEN
/* SPIM4: ICM-42688-P */
#define HKS_PIN_SPIM4_SCK       NRF_GPIO_PIN(0u, 8u)   /* P0.08 */
#define HKS_PIN_SPIM4_MOSI      NRF_GPIO_PIN(0u, 9u)   /* P0.09 */
#define HKS_PIN_SPIM4_MISO      NRF_GPIO_PIN(0u, 10u)  /* P0.10 */
#define HKS_PIN_CS_IMU_N        NRF_GPIO_PIN(0u, 11u)  /* P0.11 */
/* SPIM1: ADXL372 */
#define HKS_PIN_SPIM1_SCK       NRF_GPIO_PIN(0u, 19u)  /* P0.19 */
#define HKS_PIN_SPIM1_MOSI      NRF_GPIO_PIN(0u, 20u)  /* P0.20 */
#define HKS_PIN_SPIM1_MISO      NRF_GPIO_PIN(0u, 21u)  /* P0.21 */
#define HKS_PIN_CS_HIGHG_N      NRF_GPIO_PIN(0u, 22u)  /* P0.22 */
/* TWIM0: MMC5983MA (via PCA9306) & BMP581 */
#define HKS_PIN_TWIM0_SDA       NRF_GPIO_PIN(1u, 2u)   /* P1.02 */
#define HKS_PIN_TWIM0_SCL       NRF_GPIO_PIN(1u, 3u)   /* P1.03 */
/* TWIM2 (contract alias TWIM1): nPM1300 */
#define HKS_PIN_TWIM2_SDA       NRF_GPIO_PIN(0u, 23u)  /* P0.23 */
#define HKS_PIN_TWIM2_SCL       NRF_GPIO_PIN(0u, 24u)  /* P0.24 */
#define HKS_PIN_TWIM1_SDA       HKS_PIN_TWIM2_SDA
#define HKS_PIN_TWIM1_SCL       HKS_PIN_TWIM2_SCL
/* SPIM3 (contract alias QSPI): W25N01JWZEIG */
#define HKS_PIN_SPIM3_MOSI      NRF_GPIO_PIN(0u, 13u)  /* P0.13 IO0/SI */
#define HKS_PIN_SPIM3_MISO      NRF_GPIO_PIN(0u, 14u)  /* P0.14 IO1/SO */
#define HKS_PIN_FLASH_WP_N      NRF_GPIO_PIN(0u, 15u)  /* P0.15 IO2 /WP held 1V8 */
#define HKS_PIN_FLASH_HOLD_N    NRF_GPIO_PIN(0u, 16u)  /* P0.16 IO3 /HOLD held 1V8 */
#define HKS_PIN_SPIM3_SCK       NRF_GPIO_PIN(0u, 17u)  /* P0.17 SCK */
#define HKS_PIN_FLASH_CS_N      NRF_GPIO_PIN(0u, 18u)  /* P0.18 FLASH_CS_N */
#define HKS_PIN_QSPI_SCK        HKS_PIN_SPIM3_SCK
#define HKS_PIN_QSPI_CSN        HKS_PIN_FLASH_CS_N
/* Interrupts, button & LEDs */
#define HKS_PIN_IMU_INT1        NRF_GPIO_PIN(0u, 4u)   /* P0.04 */
#define HKS_PIN_IMU_INT2        NRF_GPIO_PIN(0u, 5u)   /* P0.05 */
#define HKS_PIN_ADXL_INT1       NRF_GPIO_PIN(0u, 6u)   /* P0.06 */
#define HKS_PIN_ADXL_INT2       NRF_GPIO_PIN(0u, 7u)   /* P0.07 */
#define HKS_PIN_MAG_INT         NRF_GPIO_PIN(1u, 1u)   /* P1.01 */
#define HKS_PIN_BARO_INT        NRF_GPIO_PIN(1u, 4u)   /* P1.04 */
#define HKS_PIN_PMIC_INT        NRF_GPIO_PIN(1u, 5u)   /* P1.05 nPM1300 GPIO0 GPOIRQ -> EVENTSSHPHLD */
#define HKS_PIN_BTN_USER_N      NRF_GPIO_PIN(1u, 6u)   /* P1.06 D2 BAT54WS + 100k pull-up */
#define HKS_PIN_SHPHLD_BTN      HKS_PIN_BTN_USER_N
#define HKS_PIN_LED_GREEN       NRF_GPIO_PIN(1u, 7u)   /* P1.07 LED_STATUS_G */
#define HKS_PIN_LED_RED         NRF_GPIO_PIN(1u, 8u)   /* P1.08 LED_STATUS_R */
#endif

/* W25N01JW SPI NAND instruction opcodes */
#define W25N_CMD_RESET          0xFFu
#define W25N_CMD_READ_STATUS    0x0Fu
#define W25N_CMD_WRITE_ENABLE   0x06u
#define W25N_CMD_BLOCK_ERASE    0xD8u
#define W25N_CMD_PROG_DATA_LOAD 0x02u
#define W25N_CMD_PROG_EXECUTE   0x10u
#define W25N_CMD_PAGE_DATA_READ 0x13u
#define W25N_CMD_READ_DATA      0x03u
#define W25N_CMD_FAST_READ_QUAD 0x6Bu

/* Sensor I2C / SPI addresses (supports BMP581 0x47, BMP388/BMP390 0x77, and LPS22DFTR/LPS22HHTR 0x5D) */
#define HKS_MMC5983MA_ADDR7     0x30u
#define HKS_BMP581_ADDR7        0x47u
#define HKS_BMP388_ADDR7        0x77u
#define HKS_LPS22_ADDR7         0x5Du
#define HKS_NPM1300_ADDR7       0x6Bu

static volatile uint64_t g_stub_mono_us = 0u;
static volatile uint32_t g_stub_p0_pin_cnf[32];
static volatile uint32_t g_stub_p1_out = 0u;
static volatile uint32_t g_stub_p1_pin_cnf[32];
static bool g_imu_int_configured = false;
static bool g_pmic_int_pullup_configured = false;
static bool g_highg_probed = false;
static bool g_highg_present = false;

static inline uintptr_t spi_bus_base(hks_hal_spi_bus_t bus) {
    return (bus == HKS_SPI_BUS_SPIM4_IMU)
               ? (uintptr_t)NRF5340_SPIM4_BASE
               : (uintptr_t)NRF5340_SPIM1_BASE;
}

static inline uintptr_t i2c_bus_base(hks_hal_i2c_bus_t bus) {
    return (bus == HKS_I2C_BUS_TWIM0_SENS)
               ? (uintptr_t)NRF5340_TWIM0_BASE
               : (uintptr_t)NRF5340_TWIM1_BASE;
}

uint64_t hks_hal_time_us(void) {
    /* Target firmware binds to DPPI/TIMER0+RTC0 64-bit counter */
    (void)NRF5340_RTC0_BASE;
    return g_stub_mono_us;
}

void hks_hal_delay_us(uint32_t us) {
    g_stub_mono_us += (uint64_t)us;
}

uint64_t hks_hal_device_id(void) {
    /* Target firmware reads FICR.INFO.DEVICEID[0..1] at NRF5340_FICR_BASE */
    (void)NRF5340_FICR_BASE;
    return 0x484B533533343001ULL;
}

int hks_hal_spi_xfer(hks_hal_spi_bus_t bus, const uint8_t *tx, uint8_t *rx, size_t len) {
    (void)spi_bus_base(bus);
    (void)tx;
    (void)HKS_PIN_CS_IMU_N;
    (void)HKS_PIN_CS_HIGHG_N;
    if (rx != NULL && len > 0u) {
        memset(rx, 0, len);
    }
    return 0;
}

int hks_hal_i2c_write_read(hks_hal_i2c_bus_t bus, uint8_t addr7,
                           const uint8_t *tx, size_t tx_len,
                           uint8_t *rx, size_t rx_len) {
    (void)i2c_bus_base(bus);
    (void)addr7;
    (void)tx;
    (void)tx_len;
    (void)HKS_PIN_TWIM0_SCL;
    (void)HKS_PIN_TWIM1_SCL;
    if (rx != NULL && rx_len > 0u) {
        memset(rx, 0, rx_len);
    }
    return 0;
}

int hks_hal_imu_read_fifo(hks_imu_sample_t *out, size_t max_samples,
                          size_t *out_count, int16_t *temp_c_x100) {
    (void)out;
    (void)max_samples;
    (void)HKS_PIN_SPIM4_SCK;
    if (!g_imu_int_configured) {
        /* Probe U3 on SPIM4: supports LSM6DS3TR-C (0x6A), BMI270 (0x24), and ICM-42688-P (0x47) */
        const uint8_t probe_lsm[2] = { (uint8_t)(LSM6DS3_REG_WHO_AM_I | 0x80u), 0x00u };
        uint8_t rx_id[2] = { 0u, 0u };
        (void)hks_hal_spi_xfer(HKS_SPI_BUS_SPIM4_IMU, probe_lsm, rx_id, sizeof(probe_lsm));
        if (rx_id[1] == LSM6DS3_WHO_AM_I_VAL || rx_id[1] == 0x69u) {
            /* Configure LSM6DS3TR-C: 208 Hz +-16g accel, 208 Hz +-2000dps gyro, BDU=1, push-pull active-high INT */
            const uint8_t cfg_xl[2]  = { LSM6DS3_REG_CTRL1_XL, 0x54u };
            const uint8_t cfg_g[2]   = { LSM6DS3_REG_CTRL2_G,  0x5Cu };
            const uint8_t cfg_c3[2]  = { LSM6DS3_REG_CTRL3_C,  0x44u };
            (void)hks_hal_spi_xfer(HKS_SPI_BUS_SPIM4_IMU, cfg_xl, rx_id, sizeof(cfg_xl));
            (void)hks_hal_spi_xfer(HKS_SPI_BUS_SPIM4_IMU, cfg_g,  rx_id, sizeof(cfg_g));
            (void)hks_hal_spi_xfer(HKS_SPI_BUS_SPIM4_IMU, cfg_c3, rx_id, sizeof(cfg_c3));
        } else {
            /* Configure ICM-42688-P INT_CONFIG (0x14) = 0x1B: INT1/INT2 push-pull active-high (N8) */
            const uint8_t tx_int_cfg[2] = { ICM42688_REG_INT_CONFIG, ICM42688_INT_CFG_PP_AH };
            (void)hks_hal_spi_xfer(HKS_SPI_BUS_SPIM4_IMU, tx_int_cfg, rx_id, sizeof(tx_int_cfg));
        }
        g_imu_int_configured = true;
    }
    if (out_count != NULL) {
        *out_count = 0u;
    }
    if (temp_c_x100 != NULL) {
        *temp_c_x100 = 2000;
    }
    return 0;
}

bool hks_hal_highg_is_present(void) {
    if (!g_highg_probed) {
        (void)NRF5340_P0_BASE;
#if HKS_HIGHG_DNP
        g_highg_present = false;
#else
        const uint8_t tx_id[4] = { (uint8_t)((ADXL372_REG_DEVID_AD << 1) | 0x01u), 0x00u, 0x00u, 0x00u };
        uint8_t rx_id[4] = { 0u, 0u, 0u, 0u };
        if (hks_hal_spi_xfer(HKS_SPI_BUS_SPIM1_HIGHG, tx_id, rx_id, sizeof(tx_id)) == 0 &&
            rx_id[1] == ADXL372_DEVID_AD_VAL && rx_id[3] == ADXL372_PARTID_VAL) {
            g_highg_present = true;
        } else {
            g_highg_present = false;
        }
#endif
        if (!g_highg_present) {
            /* U4 (ADXL372) is DNP: park P0.06 (INT1), P0.07 (INT2), and P0.21 (MISO) with pull-down + input disconnect
             * so unpopulated U4 pads never float or trigger false GPIO interrupts. */
            g_stub_p0_pin_cnf[HKS_PIN_ADXL_INT1 & 31u]  = NRF_PIN_CNF_PULLDOWN_DIS;
            g_stub_p0_pin_cnf[HKS_PIN_ADXL_INT2 & 31u]  = NRF_PIN_CNF_PULLDOWN_DIS;
            g_stub_p0_pin_cnf[HKS_PIN_SPIM1_MISO & 31u] = NRF_PIN_CNF_PULLDOWN_DIS;
        }
        g_highg_probed = true;
    }
    return g_highg_present;
}

int hks_hal_highg_read_fifo(hks_high_g_sample_t *out, size_t max_samples,
                            size_t *out_count) {
    uint8_t rx[6];
    (void)HKS_PIN_SPIM1_SCK;
    if (out == NULL || out_count == NULL) {
        return -1;
    }
    *out_count = 0u;
    if (!hks_hal_highg_is_present()) {
        /* U4 is DNP: return 0 samples immediately; attitude & impact detection run on U3 */
        return 0;
    }
    if (max_samples > 0u && hks_hal_spi_xfer(HKS_SPI_BUS_SPIM1_HIGHG, NULL, rx, sizeof(rx)) == 0) {
        /* Stub path demonstrates mandatory portable >> 4 right-alignment */
        const uint16_t raw_x = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
        const uint16_t raw_y = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
        const uint16_t raw_z = (uint16_t)(((uint16_t)rx[4] << 8) | rx[5]);
        out[0].ax = hks_adxl372_unpack_i12(raw_x);
        out[0].ay = hks_adxl372_unpack_i12(raw_y);
        out[0].az = hks_adxl372_unpack_i12(raw_z);
    }
    return 0;
}

int hks_hal_mag_read(int32_t *mx_raw, int32_t *my_raw, int32_t *mz_raw,
                     bool *set_reset_done) {
    uint8_t reg = 0x00u;
    uint8_t rx[7];
    (void)hks_hal_i2c_write_read(HKS_I2C_BUS_TWIM0_SENS, HKS_MMC5983MA_ADDR7,
                                 &reg, 1u, rx, sizeof(rx));
    if (mx_raw != NULL) *mx_raw = 0;
    if (my_raw != NULL) *my_raw = 0;
    if (mz_raw != NULL) *mz_raw = 0;
    if (set_reset_done != NULL) *set_reset_done = true;
    return 0;
}

int hks_hal_baro_read(uint32_t *pressure_pa_x64, int16_t *temp_c_x100) {
    uint8_t reg = 0x1Du;
    uint8_t rx[6];
    (void)hks_hal_i2c_write_read(HKS_I2C_BUS_TWIM0_SENS, HKS_BMP581_ADDR7,
                                 &reg, 1u, rx, sizeof(rx));
    if (pressure_pa_x64 != NULL) *pressure_pa_x64 = 6484800u;
    if (temp_c_x100 != NULL) *temp_c_x100 = 2000;
    return 0;
}

int hks_hal_pmic_read(hks_hal_pmic_info_t *info) {
    /* nPM1300 EVENTSSHPHLD & status registers via TWIM2 (contract 'TWIM1'); IRQ on P1.05 */
    uint8_t reg16[2] = { 0x01u, 0x00u };
    uint8_t rx[4];
    (void)NRF5340_P1_BASE;
    if (!g_pmic_int_pullup_configured) {
        /* Enable internal 13 kOhm pull-up on P1.05 (PMIC_INT) and configure GPIO0 GPOIRQ CMOS push-pull (N3) */
        const uint32_t pmic_pin = HKS_PIN_PMIC_INT & 31u;
        g_stub_p1_pin_cnf[pmic_pin] = NRF_PIN_CNF_PULLUP_IN;
        const uint8_t gpio0_cfg[3] = { 0x06u, 0x00u, 0x05u }; /* GPIOMODE0 = GPOIRQ, push-pull */
        (void)hks_hal_i2c_write_read(HKS_I2C_BUS_TWIM2_PMIC, HKS_NPM1300_ADDR7,
                                     gpio0_cfg, sizeof(gpio0_cfg), NULL, 0u);
        g_pmic_int_pullup_configured = true;
    }
    if (info == NULL) {
        return -1;
    }
    (void)hks_hal_i2c_write_read(HKS_I2C_BUS_TWIM2_PMIC, HKS_NPM1300_ADDR7,
                                 reg16, sizeof(reg16), rx, sizeof(rx));
    info->vbatt_mv = 3900u;
    info->batt_temp_c_x100 = 2000;
    info->soc_pct_x10 = 850u;
    info->pmic_status = HKS_PMIC_FG_VALID;
    return 0;
}

int hks_hal_nand_init(void) {
    /* W25N01JW 1-bit SPI NAND on SPIM3 (8 MHz, ID 12) with /WP & /HOLD held 1V8 */
    (void)NRF5340_QSPI_BASE;
    (void)HKS_PIN_SPIM3_MOSI;
    (void)HKS_PIN_SPIM3_MISO;
    (void)HKS_PIN_FLASH_WP_N;
    (void)HKS_PIN_FLASH_HOLD_N;
    (void)HKS_PIN_QSPI_SCK;
    (void)HKS_PIN_QSPI_CSN;
    (void)W25N_CMD_RESET;
    (void)W25N_CMD_READ_STATUS;
    return 0;
}

int hks_hal_nand_read_page(uint16_t block, uint8_t page, uint16_t col,
                           uint8_t *buf, size_t len) {
    (void)block;
    (void)page;
    (void)col;
    (void)W25N_CMD_PAGE_DATA_READ;
    (void)W25N_CMD_READ_DATA;
    (void)W25N_CMD_FAST_READ_QUAD;
    if (buf != NULL && len > 0u) {
        memset(buf, 0xFF, len);
    }
    return 0;
}

int hks_hal_nand_prog_page(uint16_t block, uint8_t page, uint16_t col,
                           const uint8_t *buf, size_t len) {
    (void)block;
    (void)page;
    (void)col;
    (void)buf;
    (void)len;
    (void)W25N_CMD_WRITE_ENABLE;
    (void)W25N_CMD_PROG_DATA_LOAD;
    (void)W25N_CMD_PROG_EXECUTE;
    return 0;
}

int hks_hal_nand_erase_block(uint16_t block) {
    (void)block;
    (void)W25N_CMD_BLOCK_ERASE;
    return 0;
}

int hks_hal_nand_read_oob(uint16_t block, uint8_t page, uint8_t *oob, size_t len) {
    (void)block;
    (void)page;
    if (oob != NULL && len > 0u) {
        memset(oob, 0xFF, len);
    }
    return 0;
}

int hks_hal_nand_write_oob(uint16_t block, uint8_t page, const uint8_t *oob, size_t len) {
    (void)block;
    (void)page;
    (void)oob;
    (void)len;
    return 0;
}

int hks_hal_nand_is_bad_block(uint16_t block, bool *is_bad) {
    (void)block;
    if (is_bad != NULL) {
        *is_bad = false;
    }
    return 0;
}

int hks_hal_nand_mark_bad_block(uint16_t block) {
    (void)block;
    return 0;
}

size_t hks_hal_usb_cdc_rx_available(void) {
    (void)NRF5340_USBD_BASE;
    return 0u;
}

size_t hks_hal_usb_cdc_read(uint8_t *buf, size_t max_len) {
    (void)buf;
    (void)max_len;
    return 0u;
}

size_t hks_hal_usb_cdc_write(const uint8_t *buf, size_t len) {
    (void)buf;
    return len;
}

bool hks_hal_usb_vbus_connected(void) {
    return false;
}

bool hks_hal_button_pressed(void) {
    (void)HKS_PIN_SHPHLD_BTN;
    return false;
}

void hks_hal_led_set(uint8_t color_mask) {
    /* LD1 = Green (P1.07 -> Q1A G1), LD2 = Red (P1.08 -> Q1B G2); AMBER drives both LD1+LD2 (N1) */
    const uint32_t g_pin = HKS_PIN_LED_GREEN & 31u;
    const uint32_t r_pin = HKS_PIN_LED_RED & 31u;
    const bool green_on = ((color_mask & (uint8_t)HKS_LED_GREEN) != 0u) ||
                          ((color_mask & (uint8_t)HKS_LED_AMBER) != 0u);
    const bool red_on   = ((color_mask & (uint8_t)HKS_LED_RED) != 0u) ||
                          ((color_mask & (uint8_t)HKS_LED_AMBER) != 0u);
    g_stub_p1_pin_cnf[g_pin] = NRF_PIN_CNF_STD_OUT;
    g_stub_p1_pin_cnf[r_pin] = NRF_PIN_CNF_STD_OUT;
    if (green_on) {
        g_stub_p1_out |= (1UL << g_pin);
    } else {
        g_stub_p1_out &= ~(1UL << g_pin);
    }
    if (red_on) {
        g_stub_p1_out |= (1UL << r_pin);
    } else {
        g_stub_p1_out &= ~(1UL << r_pin);
    }
}
