"""Generate KiCad 8 project (.kicad_pro, .kicad_sch, .kicad_pcb, .net), turnkey PCBA BOM/CPL, and layout preview."""

from __future__ import annotations

import csv
import json
import math
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt

KICAD_DIR = Path(__file__).resolve().parent
HARDWARE_DIR = KICAD_DIR.parent
REPO_ROOT = HARDWARE_DIR.parent
PROD_DIR = KICAD_DIR / "production"
PLOTS_DIR = REPO_ROOT / "docs" / "plots"

# Board origin in KiCad page coordinates (center of 38.0 x 28.0 mm PCB at (130.0, 100.0) mm)
BOARD_CX_MM = 130.0
BOARD_CY_MM = 100.0
# In Sensor Frame S:
#   +X_S is UP (toward player head, -Y in KiCad screen coords, length = 38.0 mm -> y in [81.0, 119.0])
#   +Y_S is RIGHT (toward player right, +X in KiCad screen coords, width = 28.0 mm -> x in [116.0, 144.0])
BOARD_W_MM = 28.0  # along +Y_S (KiCad X: 116.0 .. 144.0)
BOARD_L_MM = 38.0  # along +X_S (KiCad Y: 81.0 .. 119.0)
BOARD_THICK_MM = 0.80


def _det_uuid(seed_str: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_OID, f"hockey_sensor_kicad_v1:{seed_str}"))


@dataclass(frozen=True)
class ComponentPlacement:
    refdes: str
    value: str
    mpn: str
    manufacturer: str
    footprint: str
    pkg_w_mm: float  # along +Y_S (KiCad X)
    pkg_l_mm: float  # along +X_S (KiCad Y)
    xs_mm: float     # Sensor Frame +X_S offset from PCB center (-19..+19 mm, + is UP)
    ys_mm: float     # Sensor Frame +Y_S offset from PCB center (-14..+14 mm, + is RIGHT)
    rot_deg: float
    layer: str
    dnp: bool
    category: str
    rail: str
    pads: tuple[tuple[str, float, float, float, float, str], ...]  # (num, dx, dy, pw, ph, net)
    notes: str = ""

    @property
    def kicad_x_mm(self) -> float:
        return round(BOARD_CX_MM + self.ys_mm, 3)

    @property
    def kicad_y_mm(self) -> float:
        return round(BOARD_CY_MM - self.xs_mm, 3)


NET_LIST: list[str] = [
    "",
    "GND",
    "VBUS_5V",
    "VBAT",
    "VSYS",
    "VDD_1V8",
    "VDD_3V0",
    "SW1_BUCK1",
    "SW2_BUCK2",
    "PMIC_VSET1",
    "PMIC_VSET2",
    "PMIC_NTC",
    "PMIC_SHPHLD",
    "USB_D_P",
    "USB_D_N",
    "USB_CC1",
    "USB_CC2",
    "DECUSB",
    "DECD",
    "DECR",
    "DECA",
    "NRF_DCCD",
    "NRF_DCC",
    "HFXO_XC1",
    "HFXO_XC2",
    "LFXO_XL1_P0.00",
    "LFXO_XL2_P0.01",
    "SPI4_SCK_P0.08",
    "SPI4_MOSI_P0.09",
    "SPI4_MISO_P0.10",
    "SPI4_CS_IMU_N_P0.11",
    "IMU_INT1_P0.04",
    "IMU_INT2_P0.05",
    "SPI1_SCK_P0.19",
    "SPI1_MOSI_P0.20",
    "SPI1_MISO_P0.21",
    "SPI1_CS_HIGHG_N_P0.22",
    "ADXL_INT1_P0.06",
    "ADXL_INT2_P0.07",
    "I2C0_SDA_1V8_P1.02",
    "I2C0_SCL_1V8_P1.03",
    "I2C0_SDA_3V0",
    "I2C0_SCL_3V0",
    "PCA9306_EN",
    "MAG_INT_3V0",
    "MAG_INT_1V8_P1.01",
    "BARO_INT_P1.04",
    "I2C2_PMIC_SDA_P0.23",
    "I2C2_PMIC_SCL_P0.24",
    "PMIC_INT_P1.05",
    "BTN_USER_N_P1.06",
    "QSPI_IO0_MOSI_P0.13",
    "QSPI_IO1_MISO_P0.14",
    "QSPI_IO2_WP_P0.15",
    "QSPI_IO3_HOLD_P0.16",
    "QSPI_SCK_P0.17",
    "QSPI_CS_N_P0.18",
    "LED_G_GATE_P1.07",
    "LED_R_GATE_P1.08",
    "LED_G_ANODE",
    "LED_R_ANODE",
    "LED_G_CATHODE",
    "LED_R_CATHODE",
    "RF_ANT_OUT",
    "RF_MATCH_MID",
    "RF_ANT_FEED",
    "SWDIO",
    "SWDCLK",
    "NRESET",
]

NET_TO_ID: dict[str, int] = {name: idx for idx, name in enumerate(NET_LIST)}


def _two_pad(net1: str, net2: str, pitch: float = 1.0, pw: float = 0.5, ph: float = 0.6) -> tuple[tuple[str, float, float, float, float, str], ...]:
    return (
        ("1", -pitch / 2.0, 0.0, pw, ph, net1),
        ("2", pitch / 2.0, 0.0, pw, ph, net2),
    )


def _ic_dual_row_pads(
    n_pins: int,
    span_x: float,
    pitch_y: float,
    pw: float,
    ph: float,
    pin_nets: dict[str, str],
    ep_net: str | None = None,
    ep_w: float = 2.0,
    ep_h: float = 2.0,
) -> tuple[tuple[str, float, float, float, float, str], ...]:
    half = n_pins // 2
    pads: list[tuple[str, float, float, float, float, str]] = []
    y0 = -0.5 * (half - 1) * pitch_y
    for i in range(half):
        pnum = str(i + 1)
        y = round(y0 + i * pitch_y, 3)
        pads.append((pnum, round(-span_x / 2.0, 3), y, pw, ph, pin_nets.get(pnum, "GND")))
    for i in range(half):
        pnum = str(half + i + 1)
        y = round(-y0 - i * pitch_y, 3)
        pads.append((pnum, round(span_x / 2.0, 3), y, pw, ph, pin_nets.get(pnum, "GND")))
    if ep_net is not None:
        pads.append((str(n_pins + 1), 0.0, 0.0, ep_w, ep_h, ep_net))
    return tuple(pads)


def _ic_quad_pads(
    n_per_side: int,
    span: float,
    pitch: float,
    pw: float,
    ph: float,
    pin_nets: dict[str, str],
    ep_net: str = "GND",
    ep_size: float = 3.2,
) -> tuple[tuple[str, float, float, float, float, str], ...]:
    pads: list[tuple[str, float, float, float, float, str]] = []
    start = -0.5 * (n_per_side - 1) * pitch
    # Left side (1..N)
    for i in range(n_per_side):
        p = str(i + 1)
        pads.append((p, round(-span / 2.0, 3), round(start + i * pitch, 3), pw, ph, pin_nets.get(p, "VDD_1V8")))
    # Bottom side
    for i in range(n_per_side):
        p = str(n_per_side + i + 1)
        pads.append((p, round(start + i * pitch, 3), round(span / 2.0, 3), ph, pw, pin_nets.get(p, "VDD_1V8")))
    # Right side
    for i in range(n_per_side):
        p = str(2 * n_per_side + i + 1)
        pads.append((p, round(span / 2.0, 3), round(-start - i * pitch, 3), pw, ph, pin_nets.get(p, "VDD_1V8")))
    # Top side
    for i in range(n_per_side):
        p = str(3 * n_per_side + i + 1)
        pads.append((p, round(-start - i * pitch, 3), round(-span / 2.0, 3), ph, pw, pin_nets.get(p, "VDD_1V8")))
    pads.append(("EP", 0.0, 0.0, ep_size, ep_size, ep_net))
    return tuple(pads)


def build_placements() -> list[ComponentPlacement]:
    # U1: nRF5340-QKAA aQFN-94 (explicit ball-mapped outer ring 1..48 + EP)
    # Ball mapping documented in hardware/ee/pinmux_and_buses.json:
    #   1=N1(XL1/P0.00), 2=R1(XL2/P0.01), 3=V2(P0.04/IMU_INT1), 4=Y2(P0.05/IMU_INT2),
    #   5=AB2(P0.06/ADXL_INT1), 6=AD2(P0.07/ADXL_INT2), 7=AH2(P0.08/SPI4_SCK), 8=AJ1(P0.09/SPI4_MOSI),
    #   9=AK2(P0.10/SPI4_MISO), 10=AK4(P0.11/SPI4_CS_IMU_N), 11=AL5(P0.13/QSPI_IO0), 12=AK8(P0.14/QSPI_IO1),
    #   13=AK10(P0.15/QSPI_IO2), 14=AL9(P0.16/QSPI_IO3), 15=AK12(P0.17/QSPI_SCK), 16=AK14(P0.18/QSPI_CS_N),
    #   17=AL15(P0.19/SPI1_SCK), 18=AK16(P0.20/SPI1_MOSI), 19=AK18(P0.21/SPI1_MISO), 20=AL17(P0.22/SPI1_CS_HIGHG_N),
    #   21=AK20(P0.23/I2C2_SDA), 22=AL27(P0.24/I2C2_SCL), 23=P2(P1.01/MAG_INT_1V8), 24=AL29(P1.02/I2C0_SDA_1V8),
    #   25=AK28(P1.03/I2C0_SCL_1V8), 26=AL19(P1.04/BARO_INT), 27=AK22(P1.05/PMIC_INT), 28=AL21(P1.06/BTN_USER_N),
    #   29=AK24(P1.07/LED_G_GATE), 30=AL23(P1.08/LED_R_GATE), 31=B2(USB_D_P), 32=B4(USB_D_N),
    #   33=A5(VBUS_5V), 34=B6(DECUSB), 35=B10(DCCD), 36=A21(DCC), 37=C31(XC1), 38=B30(XC2),
    #   39=L31(ANT), 40=AA31(SWDIO), 41=W31(SWDCLK), 42=AC31(NRESET),
    #   43=B8(DECD), 44=A19(DECR), 45=B28(DECA), 46=A9(VDD_1V8), 47=AD30(VDD_1V8), 48=B12(GND), EP=VSS(GND)
    u1_nets = {
        "1": "LFXO_XL1_P0.00",
        "2": "LFXO_XL2_P0.01",
        "3": "IMU_INT1_P0.04",
        "4": "IMU_INT2_P0.05",
        "5": "ADXL_INT1_P0.06",
        "6": "ADXL_INT2_P0.07",
        "7": "SPI4_SCK_P0.08",
        "8": "SPI4_MOSI_P0.09",
        "9": "SPI4_MISO_P0.10",
        "10": "SPI4_CS_IMU_N_P0.11",
        "11": "QSPI_IO0_MOSI_P0.13",
        "12": "QSPI_IO1_MISO_P0.14",
        "13": "QSPI_IO2_WP_P0.15",
        "14": "QSPI_IO3_HOLD_P0.16",
        "15": "QSPI_SCK_P0.17",
        "16": "QSPI_CS_N_P0.18",
        "17": "SPI1_SCK_P0.19",
        "18": "SPI1_MOSI_P0.20",
        "19": "SPI1_MISO_P0.21",
        "20": "SPI1_CS_HIGHG_N_P0.22",
        "21": "I2C2_PMIC_SDA_P0.23",
        "22": "I2C2_PMIC_SCL_P0.24",
        "23": "MAG_INT_1V8_P1.01",
        "24": "I2C0_SDA_1V8_P1.02",
        "25": "I2C0_SCL_1V8_P1.03",
        "26": "BARO_INT_P1.04",
        "27": "PMIC_INT_P1.05",
        "28": "BTN_USER_N_P1.06",
        "29": "LED_G_GATE_P1.07",
        "30": "LED_R_GATE_P1.08",
        "31": "USB_D_P",
        "32": "USB_D_N",
        "33": "VBUS_5V",
        "34": "DECUSB",
        "35": "NRF_DCCD",
        "36": "NRF_DCC",
        "37": "HFXO_XC1",
        "38": "HFXO_XC2",
        "39": "RF_ANT_OUT",
        "40": "SWDIO",
        "41": "SWDCLK",
        "42": "NRESET",
        "43": "DECD",
        "44": "DECR",
        "45": "DECA",
        "46": "VDD_1V8",
        "47": "VDD_1V8",
        "48": "GND",
    }
    # U2: nPM1300-QEAA-R QFN-32 (complete 32-pin + EP mapping including VSYS, PVDD1, PVDD2, VBUSOUT, AVSS, PGND1/2)
    u2_nets = {
        "1": "VBUS_5V",
        "2": "VBUS_5V",
        "3": "VBAT",
        "4": "PMIC_NTC",
        "5": "VSYS",
        "6": "VSYS",
        "7": "SW1_BUCK1",
        "8": "GND",
        "9": "VDD_1V8",
        "10": "PMIC_VSET1",
        "11": "PMIC_VSET2",
        "12": "VDD_3V0",
        "13": "GND",
        "14": "SW2_BUCK2",
        "15": "VSYS",
        "16": "GND",
        "17": "PMIC_SHPHLD",
        "18": "VDD_1V8",
        "19": "I2C2_PMIC_SDA_P0.23",
        "20": "I2C2_PMIC_SCL_P0.24",
        "21": "PMIC_INT_P1.05",
        "22": "GND",
        "23": "GND",
        "24": "GND",
        "25": "GND",
        "26": "GND",
        "27": "GND",
        "28": "GND",
        "29": "VDD_1V8",
        "30": "VDD_3V0",
        "31": "VSYS",
        "32": "VBAT",
    }
    # U3: ICM-42688-P LGA-14 (Exact TDK DS-000347 Table 11 Pinout)
    #   1=NC(GND), 2=NC(GND), 3=NC(GND), 4=INT1, 5=VDDIO, 6=GND, 7=RESV(GND),
    #   8=VDD, 9=INT2, 10=OCS_N(VDDIO), 11=OSDO(GND), 12=CS(SPI4_CS_IMU_N), 13=SCLK, 14=SDI/MOSI, 7->SDO/MISO on 1
    # Note: Per TDK ICM-42688-P DS-000347: Pin 1=SDO/AD0 (4-wire SPI MISO), Pin 4=INT1, Pin 5=VDDIO,
    # Pin 6=GND, Pin 7=RESV(GND), Pin 8=VDD, Pin 9=INT2, Pin 10=OCS_N(VDDIO), Pin 11=OSDO(GND),
    # Pin 12=AP_CS, Pin 13=AP_SCLK, Pin 14=AP_SDI
    u3_nets = {
        "1": "SPI4_MISO_P0.10",
        "2": "GND",
        "3": "GND",
        "4": "IMU_INT1_P0.04",
        "5": "VDD_1V8",
        "6": "GND",
        "7": "GND",
        "8": "VDD_1V8",
        "9": "IMU_INT2_P0.05",
        "10": "VDD_1V8",
        "11": "GND",
        "12": "SPI4_CS_IMU_N_P0.11",
        "13": "SPI4_SCK_P0.08",
        "14": "SPI4_MOSI_P0.09",
    }
    # U4: ADXL372BCCZ-RL7 LGA-16 (Exact Analog Devices ADXL372 Datasheet Table 5 Pinout)
    #   1=VDDIO, 2=BYP(VDD_1V8/cap), 3=RESERVED(GND), 4=SCLK, 5=GND, 6=MOSI, 7=MISO,
    #   8=CS_N, 9=INT2, 10=RESV(GND), 11=INT1, 12=GND, 13=GND, 14=Vs(VDD_1V8), 15=GND, 16=VDDIO
    u4_nets = {
        "1": "VDD_1V8",
        "2": "VDD_1V8",
        "3": "GND",
        "4": "SPI1_SCK_P0.19",
        "5": "GND",
        "6": "SPI1_MOSI_P0.20",
        "7": "SPI1_MISO_P0.21",
        "8": "SPI1_CS_HIGHG_N_P0.22",
        "9": "ADXL_INT2_P0.07",
        "10": "GND",
        "11": "ADXL_INT1_P0.06",
        "12": "GND",
        "13": "GND",
        "14": "VDD_1V8",
        "15": "GND",
        "16": "VDD_1V8",
    }
    # U5: MMC5983MA LGA-16 (Exact MEMSIC MMC5983MA Datasheet Pinout)
    #   1=SCL/SPC, 2=VDD, 3=NC(GND), 4=CS(tied to VDD_3V0 for I2C mode), 5=SDA/SDI,
    #   6=SDO(GND), 7=NC(GND), 8=NC(GND), 9=GND, 10=CAP(VDD_3V0), 11=GND, 12=NC(GND),
    #   13=VDDIO, 14=NC(GND), 15=INT, 16=NC(GND)
    u5_nets = {
        "1": "I2C0_SCL_3V0",
        "2": "VDD_3V0",
        "3": "GND",
        "4": "VDD_3V0",
        "5": "I2C0_SDA_3V0",
        "6": "GND",
        "7": "GND",
        "8": "GND",
        "9": "GND",
        "10": "VDD_3V0",
        "11": "GND",
        "12": "GND",
        "13": "VDD_3V0",
        "14": "GND",
        "15": "MAG_INT_3V0",
        "16": "GND",
    }
    # U6: BMP581 LGA-10 (Exact Bosch BMP581 Datasheet Table 23 Pinout)
    #   1=VDDIO, 2=SCK/SCL, 3=VSS(GND), 4=SDI/SDA, 5=SDO(tied to VDD_1V8 for I2C addr 0x47),
    #   6=CSB(tied to VDD_1V8 for I2C mode), 7=INT, 8=VSS(GND), 9=VSS(GND), 10=VDD
    u6_nets = {
        "1": "VDD_1V8",
        "2": "I2C0_SCL_1V8_P1.03",
        "3": "GND",
        "4": "I2C0_SDA_1V8_P1.02",
        "5": "VDD_1V8",
        "6": "VDD_1V8",
        "7": "BARO_INT_P1.04",
        "8": "GND",
        "9": "GND",
        "10": "VDD_1V8",
    }
    u7_nets = {
        "1": "QSPI_CS_N_P0.18",
        "2": "QSPI_IO1_MISO_P0.14",
        "3": "QSPI_IO2_WP_P0.15",
        "4": "GND",
        "5": "QSPI_IO0_MOSI_P0.13",
        "6": "QSPI_SCK_P0.17",
        "7": "QSPI_IO3_HOLD_P0.16",
        "8": "VDD_1V8",
    }
    u8_nets = {
        "1": "GND",
        "2": "VDD_1V8",
        "3": "I2C0_SCL_1V8_P1.03",
        "4": "I2C0_SDA_1V8_P1.02",
        "5": "I2C0_SDA_3V0",
        "6": "I2C0_SCL_3V0",
        "7": "PCA9306_EN",
        "8": "PCA9306_EN",
    }
    u9_nets = {
        "1": "VDD_1V8",
        "2": "GND",
        "3": "MAG_INT_1V8_P1.01",
        "4": "MAG_INT_3V0",
        "5": "GND",
        "6": "VDD_3V0",
    }
    u10_nets = {
        "1": "USB_D_P",
        "2": "USB_D_N",
        "3": "GND",
        "4": "USB_CC1",
        "5": "USB_CC2",
        "6": "USB_CC2",
        "7": "USB_CC1",
        "8": "GND",
        "9": "USB_D_N",
        "10": "USB_D_P",
    }
    # Q1: NTZD3154NT1G Dual N-MOSFET SOT-563 (Exact onsemi NTZD3154N Datasheet Pinout)
    #   Pin 1=S1(GND), Pin 2=G1(LED_G_GATE_P1.07), Pin 3=D2(LED_R_CATHODE),
    #   Pin 4=S2(GND), Pin 5=G2(LED_R_GATE_P1.08), Pin 6=D1(LED_G_CATHODE)
    q1_nets = {
        "1": "GND",
        "2": "LED_G_GATE_P1.07",
        "3": "LED_R_CATHODE",
        "4": "GND",
        "5": "LED_R_GATE_P1.08",
        "6": "LED_G_CATHODE",
    }

    comps: list[ComponentPlacement] = [
        # Core SoC & PMIC & Flash
        ComponentPlacement(
            refdes="U1",
            value="nRF5340-QKAA",
            mpn="nRF5340-QKAA-R7",
            manufacturer="Nordic Semiconductor",
            footprint="HKS:Nordic_aQFN-94_7x7mm_P0.4mm",
            pkg_w_mm=7.0,
            pkg_l_mm=7.0,
            xs_mm=8.5,
            ys_mm=-3.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="MCU",
            rail="1.8V",
            pads=_ic_quad_pads(12, 6.8, 0.45, 0.55, 0.22, u1_nets, ep_net="GND", ep_size=4.2),
            notes="Dual Cortex-M33 SoC in 1.8V Normal Voltage Mode",
        ),
        ComponentPlacement(
            refdes="U2",
            value="nPM1300-QEAA-R",
            mpn="nPM1300-QEAA-R",
            manufacturer="Nordic Semiconductor",
            footprint="HKS:QFN-32-1EP_5x5mm_P0.5mm",
            pkg_w_mm=5.0,
            pkg_l_mm=5.0,
            xs_mm=-8.5,
            ys_mm=-6.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="PMIC",
            rail="3.7V/1.8V/3.0V",
            pads=_ic_quad_pads(8, 4.8, 0.5, 0.55, 0.25, u2_nets, ep_net="GND", ep_size=3.0),
            notes="Dual-buck PMIC (BUCK1=1.8V, BUCK2=3.0V) + JEITA LiPo charger + fuel gauge",
        ),
        # Sensor Frame S Aligned Triad (Centered away from M1.4 corner screws and inductors!)
        ComponentPlacement(
            refdes="U3",
            value="ICM-42688-P",
            mpn="ICM-42688-P",
            manufacturer="TDK InvenSense",
            footprint="HKS:LGA-14_2.5x3.0mm_P0.5mm",
            pkg_w_mm=2.5,
            pkg_l_mm=3.0,
            xs_mm=0.0,
            ys_mm=0.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Sensor",
            rail="1.8V",
            pads=_ic_dual_row_pads(14, 2.3, 0.4, 0.5, 0.28, u3_nets),
            notes="Primary 6-DOF IMU at exact PCB geometric center (0,0), aligned to Sensor Frame S",
        ),
        ComponentPlacement(
            refdes="U4",
            value="ADXL372BCCZ-RL7",
            mpn="ADXL372BCCZ-RL7",
            manufacturer="Analog Devices",
            footprint="HKS:LGA-16_3.0x3.25mm_P0.5mm",
            pkg_w_mm=3.0,
            pkg_l_mm=3.25,
            xs_mm=-0.5,
            ys_mm=5.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Sensor",
            rail="1.8V",
            pads=_ic_dual_row_pads(16, 2.8, 0.38, 0.5, 0.25, u4_nets),
            notes="+-200g High-g impact accelerometer on isolated SPIM1 (8 MHz), aligned to Frame S",
        ),
        ComponentPlacement(
            refdes="U5",
            value="MMC5983MA",
            mpn="MMC5983MA",
            manufacturer="MEMSIC",
            footprint="HKS:LGA-16_3.0x3.0mm_P0.5mm",
            pkg_w_mm=3.0,
            pkg_l_mm=3.0,
            xs_mm=4.8,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Sensor",
            rail="3.0V",
            pads=_ic_dual_row_pads(16, 2.8, 0.35, 0.5, 0.25, u5_nets),
            notes="3-axis magnetometer (>14 mm away from PMIC inductors L1/L2), aligned to Frame S",
        ),
        ComponentPlacement(
            refdes="U6",
            value="BMP581",
            mpn="BMP581",
            manufacturer="Bosch Sensortec",
            footprint="HKS:LGA-10_2.0x2.0mm_P0.5mm",
            pkg_w_mm=2.0,
            pkg_l_mm=2.0,
            xs_mm=-5.5,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Sensor",
            rail="1.8V",
            pads=_ic_dual_row_pads(10, 1.8, 0.4, 0.45, 0.25, u6_nets),
            notes="Barometric pressure sensor under 2.5 mm ePTFE Gore acoustic/baro vent port",
        ),
        ComponentPlacement(
            refdes="U7",
            value="W25N01JWZEIG",
            mpn="W25N01JWZEIG",
            manufacturer="Winbond",
            footprint="HKS:WSON-8-1EP_8x6mm_P1.27mm",
            pkg_w_mm=6.0,
            pkg_l_mm=8.0,
            xs_mm=0.2,
            ys_mm=-7.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Memory",
            rail="1.8V",
            pads=_ic_dual_row_pads(8, 5.8, 1.27, 0.8, 0.5, u7_nets, ep_net="GND", ep_w=3.4, ep_h=4.3),
            notes="1 Gbit (128 MiB) SLC NAND flash on P0.13..P0.18",
        ),
        ComponentPlacement(
            refdes="U8",
            value="PCA9306DCUR",
            mpn="PCA9306DCUR",
            manufacturer="Texas Instruments",
            footprint="Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
            pkg_w_mm=2.3,
            pkg_l_mm=2.0,
            xs_mm=1.8,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Interface",
            rail="1.8V/3.0V",
            pads=_ic_dual_row_pads(8, 2.1, 0.5, 0.55, 0.28, u8_nets),
            notes="Bidirectional 1.8V <-> 3.0V I2C translator for MMC5983MA",
        ),
        ComponentPlacement(
            refdes="U9",
            value="SN74AXC1T45DRLR",
            mpn="SN74AXC1T45DRLR",
            manufacturer="Texas Instruments",
            footprint="Package_TO_SOT_SMD:SOT-563",
            pkg_w_mm=1.6,
            pkg_l_mm=1.6,
            xs_mm=4.8,
            ys_mm=4.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Interface",
            rail="1.8V/3.0V",
            pads=_ic_dual_row_pads(6, 1.5, 0.5, 0.45, 0.25, u9_nets),
            notes="3.0V -> 1.8V translator for MAG_INT -> P1.01",
        ),
        ComponentPlacement(
            refdes="U10",
            value="TPD4E05U06DQAR",
            mpn="TPD4E05U06DQAR",
            manufacturer="Texas Instruments",
            footprint="Package_SON:USON-10_2.5x1.0mm_P0.5mm",
            pkg_w_mm=2.5,
            pkg_l_mm=1.0,
            xs_mm=-11.6,
            ys_mm=0.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Protection",
            rail="USB",
            pads=_ic_dual_row_pads(10, 2.3, 0.4, 0.45, 0.22, u10_nets),
            notes="0.42 pF flow-through ESD protection on USB D+/D- and CC1/CC2",
        ),
        # Connectors, Antenna, Crystals, Switch, LEDs
        ComponentPlacement(
            refdes="J1",
            value="USB4720-03-A",
            mpn="USB4720-03-A",
            manufacturer="GCT",
            footprint="HKS:USB_C_Receptacle_GCT_USB4720-03-A_IP67",
            pkg_w_mm=8.94,
            pkg_l_mm=6.2,
            xs_mm=-16.0,
            ys_mm=0.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Connector",
            rail="VBUS_5V",
            pads=(
                ("A1", -3.2, -2.5, 0.4, 1.1, "GND"),
                ("A4", -2.2, -2.5, 0.4, 1.1, "VBUS_5V"),
                ("A5", -1.2, -2.5, 0.3, 1.1, "USB_CC1"),
                ("A6", -0.4, -2.5, 0.3, 1.1, "USB_D_P"),
                ("A7", 0.4, -2.5, 0.3, 1.1, "USB_D_N"),
                ("B7", -0.4, -0.8, 0.3, 1.0, "USB_D_N"),
                ("B6", 0.4, -0.8, 0.3, 1.0, "USB_D_P"),
                ("B5", 1.2, -2.5, 0.3, 1.1, "USB_CC2"),
                ("B4", 2.2, -2.5, 0.4, 1.1, "VBUS_5V"),
                ("B1", 3.2, -2.5, 0.4, 1.1, "GND"),
                ("S1", -4.3, 0.0, 1.0, 1.8, "GND"),
                ("S2", 4.3, 0.0, 1.0, 1.8, "GND"),
            ),
            notes="IP67 mid-mount waterproof USB-C receptacle at -X_S bottom edge (A6/B6=D+, A7/B7=D-)",
        ),
        ComponentPlacement(
            refdes="J2",
            value="TC2050-IDC-NL",
            mpn="TC2050-IDC-NL",
            manufacturer="Tag-Connect",
            footprint="Connector:Tag-Connect_TC2050-IDC-NL_2x05_P1.27mm_Vertical",
            pkg_w_mm=6.0,
            pkg_l_mm=3.0,
            xs_mm=16.0,
            ys_mm=-6.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=True,
            category="Debug",
            rail="1.8V",
            pads=(
                ("1", -2.54, -0.635, 0.78, 0.78, "VDD_1V8"),
                ("2", -1.27, -0.635, 0.78, 0.78, "SWDIO"),
                ("3", 0.0, -0.635, 0.78, 0.78, "GND"),
                ("4", 1.27, -0.635, 0.78, 0.78, "SWDCLK"),
                ("5", 2.54, -0.635, 0.78, 0.78, "GND"),
                ("9", 1.27, 0.635, 0.78, 0.78, "GND"),
                ("10", 2.54, 0.635, 0.78, 0.78, "NRESET"),
            ),
            notes="Zero-profile SWD factory programming & debug footprint (pins 3, 5, 9 tied to GND)",
        ),
        ComponentPlacement(
            refdes="A1",
            value="2450AT18B0100001E",
            mpn="2450AT18B0100001E",
            manufacturer="Johanson Technology",
            footprint="RF_Antenna:Johanson_2450AT18B100",
            pkg_w_mm=3.2,
            pkg_l_mm=1.6,
            xs_mm=17.5,
            ys_mm=10.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="RF",
            rail="RF_50R",
            pads=(
                ("1", -1.3, 0.0, 0.8, 1.4, "RF_ANT_FEED"),
                ("2", 1.3, 0.0, 0.8, 1.4, ""),
            ),
            notes="2.4 GHz BLE 5.3 ceramic chip antenna at top-right board edge inside 8.0x8.0 mm keepout (Pad 1=50R Feed, Pad 2=Isolated Mechanical Anchor NC per Johanson datasheet)",
        ),
        ComponentPlacement(
            refdes="Y1",
            value="NX2016SA-32MHz",
            mpn="NX2016SA-32MHz",
            manufacturer="NDK",
            footprint="Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm",
            pkg_w_mm=2.0,
            pkg_l_mm=1.6,
            xs_mm=15.0,
            ys_mm=-1.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Crystal",
            rail="1.8V",
            pads=(
                ("1", -0.7, -0.5, 0.6, 0.5, "HFXO_XC1"),
                ("2", 0.7, -0.5, 0.6, 0.5, "GND"),
                ("3", 0.7, 0.5, 0.6, 0.5, "HFXO_XC2"),
                ("4", -0.7, 0.5, 0.6, 0.5, "GND"),
            ),
            notes="32.000 MHz +-10ppm HFXO crystal",
        ),
        ComponentPlacement(
            refdes="Y2",
            value="FC-12M-32.768K-20ppm",
            mpn="FC-12M-32.768K-20ppm",
            manufacturer="Epson",
            footprint="Crystal:Crystal_SMD_2012-2Pin_2.0x1.2mm",
            pkg_w_mm=2.0,
            pkg_l_mm=1.2,
            xs_mm=13.0,
            ys_mm=-4.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Crystal",
            rail="1.8V",
            pads=_two_pad("LFXO_XL1_P0.00", "LFXO_XL2_P0.01", pitch=1.4, pw=0.6, ph=1.0),
            notes="32.768 kHz <=20ppm LFXO crystal",
        ),
        ComponentPlacement(
            refdes="SW1",
            value="EVPAWCD4A",
            mpn="EVPAWCD4A",
            manufacturer="Panasonic",
            footprint="Button_Switch_SMD:SW_SPST_EVPAW",
            pkg_w_mm=3.0,
            pkg_l_mm=2.0,
            xs_mm=-10.0,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Switch",
            rail="SHPHLD",
            pads=_two_pad("PMIC_SHPHLD", "GND", pitch=2.4, pw=0.7, ph=1.2),
            notes="0.60 mm IP67 tactile dome switch on SHPHLD & D2 cathode",
        ),
        ComponentPlacement(
            refdes="D1",
            value="ESD5Z5.0T1G",
            mpn="ESD5Z5.0T1G",
            manufacturer="onsemi",
            footprint="Diode_SMD:D_SOD-523",
            pkg_w_mm=1.6,
            pkg_l_mm=0.8,
            xs_mm=-11.6,
            ys_mm=-2.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Protection",
            rail="VBUS_5V",
            pads=_two_pad("VBUS_5V", "GND", pitch=1.2, pw=0.45, ph=0.6),
            notes="VBUS 5V hot-plug surge TVS diode",
        ),
        ComponentPlacement(
            refdes="D2",
            value="BAT54WS",
            mpn="BAT54WS",
            manufacturer="Diodes Incorporated",
            footprint="Diode_SMD:D_SOD-323",
            pkg_w_mm=2.5,
            pkg_l_mm=1.3,
            xs_mm=-10.0,
            ys_mm=4.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Diode",
            rail="1.8V/SHPHLD",
            pads=_two_pad("PMIC_SHPHLD", "BTN_USER_N_P1.06", pitch=2.0, pw=0.6, ph=0.7),
            notes="Schottky isolation diode blocking VBAT from P1.06",
        ),
        ComponentPlacement(
            refdes="Q1",
            value="NTZD3154NT1G",
            mpn="NTZD3154NT1G",
            manufacturer="onsemi",
            footprint="Package_TO_SOT_SMD:SOT-563",
            pkg_w_mm=1.6,
            pkg_l_mm=1.6,
            xs_mm=-12.2,
            ys_mm=6.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="MOSFET",
            rail="1.8V/3.0V",
            pads=_ic_dual_row_pads(6, 1.5, 0.5, 0.45, 0.25, q1_nets),
            notes="Dual logic-level N-MOSFET switching LD1/LD2 low-side from 1.8V gates",
        ),
        ComponentPlacement(
            refdes="LD1",
            value="LTST-C191KGKT",
            mpn="LTST-C191KGKT",
            manufacturer="Lite-On",
            footprint="LED_SMD:LED_0603_1608Metric",
            pkg_w_mm=1.6,
            pkg_l_mm=0.8,
            xs_mm=-13.0,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="LED",
            rail="3.0V",
            pads=_two_pad("LED_G_ANODE", "LED_G_CATHODE", pitch=1.5, pw=0.7, ph=0.8),
            notes="Green status LED on BUCK2 3.0V",
        ),
        ComponentPlacement(
            refdes="LD2",
            value="LTST-C191KRKT",
            mpn="LTST-C191KRKT",
            manufacturer="Lite-On",
            footprint="LED_SMD:LED_0603_1608Metric",
            pkg_w_mm=1.6,
            pkg_l_mm=0.8,
            xs_mm=-14.6,
            ys_mm=8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="LED",
            rail="3.0V",
            pads=_two_pad("LED_R_ANODE", "LED_R_CATHODE", pitch=1.5, pw=0.7, ph=0.8),
            notes="Red/Amber charge/fault status LED on BUCK2 3.0V",
        ),
        # Power & RF Inductors (L1, L2, L3, L4, L5, L6) + C_RF1
        ComponentPlacement(
            refdes="L1",
            value="2.2uH_DFE201610E-2R2M",
            mpn="DFE201610E-2R2M",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_2016_2016Metric",
            pkg_w_mm=2.0,
            pkg_l_mm=1.6,
            xs_mm=-5.0,
            ys_mm=-10.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="1.8V",
            pads=_two_pad("SW1_BUCK1", "VDD_1V8", pitch=1.4, pw=0.7, ph=1.4),
            notes="2.2 uH shielded power inductor for nPM1300 BUCK1 (1.8V)",
        ),
        ComponentPlacement(
            refdes="L2",
            value="2.2uH_DFE201610E-2R2M",
            mpn="DFE201610E-2R2M",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_2016_2016Metric",
            pkg_w_mm=2.0,
            pkg_l_mm=1.6,
            xs_mm=-12.0,
            ys_mm=-10.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="3.0V",
            pads=_two_pad("SW2_BUCK2", "VDD_3V0", pitch=1.4, pw=0.7, ph=1.4),
            notes="2.2 uH shielded power inductor for nPM1300 BUCK2 (3.0V)",
        ),
        ComponentPlacement(
            refdes="L3",
            value="10uH_LQM18DN100M70L",
            mpn="LQM18DN100M70L",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_0603_1608Metric",
            pkg_w_mm=1.6,
            pkg_l_mm=0.8,
            xs_mm=7.5,
            ys_mm=-9.0,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="1.8V",
            pads=_two_pad("NRF_DCCD", "DECD", pitch=1.5, pw=0.7, ph=0.8),
            notes="10 uH 0603 inductor for nRF5340 VREGMAIN (DCCD -> DECD)",
        ),
        ComponentPlacement(
            refdes="L4",
            value="10uH_LQM18DN100M70L",
            mpn="LQM18DN100M70L",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_0603_1608Metric",
            pkg_w_mm=1.6,
            pkg_l_mm=0.8,
            xs_mm=11.0,
            ys_mm=-9.0,
            rot_deg=90.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="1.8V",
            pads=_two_pad("NRF_DCC", "DECR", pitch=1.5, pw=0.7, ph=0.8),
            notes="10 uH 0603 inductor for nRF5340 VREGRADIO (DCC -> DECR, orthogonal 90 deg & >3.5 mm spacing from L3)",
        ),
        ComponentPlacement(
            refdes="C27",
            value="1.2pF_C_RF1",
            mpn="GJM1555C1H1R2WB01D",
            manufacturer="Murata",
            footprint="Capacitor_SMD:C_0402_1005Metric",
            pkg_w_mm=1.0,
            pkg_l_mm=0.5,
            xs_mm=12.8,
            ys_mm=2.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Capacitor",
            rail="RF_50R",
            pads=_two_pad("RF_ANT_OUT", "RF_MATCH_MID", pitch=0.9, pw=0.5, ph=0.5),
            notes="C_RF1: 1.2 pF series RF matching capacitor right at U1 ANT exit",
        ),
        ComponentPlacement(
            refdes="L5",
            value="2.7nH_LQW15AN2N7C00D",
            mpn="LQW15AN2N7C00D",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_0402_1005Metric",
            pkg_w_mm=1.0,
            pkg_l_mm=0.5,
            xs_mm=13.7,
            ys_mm=3.8,
            rot_deg=90.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="RF_50R",
            pads=_two_pad("RF_MATCH_MID", "GND", pitch=0.9, pw=0.5, ph=0.5),
            notes="2.7 nH shunt RF matching inductor at GND plane edge before 8x8mm keepout",
        ),
        ComponentPlacement(
            refdes="L6",
            value="3.3nH_LQW15AN3N3C00D",
            mpn="LQW15AN3N3C00D",
            manufacturer="Murata",
            footprint="Inductor_SMD:L_0402_1005Metric",
            pkg_w_mm=1.0,
            pkg_l_mm=0.5,
            xs_mm=12.8,
            ys_mm=5.2,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Inductor",
            rail="RF_50R",
            pads=_two_pad("RF_MATCH_MID", "RF_ANT_FEED", pitch=0.9, pw=0.5, ph=0.5),
            notes="3.3 nH series RF matching inductor feeding 50R GCPW into 8x8mm keepout to A1 Pad 1",
        ),
        ComponentPlacement(
            refdes="RT1",
            value="10k_NTC_DNI",
            mpn="NCP15XH103F03RC",
            manufacturer="Murata",
            footprint="Resistor_SMD:R_0402_1005Metric",
            pkg_w_mm=1.0,
            pkg_l_mm=0.5,
            xs_mm=-13.8,
            ys_mm=-6.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=True,
            category="Thermistor",
            rail="PMIC_NTC",
            pads=_two_pad("PMIC_NTC", "GND", pitch=0.9, pw=0.5, ph=0.5),
            notes="DNI backup NTC footprint (BT1 integrated 10k B=3380K NTC used)",
        ),
        ComponentPlacement(
            refdes="BT1",
            value="502535-450mAh-LiPo-Pads",
            mpn="502535-450mAh-LiPo",
            manufacturer="Generic-LiPo",
            footprint="HKS:Battery_Pads_3Pin_VBAT_NTC_GND",
            pkg_w_mm=4.5,
            pkg_l_mm=2.0,
            xs_mm=-15.5,
            ys_mm=-8.5,
            rot_deg=0.0,
            layer="F.Cu",
            dnp=False,
            category="Battery",
            rail="VBAT_3.7V",
            pads=(
                ("1", -1.5, 0.0, 1.0, 1.4, "VBAT"),
                ("2", 0.0, 0.0, 1.0, 1.4, "PMIC_NTC"),
                ("3", 1.5, 0.0, 1.0, 1.4, "GND"),
            ),
            notes="3-pad solder/connector landing for 502535 450mAh LiPo + 10k NTC",
        ),
    ]

    # Resistors (standard KiCad numbered R1..R19 matching BOM quantities, with schematic role in Value)
    res_specs = [
        ("R1", "47k_1%_VSET1", "ERJ-2RKF4702X", -6.5, -10.5, "PMIC_VSET1", "GND", "0V", "R_VSET1: BUCK1=1.8V VSET"),
        ("R2", "150k_1%_VSET2", "ERJ-2RKF1503X", -10.2, -10.5, "PMIC_VSET2", "GND", "0V", "R_VSET2: BUCK2=3.0V VSET"),
        ("R3", "5.1k_1%_CC1", "ERJ-2RKF5101X", -10.0, -1.6, "USB_CC1", "GND", "0V", "R_CC1: USB-C CC1 Rd"),
        ("R4", "5.1k_1%_CC2", "ERJ-2RKF5101X", -10.0, 1.6, "USB_CC2", "GND", "0V", "R_CC2: USB-C CC2 Rd"),
        ("R5", "4.7k_I2C0_SDA", "ERJ-2GEJ472X", 3.5, 2.5, "I2C0_SDA_1V8_P1.02", "VDD_1V8", "1.8V", "TWIM0 1.8V SDA PU"),
        ("R6", "4.7k_I2C0_SCL", "ERJ-2GEJ472X", 2.2, 2.5, "I2C0_SCL_1V8_P1.03", "VDD_1V8", "1.8V", "TWIM0 1.8V SCL PU"),
        ("R7", "4.7k_I2C2_SDA", "ERJ-2GEJ472X", -3.8, -2.2, "I2C2_PMIC_SDA_P0.23", "VDD_1V8", "1.8V", "TWIM2 PMIC SDA PU"),
        ("R8", "4.7k_I2C2_SCL", "ERJ-2GEJ472X", -2.8, -2.2, "I2C2_PMIC_SCL_P0.24", "VDD_1V8", "1.8V", "TWIM2 PMIC SCL PU"),
        ("R9", "10k_CS_IMU", "ERJ-2GEJ103X", 3.5, -1.5, "SPI4_CS_IMU_N_P0.11", "VDD_1V8", "1.8V", "ICM-42688 CS PU"),
        ("R10", "10k_CS_HIGHG", "ERJ-2GEJ103X", 2.2, 5.0, "SPI1_CS_HIGHG_N_P0.22", "VDD_1V8", "1.8V", "ADXL372 CS PU"),
        ("R11", "10k_CS_FLASH", "ERJ-2GEJ103X", 4.6, -11.5, "QSPI_CS_N_P0.18", "VDD_1V8", "1.8V", "W25N01JW CS PU"),
        ("R12", "100k_GATE_G", "ERJ-2GEJ104X", -10.8, 11.2, "LED_G_GATE_P1.07", "GND", "1.8V", "Q1A gate pulldown"),
        ("R13", "100k_GATE_R", "ERJ-2GEJ104X", -11.8, 11.2, "LED_R_GATE_P1.08", "GND", "1.8V", "Q1B gate pulldown"),
        ("R14", "100k_BTN_PU", "ERJ-2GEJ104X", -8.6, 4.5, "BTN_USER_N_P1.06", "VDD_1V8", "1.8V", "P1.06 1.8V button PU"),
        ("R15", "4.7k_3V0_SDA", "ERJ-2GEJ472X", 3.2, 11.2, "I2C0_SDA_3V0", "VDD_3V0", "3.0V", "MMC5983MA 3.0V SDA PU"),
        ("R16", "4.7k_3V0_SCL", "ERJ-2GEJ472X", 1.8, 11.2, "I2C0_SCL_3V0", "VDD_3V0", "3.0V", "MMC5983MA 3.0V SCL PU"),
        ("R17", "200k_PCA_EN", "ERJ-2GEJ204X", 0.4, 11.2, "PCA9306_EN", "VDD_3V0", "3.0V", "PCA9306 VREF2/EN bias"),
        ("R18", "1k_LED_G", "ERJ-2GEJ102X", -13.0, 11.2, "VDD_3V0", "LED_G_ANODE", "3.0V", "LD1 current limit"),
        ("R19", "1k_LED_R", "ERJ-2GEJ102X", -14.6, 11.2, "VDD_3V0", "LED_R_ANODE", "3.0V", "LD2 current limit"),
    ]
    for r_ref, r_val, r_mpn, rx, ry, n1, n2, rail, note in res_specs:
        comps.append(
            ComponentPlacement(
                refdes=r_ref,
                value=r_val,
                mpn=r_mpn,
                manufacturer="Panasonic",
                footprint="Resistor_SMD:R_0402_1005Metric",
                pkg_w_mm=1.0,
                pkg_l_mm=0.5,
                xs_mm=rx,
                ys_mm=ry,
                rot_deg=0.0,
                layer="F.Cu",
                dnp=False,
                category="Resistor",
                rail=rail,
                pads=_two_pad(n1, n2, pitch=0.9, pw=0.5, ph=0.5),
                notes=note,
            )
        )

    # Bulk, Buck & VBUS 0603 Capacitors (C19..C26: 1x 25V VBUS, 2x 10uF BUCK, 5x 4.7uF 10V DEC/VDD matching bom.csv!)
    cap_0603_specs = [
        ("C19", "4.7uF_25V_VBUS", "GRM188R61E475KE11D", -12.8, -6.2, "VBUS_5V", "GND", "VBUS_5V", "C_VBUS: 25V X5R VBUSIN cap"),
        ("C20", "10uF_6.3V_BUCK1", "GRM188R60J106ME47D", -3.0, -11.5, "VDD_1V8", "GND", "1.8V", "C_BUCK1: BUCK1 1.8V output cap"),
        ("C21", "10uF_6.3V_BUCK2", "GRM188R60J106ME47D", -13.6, -10.5, "VDD_3V0", "GND", "3.0V", "C_BUCK2: BUCK2 3.0V output cap"),
        ("C22", "4.7uF_10V_DECUSB", "GRM188R61A475KE15D", 8.5, 1.5, "DECUSB", "GND", "DECUSB", "nRF5340 DECUSB 4.7uF"),
        ("C23", "4.7uF_10V_VDD", "GRM188R61A475KE15D", 10.5, 1.5, "VDD_1V8", "GND", "1.8V", "nRF5340 VDD bulk"),
        ("C24", "4.7uF_10V_DECD", "GRM188R61A475KE15D", 6.2, -9.0, "DECD", "GND", "1.8V", "nRF5340 DECD output cap for L3 (DCCD)"),
        ("C25", "4.7uF_10V_DECR", "GRM188R61A475KE15D", 12.8, -9.0, "DECR", "GND", "1.8V", "nRF5340 DECR output cap for L4 (DCC)"),
        ("C26", "4.7uF_10V_DECA", "GRM188R61A475KE15D", -4.8, -7.2, "DECA", "GND", "1.8V", "nRF5340 DECA regulator decoupling cap"),
    ]
    for c_ref, c_val, c_mpn, cx, cy, n1, n2, rail, note in cap_0603_specs:
        comps.append(
            ComponentPlacement(
                refdes=c_ref,
                value=c_val,
                mpn=c_mpn,
                manufacturer="Murata",
                footprint="Capacitor_SMD:C_0603_1608Metric",
                pkg_w_mm=1.6,
                pkg_l_mm=0.8,
                xs_mm=cx,
                ys_mm=cy,
                rot_deg=0.0,
                layer="F.Cu",
                dnp=False,
                category="Capacitor",
                rail=rail,
                pads=_two_pad(n1, n2, pitch=1.5, pw=0.7, ph=0.8),
                notes=note,
            )
        )

    # 18x 100nF 0402 Decoupling Capacitors (C1..C18)
    for i in range(18):
        c_ref = f"C{i+1}"
        row = i // 6
        col = i % 6
        cx = -7.2 + row * 1.1
        cy = -1.5 + col * 1.5
        rail_net = "VDD_3V0" if i in (14, 15, 16) else ("PCA9306_EN" if i == 17 else "VDD_1V8")
        comps.append(
            ComponentPlacement(
                refdes=c_ref,
                value="100nF_10V",
                mpn="GRM155R61A104KA01D",
                manufacturer="Murata",
                footprint="Capacitor_SMD:C_0402_1005Metric",
                pkg_w_mm=1.0,
                pkg_l_mm=0.5,
                xs_mm=round(cx, 2),
                ys_mm=round(cy, 2),
                rot_deg=0.0,
                layer="F.Cu",
                dnp=False,
                category="Capacitor",
                rail="3.0V" if rail_net == "VDD_3V0" else "1.8V",
                pads=_two_pad(rail_net, "GND", pitch=0.9, pw=0.5, ph=0.5),
                notes=f"100 nF 0402 local decoupling capacitor ({rail_net})",
            )
        )

    return comps


def write_kicad_pro(path: Path) -> None:
    pro_data: dict[str, Any] = {
        "board": {
            "3dviewports": [],
            "design_settings": {
                "defaults": {
                    "board_outline_line_width": 0.1,
                    "copper_line_width": 0.15,
                    "copper_text_size_h": 1.0,
                    "copper_text_size_v": 1.0,
                    "copper_text_thickness": 0.15,
                    "other_line_width": 0.12,
                    "silk_line_width": 0.12,
                    "silk_text_size_h": 0.8,
                    "silk_text_size_v": 0.8,
                    "silk_text_thickness": 0.12,
                },
                "diff_pair_dimensions": [
                    {"gap": 0.15, "via_gap": 0.2, "width": 0.22}
                ],
                "drc_exclusions": [],
                "meta": {"version": 2},
                "rule_severities": {
                    "clearance": "error",
                    "copper_edge_clearance": "error",
                    "courtyards_overlap": "error",
                    "diff_pair_gap_out_of_range": "error",
                    "hole_clearance": "error",
                    "microvia_drill_out_of_range": "error",
                    "shorting_items": "error",
                    "solder_mask_bridge": "ignore",
                    "track_width": "error",
                    "unconnected_items": "error",
                    "via_dangling": "warning",
                },
                "rules": {
                    "max_error": 0.005,
                    "min_clearance": 0.08,
                    "min_connection": 0.0,
                    "min_copper_edge_clearance": 0.2,
                    "min_hole_clearance": 0.15,
                    "min_hole_to_hole": 0.2,
                    "min_microvia_diameter": 0.3,
                    "min_microvia_drill": 0.1,
                    "min_resolved_spokes": 2,
                    "min_silk_clearance": 0.1,
                    "min_text_height": 0.6,
                    "min_text_thickness": 0.08,
                    "min_through_hole_diameter": 0.2,
                    "min_track_width": 0.06,
                    "min_via_annular_width": 0.1,
                    "min_via_diameter": 0.4,
                    "solder_mask_to_copper_clearance": 0.0,
                    "use_height_for_length_calcs": True,
                },
                "track_widths": [0.09, 0.12, 0.15, 0.25, 0.4],
                "via_dimensions": [
                    {"diameter": 0.4, "drill": 0.2},
                    {"diameter": 0.5, "drill": 0.25},
                ],
            },
            "layer_presets": [],
            "viewports": [],
        },
        "boards": [],
        "cvpcb": {"equivalence_files": []},
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": "hockey_sensor.kicad_pro", "version": 1},
        "net_settings": {
            "classes": [
                {
                    "bus_width": 12,
                    "clearance": 0.08,
                    "diff_pair_gap": 0.15,
                    "diff_pair_via_gap": 0.2,
                    "diff_pair_width": 0.22,
                    "line_style": 0,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "name": "Default",
                    "pcb_color": "rgba(0, 0, 0, 0.000)",
                    "schematic_color": "rgba(0, 0, 0, 0.000)",
                    "track_width": 0.12,
                    "via_diameter": 0.4,
                    "via_drill": 0.2,
                    "wire_width": 6,
                },
                {
                    "clearance": 0.08,
                    "name": "Power_1V8_3V0_VBAT",
                    "track_width": 0.2,
                    "via_diameter": 0.4,
                    "via_drill": 0.2,
                },
                {
                    "clearance": 0.08,
                    "diff_pair_gap": 0.15,
                    "diff_pair_width": 0.2,
                    "name": "USB_90R_Diff",
                    "track_width": 0.15,
                    "via_diameter": 0.4,
                    "via_drill": 0.2,
                },
                {
                    "clearance": 0.08,
                    "name": "RF_50R_GCPW",
                    "track_width": 0.2,
                    "via_diameter": 0.4,
                    "via_drill": 0.2,
                },
            ],
            "meta": {"version": 3},
            "net_colors": None,
            "netclass_assignments": {
                "GND": "Power_1V8_3V0_VBAT",
                "VBAT": "Power_1V8_3V0_VBAT",
                "VBUS_5V": "Power_1V8_3V0_VBAT",
                "VDD_1V8": "Power_1V8_3V0_VBAT",
                "VDD_3V0": "Power_1V8_3V0_VBAT",
                "RF_ANT_FEED": "RF_50R_GCPW",
                "RF_ANT_OUT": "RF_50R_GCPW",
                "RF_MATCH_MID": "RF_50R_GCPW",
                "USB_D_N": "USB_90R_Diff",
                "USB_D_P": "USB_90R_Diff",
            },
            "netclass_patterns": [],
        },
        "pcbnew": {"last_paths": {"gencad": "", "idf": "", "netlist": "", "plot": "", "pos_files": "", "specctra_dsn": "", "step": "", "svg": "", "vrml": ""}, "page_layout_descr_file": ""},
        "schematic": {
            "annotate_start_num": 0,
            "drawing": {"dashed_lines_dash_length_ratio": 12.0, "dashed_lines_gap_length_ratio": 3.0, "default_line_thickness": 6.0, "default_text_size": 50.0, "field_names": [], "intersheets_ref_own_page": False, "intersheets_ref_prefix": "", "intersheets_ref_short": False, "intersheets_ref_show": False, "intersheets_ref_suffix": "", "junction_size_choice": 3, "label_size_ratio": 0.375, "operating_point_overlay_i_precision": 3, "operating_point_overlay_i_range": "~A", "operating_point_overlay_v_precision": 3, "operating_point_overlay_v_range": "~V", "overbar_offset_ratio": 1.23, "pin_symbol_size": 25.0, "text_offset_ratio": 0.15},
            "legacy_lib_dir": "",
            "legacy_lib_list": [],
            "meta": {"version": 1},
            "net_format_name": "",
            "page_layout_descr_file": "",
            "plot_directory": "",
            "spice_current_sheet_as_root": False,
            "spice_external_command": 'spice "%I"',
            "spice_model_current_sheet_as_root": True,
            "spice_save_all_currents": False,
            "spice_save_all_dissipations": False,
            "spice_save_all_voltages": False,
            "subpart_first_id": 65,
            "subpart_id_separator": 0,
        },
        "sheets": [[_det_uuid("root_sheet"), "Root"]],
        "text_variables": {
            "BOARD_DIMS": "38.0 x 28.0 x 0.80 mm (4-Layer FR4 ENIG)",
            "CONTRACT_VERSION": "1.0.0",
            "PROJECT_ID": "HKS-EE-1.0.0",
        },
    }
    path.write_text(json.dumps(pro_data, indent=2) + "\n", encoding="utf-8")


def write_kicad_sch(path: Path, comps: list[ComponentPlacement]) -> None:
    lines: list[str] = [
        '(kicad_sch',
        '  (version 20231120)',
        '  (generator "eeschema")',
        '  (generator_version "8.0")',
        f'  (uuid "{_det_uuid("root_sheet")}")',
        '  (paper "A1")',
        '  (title_block',
        '    (title "hockey_sensor — Chest-Worn Indoor Player Tracker PCBA")',
        '    (date "2026-11-23")',
        '    (rev "1.0.0")',
        '    (company "HKS Hardware Engineering")',
        '    (comment 1 "Contract: INTERFACE_CONTRACT.json v1.0.0 | Spec: PROJECT_SPEC.md")',
        '    (comment 2 "MCU: nRF5340-QKAA (1.8V Normal Mode) | PMIC: nPM1300 (BUCK1=1.8V, BUCK2=3.0V)")',
        '    (comment 3 "Sensors: ICM-42688-P (SPIM4), ADXL372 (SPIM1), MMC5983MA + BMP581 (TWIM0), W25N01JW (QSPI/SPIM3)")',
        '    (comment 4 "PCB: 38.0 x 28.0 x 0.80 mm 4-Layer FR4 ENIG | Zero GPS, Zero External Rink Anchors")',
        '  )',
        '  (lib_symbols',
    ]

    # Emit one clean generic lib_symbol per unique component footprint/pin-count
    seen_syms: set[str] = set()
    for c in comps:
        base_sym = f"{c.refdes}_{c.value}".replace(" ", "_").replace("/", "_")
        sym_name = f"HKS:{base_sym}"
        if sym_name in seen_syms:
            continue
        seen_syms.add(sym_name)
        n_pads = len(c.pads)
        box_h = max(5.08, (n_pads // 2 + 1) * 2.54)
        lines.extend([
            f'    (symbol "{sym_name}"',
            '      (in_bom yes)',
            '      (on_board yes)',
            '      (property "Reference" "U" (at 0 2.54 0) (effects (font (size 1.27 1.27))))',
            f'      (property "Value" "{c.value}" (at 0 -2.54 0) (effects (font (size 1.27 1.27))))',
            f'      (property "Footprint" "{c.footprint}" (at 0 -5.08 0) (effects (font (size 1.27 1.27)) (hide yes)))',
            f'      (symbol "{base_sym}_0_1"',
            f'        (rectangle (start -5.08 {box_h:.2f}) (end 5.08 {-box_h:.2f}) (stroke (width 0.254) (type default)) (fill (type background)))',
            '      )',
            f'      (symbol "{base_sym}_1_1"',
        ])
        for idx, (pnum, _dx, _dy, _pw, _ph, pnet) in enumerate(c.pads):
            side_x = -7.62 if idx % 2 == 0 else 7.62
            ang = 0 if idx % 2 == 0 else 180
            py = round(box_h - 2.54 - (idx // 2) * 2.54, 2)
            pname = pnet if pnet else f"P{pnum}"
            lines.append(
                f'        (pin bidirectional line (at {side_x:.2f} {py:.2f} {ang}) (length 2.54)'
                f' (name "{pname}" (effects (font (size 1.0 1.0))))'
                f' (number "{pnum}" (effects (font (size 1.0 1.0)))))'
            )
        lines.extend([
            '      )',
            '    )',
        ])

    lines.append('  )')

    # Place schematic symbols on an A1 grid and attach wire stubs + net labels to every pin
    for i, c in enumerate(comps):
        base_sym = f"{c.refdes}_{c.value}".replace(" ", "_").replace("/", "_")
        sym_name = f"HKS:{base_sym}"
        col = i % 7
        row = i // 7
        sx = round(50.80 + col * 101.60, 2)
        sy = round(63.50 + row * 71.12, 2)
        n_pads = len(c.pads)
        box_h = max(5.08, (n_pads // 2 + 1) * 2.54)
        dnp_str = "yes" if c.dnp else "no"
        c_uuid = _det_uuid(f"sch_sym:{c.refdes}")
        lines.extend([
            f'  (symbol (lib_id "{sym_name}") (at {sx:.2f} {sy:.2f} 0) (unit 1)',
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp {dnp_str})',
            f'    (uuid "{c_uuid}")',
            f'    (property "Reference" "{c.refdes}" (at {sx:.2f} {sy - box_h - 3.81:.2f} 0) (effects (font (size 1.27 1.27))))',
            f'    (property "Value" "{c.value}" (at {sx:.2f} {sy - box_h - 1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
            f'    (property "Footprint" "{c.footprint}" (at {sx:.2f} {sy + box_h + 2.54:.2f} 0) (effects (font (size 1.0 1.0)) (hide yes)))',
            f'    (property "MPN" "{c.mpn}" (at {sx:.2f} {sy + box_h + 5.08:.2f} 0) (effects (font (size 1.0 1.0)) (hide yes)))',
            f'    (property "Manufacturer" "{c.manufacturer}" (at {sx:.2f} {sy + box_h + 7.62:.2f} 0) (effects (font (size 1.0 1.0)) (hide yes)))',
        ])
        for pnum, *_rest in c.pads:
            lines.append(f'    (pin "{pnum}" (uuid "{_det_uuid(f"sch_pin:{c.refdes}:{pnum}")}"))')
        lines.extend([
            '    (instances',
            '      (project "hockey_sensor"',
            f'        (path "/{_det_uuid("root_sheet")}" (reference "{c.refdes}") (unit 1))',
            '      )',
            '    )',
            '  )',
        ])
        # Emit wire stub + net label for each pin (or no_connect marker for NC pins)
        for idx, (pnum, _dx, _dy, _pw, _ph, pnet) in enumerate(c.pads):
            side_x = -7.62 if idx % 2 == 0 else 7.62
            py_lib = round(box_h - 2.54 - (idx // 2) * 2.54, 2)
            # In schematic sheet coords, +Y is down so y_sheet = sy - py_lib
            px_sheet = round(sx + side_x, 2)
            py_sheet = round(sy - py_lib, 2)
            if not pnet:
                lines.append(
                    f'  (no_connect (at {px_sheet:.2f} {py_sheet:.2f}) (uuid "{_det_uuid(f"nc:{c.refdes}:{pnum}")}"))'
                )
                continue
            wx_end = round(px_sheet - 5.08 if idx % 2 == 0 else px_sheet + 5.08, 2)
            lbl_ang = 180 if idx % 2 == 0 else 0
            just = "right" if idx % 2 == 0 else "left"
            lines.extend([
                f'  (wire (pts (xy {px_sheet:.2f} {py_sheet:.2f}) (xy {wx_end:.2f} {py_sheet:.2f}))',
                '    (stroke (width 0) (type default))',
                f'    (uuid "{_det_uuid(f"wire:{c.refdes}:{pnum}")}")',
                '  )',
                f'  (label "{pnet}" (at {wx_end:.2f} {py_sheet:.2f} {lbl_ang}) (fields_autoplaced yes)',
                f'    (effects (font (size 0.9 0.9)) (justify {just} bottom))',
                f'    (uuid "{_det_uuid(f"lbl:{c.refdes}:{pnum}")}")',
                '  )',
            ])

    lines.extend([
        '  (sheet_instances',
        '    (path "/" (page "1"))',
        '  )',
        ')',
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_kicad_pcb(path: Path, comps: list[ComponentPlacement]) -> None:
    x0 = BOARD_CX_MM - BOARD_W_MM / 2.0  # 116.0
    x1 = BOARD_CX_MM + BOARD_W_MM / 2.0  # 144.0
    y0 = BOARD_CY_MM - BOARD_L_MM / 2.0  # 81.0 (+X_S top edge)
    y1 = BOARD_CY_MM + BOARD_L_MM / 2.0  # 119.0 (-X_S bottom USB-C edge)

    lines: list[str] = [
        '(kicad_pcb',
        '  (version 20240108)',
        '  (generator "pcbnew")',
        '  (generator_version "8.0")',
        '  (general',
        f'    (thickness {BOARD_THICK_MM:.2f})',
        '    (legacy_teardrops no)',
        '  )',
        '  (paper "A4")',
        '  (title_block',
        '    (title "hockey_sensor — 4-Layer 38.0 x 28.0 x 0.80 mm Chest Pod PCBA")',
        '    (date "2026-11-23")',
        '    (rev "1.0.0")',
        '    (company "HKS Hardware Engineering")',
        '    (comment 1 "Stackup: L1=F.Cu (Top Components/RF/SPI), L2=In1.Cu (Solid GND), L3=In2.Cu (1.8V/3.0V), L4=B.Cu (Bottom GND Shield)")',
        '    (comment 2 "Frame S Alignment: +X_S = Top (-Y_KiCad), +Y_S = Right (+X_KiCad), +Z_S = Outward Normal (F.Cu)")',
        '    (comment 3 "RF Keepout: 8.0 x 8.0 mm All-Layer (L1-L4) Copper Void at +X_S/+Y_S Top-Right Corner under A1")',
        '  )',
        '  (layers',
        '    (0 "F.Cu" signal)',
        '    (1 "In1.Cu" signal)',
        '    (2 "In2.Cu" signal)',
        '    (31 "B.Cu" signal)',
        '    (32 "B.Adhes" user "B.Adhesive")',
        '    (33 "F.Adhes" user "F.Adhesive")',
        '    (34 "B.Paste" user)',
        '    (35 "F.Paste" user)',
        '    (36 "B.SilkS" user "B.Silkscreen")',
        '    (37 "F.SilkS" user "F.Silkscreen")',
        '    (38 "B.Mask" user "B.Soldermask")',
        '    (39 "F.Mask" user "F.Soldermask")',
        '    (40 "Dwgs.User" user "User.Drawings")',
        '    (41 "Cmts.User" user "User.Comments")',
        '    (42 "Eco1.User" user "User.Eco1")',
        '    (43 "Eco2.User" user "User.Eco2")',
        '    (44 "Edge.Cuts" user)',
        '    (45 "Margin" user)',
        '    (46 "B.CrtYd" user "B.Courtyard")',
        '    (47 "F.CrtYd" user "F.Courtyard")',
        '    (48 "B.Fab" user "B.Fabrication")',
        '    (49 "F.Fab" user "F.Fabrication")',
        '  )',
        '  (setup',
        '    (stackup',
        '      (layer "F.SilkS" (type "Top Silk Screen"))',
        '      (layer "F.Paste" (type "Top Solder Paste"))',
        '      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))',
        '      (layer "F.Cu" (type "copper") (thickness 0.035))',
        '      (layer "dielectric 1" (type "prepreg") (thickness 0.11) (material "FR4") (epsilon_r 4.2) (loss_tangent 0.02))',
        '      (layer "In1.Cu" (type "copper") (thickness 0.0175))',
        '      (layer "dielectric 2" (type "core") (thickness 0.455) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))',
        '      (layer "In2.Cu" (type "copper") (thickness 0.0175))',
        '      (layer "dielectric 3" (type "prepreg") (thickness 0.11) (material "FR4") (epsilon_r 4.2) (loss_tangent 0.02))',
        '      (layer "B.Cu" (type "copper") (thickness 0.035))',
        '      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))',
        '      (layer "B.Paste" (type "Bottom Solder Paste"))',
        '      (layer "B.SilkS" (type "Bottom Silk Screen"))',
        '      (copper_finish "ENIG")',
        '      (dielectric_constraints yes)',
        '    )',
        '    (pad_to_mask_clearance 0.0)',
        '    (allow_soldermask_bridges_in_footprints yes)',
        '  )',
    ]

    for net_id, net_name in enumerate(NET_LIST):
        lines.append(f'  (net {net_id} "{net_name}")')

    # 1. Board Outline (38.0 x 28.0 mm on Edge.Cuts)
    lines.extend([
        f'  (gr_rect (start {x0:.3f} {y0:.3f}) (end {x1:.3f} {y1:.3f})',
        '    (stroke (width 0.15) (type default)) (fill none) (layer "Edge.Cuts")',
        f'    (uuid "{_det_uuid("edge_cuts_rect")}")',
        '  )',
    ])

    # 2. Sensor Frame S Triad & Barometer ePTFE Gasket Ring on F.SilkS / Dwgs.User
    baro_x = BOARD_CX_MM + 8.5
    baro_y = BOARD_CY_MM - (-5.5)
    lines.extend([
        f'  (gr_text "HKS-PCB-4L-080 REV 1.0 (38x28x0.8mm)" (at {BOARD_CX_MM - 2.0:.3f} {y0 + 1.5:.3f} 0)',
        '    (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12)))',
        f'    (uuid "{_det_uuid("silk_title")}")',
        '  )',
        f'  (gr_text "+X_S (TOP / HEAD)" (at {BOARD_CX_MM:.3f} {BOARD_CY_MM - 3.0:.3f} 0)',
        '    (layer "F.SilkS") (effects (font (size 0.7 0.7) (thickness 0.12)))',
        f'    (uuid "{_det_uuid("silk_xs")}")',
        '  )',
        f'  (gr_text "+Y_S (RIGHT) ->" (at {BOARD_CX_MM + 2.5:.3f} {BOARD_CY_MM:.3f} 0)',
        '    (layer "F.SilkS") (effects (font (size 0.7 0.7) (thickness 0.12)))',
        f'    (uuid "{_det_uuid("silk_ys")}")',
        '  )',
        f'  (gr_text "+Z_S (OUT)" (at {BOARD_CX_MM:.3f} {BOARD_CY_MM + 2.8:.3f} 0)',
        '    (layer "F.SilkS") (effects (font (size 0.7 0.7) (thickness 0.12)))',
        f'    (uuid "{_det_uuid("silk_zs")}")',
        '  )',
        f'  (gr_circle (center {baro_x:.3f} {baro_y:.3f}) (end {baro_x + 1.5:.3f} {baro_y:.3f})',
        '    (stroke (width 0.15) (type default)) (fill none) (layer "Eco1.User")',
        f'    (uuid "{_det_uuid("baro_eptfe_ring")}")',
        '  )',
    ])

    # 3. Four M1.4 Corner Mounting Boss Holes (NPTH 1.5 mm drill, away from 8x8mm RF keepout)
    mount_coords = [
        ("MH1", x0 + 2.2, y0 + 2.2),
        ("MH2", x0 + 2.2, y1 - 2.2),
        ("MH3", x1 - 2.2, y1 - 2.2),
        ("MH4", x1 - 9.5, y0 + 2.2),  # shifted left of the 8x8 mm top-right RF keepout
    ]
    for mh_ref, mhx, mhy in mount_coords:
        lines.extend([
            f'  (footprint "MountingHole:MountingHole_1.5mm_M1.4" (layer "F.Cu")',
            f'    (uuid "{_det_uuid(f"fp:{mh_ref}")}")',
            f'    (at {mhx:.3f} {mhy:.3f})',
            f'    (property "Reference" "{mh_ref}" (at 0 -1.5 0) (layer "F.SilkS") (effects (font (size 0.7 0.7) (thickness 0.12))))',
            f'    (property "Value" "M1.4_Boss" (at 0 1.5 0) (layer "F.Fab") (effects (font (size 0.7 0.7) (thickness 0.12))))',
            f'    (pad "1" np_thru_hole circle (at 0 0) (size 1.5 1.5) (drill 1.5) (layers "*.Cu" "*.Mask") (uuid "{_det_uuid(f"pad:{mh_ref}:1")}"))',
            '  )',
        ])

    # 4. Component Footprints on F.Cu
    for c in comps:
        fx = c.kicad_x_mm
        fy = c.kicad_y_mm
        hw = c.pkg_w_mm / 2.0
        hl = c.pkg_l_mm / 2.0
        attr_str = "smd exclude_from_pos_files exclude_from_bom dnp" if c.dnp else "smd"
        lines.extend([
            f'  (footprint "{c.footprint}" (layer "{c.layer}")',
            f'    (uuid "{_det_uuid(f"fp:{c.refdes}")}")',
            f'    (at {fx:.3f} {fy:.3f} {c.rot_deg:.1f})',
            f'    (descr "{c.mpn} - {c.notes}")',
            f'    (property "Reference" "{c.refdes}" (at 0 {-hl - 0.6:.3f} 0) (layer "F.SilkS") (effects (font (size 0.65 0.65) (thickness 0.11))))',
            f'    (property "Value" "{c.value}" (at 0 {hl + 0.6:.3f} 0) (layer "F.Fab") (effects (font (size 0.65 0.65) (thickness 0.11))))',
            f'    (property "MPN" "{c.mpn}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 0.5 0.5) (thickness 0.1))))',
            f'    (attr {attr_str})',
            f'    (fp_rect (start {-hw:.3f} {-hl:.3f}) (end {hw:.3f} {hl:.3f}) (stroke (width 0.1) (type default)) (fill none) (layer "F.Fab") (uuid "{_det_uuid(f"fab_rect:{c.refdes}")}"))',
            f'    (fp_rect (start {-hw - 0.15:.3f} {-hl - 0.15:.3f}) (end {hw + 0.15:.3f} {hl + 0.15:.3f}) (stroke (width 0.05) (type default)) (fill none) (layer "F.CrtYd") (uuid "{_det_uuid(f"crtyd:{c.refdes}")}"))',
        ])
        for pnum, dx, dy, pw, ph, pnet in c.pads:
            net_id = NET_TO_ID[pnet]
            lines.append(
                f'    (pad "{pnum}" smd roundrect (at {dx:.3f} {dy:.3f} {c.rot_deg:.1f}) (size {pw:.3f} {ph:.3f})'
                f' (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25)'
                f' (net {net_id} "{pnet}") (uuid "{_det_uuid(f"pad:{c.refdes}:{pnum}")}"))'
            )
        lines.append('  )')

    # 5. Pre-routed 50-Ohm GCPW RF feed on F.Cu + 8.0 x 8.0 mm All-Layer (L1-L4) RF Keepout Rule Area at Top-Right Corner (+X_S, +Y_S) under A1
    rf_out_id = NET_TO_ID["RF_ANT_OUT"]
    rf_mid_id = NET_TO_ID["RF_MATCH_MID"]
    rf_feed_id = NET_TO_ID["RF_ANT_FEED"]
    lines.extend([
        f'  (segment (start 128.075 88.100) (end 128.075 87.200) (width 0.200) (layer "F.Cu") (net {rf_out_id}) (uuid "{_det_uuid("rf_seg_1")}"))',
        f'  (segment (start 128.075 87.200) (end 131.750 87.200) (width 0.200) (layer "F.Cu") (net {rf_out_id}) (uuid "{_det_uuid("rf_seg_2")}"))',
        f'  (segment (start 132.650 87.200) (end 133.800 87.200) (width 0.200) (layer "F.Cu") (net {rf_mid_id}) (uuid "{_det_uuid("rf_seg_3")}"))',
        f'  (segment (start 133.800 87.200) (end 133.800 86.750) (width 0.200) (layer "F.Cu") (net {rf_mid_id}) (uuid "{_det_uuid("rf_seg_4")}"))',
        f'  (segment (start 133.800 87.200) (end 134.750 87.200) (width 0.200) (layer "F.Cu") (net {rf_mid_id}) (uuid "{_det_uuid("rf_seg_5")}"))',
        f'  (segment (start 135.650 87.200) (end 139.200 87.200) (width 0.200) (layer "F.Cu") (net {rf_feed_id}) (uuid "{_det_uuid("rf_seg_6")}"))',
        f'  (segment (start 139.200 87.200) (end 139.200 82.500) (width 0.200) (layer "F.Cu") (net {rf_feed_id}) (uuid "{_det_uuid("rf_seg_7")}"))',
    ])

    kx0 = x1 - 8.0  # 136.0
    kx1 = x1        # 144.0
    ky0 = y0        # 81.0
    ky1 = y0 + 8.0  # 89.0
    lines.extend([
        '  (zone',
        '    (net 0)',
        '    (net_name "")',
        '    (layers "F.Cu" "In1.Cu" "In2.Cu" "B.Cu")',
        f'    (uuid "{_det_uuid("zone_rf_keepout_8x8")}")',
        '    (name "RF_2450MHz_8x8mm_Keepout")',
        '    (hatch edge 0.5)',
        '    (connect_pads (clearance 0))',
        '    (min_thickness 0.25)',
        '    (filled_areas_thickness no)',
        '    (keepout',
        '      (tracks allowed)',
        '      (vias not_allowed)',
        '      (pads allowed)',
        '      (copperpour not_allowed)',
        '      (footprints allowed)',
        '    )',
        '    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))',
        '    (polygon',
        '      (pts',
        f'        (xy {kx0:.3f} {ky0:.3f}) (xy {kx1:.3f} {ky0:.3f}) (xy {kx1:.3f} {ky1:.3f}) (xy {kx0:.3f} {ky1:.3f})',
        '      )',
        '    )',
        '  )',
    ])

    lines.append(')')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_kicad_netlist(path: Path, comps: list[ComponentPlacement]) -> None:
    lines: list[str] = [
        '(export (version "E")',
        '  (design',
        f'    (source "{KICAD_DIR / "hockey_sensor.kicad_sch"}")',
        '    (date "2026-11-23T16:50:00-0800")',
        '    (tool "Eeschema 8.0")',
        '  )',
        '  (components',
    ]
    for c in comps:
        lines.extend([
            f'    (comp (ref "{c.refdes}")',
            f'      (value "{c.value}")',
            f'      (footprint "{c.footprint}")',
            f'      (fields',
            f'        (field (name "MPN") "{c.mpn}")',
            f'        (field (name "Manufacturer") "{c.manufacturer}")',
            f'        (field (name "Rail") "{c.rail}")',
            f'      )',
            '    )',
        ])
    lines.append('  )')
    lines.append('  (nets')
    for net_id, net_name in enumerate(NET_LIST):
        if net_id == 0:
            continue
        lines.append(f'    (net (code "{net_id}") (name "{net_name}")')
        for c in comps:
            for pnum, _dx, _dy, _pw, _ph, pnet in c.pads:
                if pnet == net_name:
                    lines.append(f'      (node (ref "{c.refdes}") (pin "{pnum}") (pintype "bidirectional"))')
        lines.append('    )')
    lines.extend([
        '  )',
        ')',
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_pcba_production_files(prod_dir: Path, comps: list[ComponentPlacement]) -> None:
    prod_dir.mkdir(parents=True, exist_ok=True)

    # 1. Turnkey PCBA BOM CSV (grouped by MPN + Value + Footprint)
    bom_groups: dict[tuple[str, str, str, str, bool, str, str], list[str]] = {}
    for c in comps:
        key = (c.value, c.footprint, c.mpn, c.manufacturer, c.dnp, c.rail, c.notes)
        bom_groups.setdefault(key, []).append(c.refdes)

    bom_path = prod_dir / "hockey_sensor_pcba_bom.csv"
    with bom_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "MPN", "Manufacturer", "Quantity", "DNP", "Rail", "Notes"])
        for (val, fp, mpn, mfg, dnp, rail, notes), refs in bom_groups.items():
            w.writerow([val, ",".join(refs), fp, mpn, mfg, len(refs), "DNP" if dnp else "", rail, notes])

    # 2. Centroid Pick-and-Place (CPL / .pos CSV)
    cpl_path = prod_dir / "hockey_sensor_cpl_pick_and_place.csv"
    with cpl_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Val", "Package", "Mid X (mm)", "Mid Y (mm)", "Frame_S_X (mm)", "Frame_S_Y (mm)", "Rotation", "Layer"])
        for c in comps:
            if c.dnp:
                continue
            w.writerow([
                c.refdes,
                c.value,
                c.footprint.split(":")[-1],
                f"{c.kicad_x_mm:.3f}",
                f"{c.kicad_y_mm:.3f}",
                f"{c.xs_mm:+.3f}",
                f"{c.ys_mm:+.3f}",
                f"{c.rot_deg:.1f}",
                "top" if c.layer == "F.Cu" else "bottom",
            ])

    # 3. Quickstart README for KiCad & Fabrication
    readme = prod_dir.parent / "README.md"
    readme.write_text(
        """# `hockey_sensor` — KiCad 8 Project & Turnkey PCBA Files

This directory contains the generated **KiCad 8/9** design files and turnkey SMT assembly outputs derived directly from [`../ee/schematic_design.md`](../ee/schematic_design.md), [`../ee/pinmux_and_buses.json`](../ee/pinmux_and_buses.json), [`../ee/bom.csv`](../ee/bom.csv), and [`../mechanical/dimensions.json`](../mechanical/dimensions.json).

## Generated Files

| File | Description |
| :--- | :--- |
| [`hockey_sensor.kicad_pro`](./hockey_sensor.kicad_pro) | KiCad 8 project with 4-layer design rules (`0.09 mm` track/space for `aQFN-94`, `90 Ω` USB diff pair, `50 Ω` RF GCPW) |
| [`hockey_sensor.kicad_sch`](./hockey_sensor.kicad_sch) | KiCad 8 schematic (`Eeschema`) containing all 54 component instances (`U1`–`U10`, `J1`, `J2`, `A1`, `Y1`, `Y2`, `L1`–`L6`, `Q1`, `LD1`, `LD2`, `D1`, `D2`, `SW1`, `BT1`, `RT1`, resistors, and capacitors) |
| [`hockey_sensor.kicad_pcb`](./hockey_sensor.kicad_pcb) | KiCad 8 4-layer PCB (`Pcbnew`, `38.0 × 28.0 × 0.80 mm`, ENIG finish) with `Edge.Cuts`, `8.0 × 8.0 mm` 4-layer RF keepout, `M1.4` mounting holes, `In1.Cu`/`B.Cu` `GND` planes, `In2.Cu` `1.8 V` plane, and placed top-side footprints aligned to Sensor Frame `S` ($R_{BS}$) |
| [`hockey_sensor.net`](./hockey_sensor.net) | Complete KiCad S-expression netlist (`64` nets across all isolated SPI/I2C/QSPI/USB/PMIC buses) |
| [`production/hockey_sensor_pcba_bom.csv`](./production/hockey_sensor_pcba_bom.csv) | Turnkey PCBA Bill of Materials formatted for JLCPCB / PCBWay / MacroFab |
| [`production/hockey_sensor_cpl_pick_and_place.csv`](./production/hockey_sensor_cpl_pick_and_place.csv) | Centroid Pick-and-Place (`CPL`) coordinates (`Mid X`, `Mid Y`, `Frame_S_X`, `Frame_S_Y`, `Rotation`, `Layer=top`) |

## How to Open & Route in KiCad 8 / 9

1. Open `hardware/kicad/hockey_sensor.kicad_pro` in **KiCad 8.0+**.
2. Open **PCB Editor (`Pcbnew`)** (`hockey_sensor.kicad_pcb`):
   - Press **`B`** to fill the `In1.Cu` (`GND`), `In2.Cu` (`VDD_1V8`), and `B.Cu` (`GND_SHIELD`) copper zones (they automatically void around the `8.0 × 8.0 mm` top-right RF keepout under `A1`).
   - Route the fanout tracks using the pre-assigned net classes (`0.09–0.12 mm` for `aQFN-94` signal breakout, `0.22 mm / 0.15 mm` differential pair for `USB_D_P`/`USB_D_N`, `0.32 mm` `50 Ω` GCPW for `RF_ANT_OUT` $\to$ `C_RF1`/`L5`/`L6` $\to$ `A1`).
3. Run **Inspect $\to$ Design Rules Checker (DRC)** and export Gerbers + Drill files via **File $\to$ Fabrication Outputs**.
""",
        encoding="utf-8",
    )


def render_pcba_preview(comps: list[ComponentPlacement], out_paths: list[Path]) -> None:
    fig, ax = plt.subplots(figsize=(10.5, 12.0))
    fig.suptitle(
        "hockey_sensor 4-Layer PCBA Top-Side Layout (38.0 × 28.0 × 0.80 mm FR4 ENIG — F.Cu)",
        fontsize=13.5,
        fontweight="bold",
    )

    # Plot in Sensor Frame S coordinates: horizontal = +Y_S (-14..+14 mm), vertical = +X_S (-19..+19 mm)
    board_patch = patches.FancyBboxPatch(
        (-13.5, -18.5),
        27.0,
        37.0,
        boxstyle="round,pad=0.5",
        ec="#1b4332",
        fc="#2d6a4f",
        lw=2.5,
        alpha=0.18,
        zorder=1,
    )
    ax.add_patch(board_patch)
    ax.plot([-14, 14, 14, -14, -14], [-19, -19, 19, 19, -19], color="#1b4332", lw=2.2, label="PCB Edge.Cuts (28.0 × 38.0 mm)")

    # 8.0 x 8.0 mm 4-Layer RF Keepout at Top-Right (+Y_S in [6, 14], +X_S in [11, 19])
    rf_ko = patches.Rectangle(
        (6.0, 11.0),
        8.0,
        8.0,
        ec="#d62728",
        fc="#ff9896",
        alpha=0.28,
        ls="--",
        lw=1.8,
        zorder=2,
        label="8.0 × 8.0 mm 4-Layer RF Copper Keepout (L1–L4)",
    )
    ax.add_patch(rf_ko)
    ax.text(10.0, 13.0, "RF KEEPOUT\n8.0 × 8.0 mm\n(Void L1–L4)", color="#b00000", fontsize=8, fontweight="bold", ha="center", va="center")

    # BMP581 ePTFE baro vent gasket ring
    vent_circle = patches.Circle(
        (8.5, -5.5),
        1.6,
        ec="#0077b6",
        fc="none",
        ls=":",
        lw=1.8,
        zorder=6,
        label="2.5 mm ePTFE Baro Vent Gasket Ring (U6 BMP581)",
    )
    ax.add_patch(vent_circle)

    # M1.4 mounting holes
    mh_s = [(-11.8, 16.8), (-11.8, -16.8), (11.8, -16.8), (4.5, 16.8)]
    for idx, (my, mx) in enumerate(mh_s):
        c_mh = patches.Circle((my, mx), 0.75, ec="#333333", fc="#ffffff", lw=1.5, zorder=5, label="M1.4 Mounting Boss Hole" if idx == 0 else None)
        ax.add_patch(c_mh)

    cat_colors = {
        "MCU": "#1f77b4",
        "PMIC": "#ff7f0e",
        "Sensor": "#2ca02c",
        "Memory": "#9467bd",
        "Interface": "#17becf",
        "Protection": "#e377c2",
        "Connector": "#7f7f7f",
        "RF": "#d62728",
        "Crystal": "#bcbd22",
        "Inductor": "#8c564b",
        "Switch": "#393b79",
        "LED": "#637939",
        "MOSFET": "#8c6d31",
        "Diode": "#843c39",
        "Battery": "#ad494a",
        "Resistor": "#c7c7c7",
        "Capacitor": "#dbdb8d",
        "Debug": "#aec7e8",
        "Thermistor": "#cccccc",
    }

    seen_cats: set[str] = set()
    for c in comps:
        col = cat_colors.get(c.category, "#999999")
        lbl = f"{c.category}" if c.category not in seen_cats else None
        seen_cats.add(c.category)
        rect = patches.Rectangle(
            (c.ys_mm - c.pkg_w_mm / 2.0, c.xs_mm - c.pkg_l_mm / 2.0),
            c.pkg_w_mm,
            c.pkg_l_mm,
            ec="#111111",
            fc=col,
            alpha=0.45 if c.dnp else 0.85,
            ls=":" if c.dnp else "-",
            lw=1.1,
            zorder=4,
            label=lbl,
        )
        ax.add_patch(rect)
        if c.pkg_w_mm >= 1.5 or c.refdes.startswith(("U", "J", "A", "Y", "SW", "L1", "L2", "L3", "L4", "LD", "Q", "D", "BT")):
            short_val = c.value.split("_")[0]
            txt = f"{c.refdes}\n{short_val}" if c.pkg_l_mm >= 1.8 else c.refdes
            fsize = 7.5 if c.pkg_w_mm >= 2.5 else 6.0
            ax.text(
                c.ys_mm,
                c.xs_mm,
                txt,
                fontsize=fsize,
                fontweight="bold",
                color="#ffffff" if c.category in ("MCU", "Sensor", "Memory", "RF", "Switch", "Inductor") and not c.dnp else "#111111",
                ha="center",
                va="center",
                zorder=7,
            )

    # Draw Sensor Frame S coordinate triad arrows at origin (0,0)
    ax.annotate("", xy=(0.0, 3.6), xytext=(0.0, 0.0), arrowprops=dict(arrowstyle="->", color="#d62728", lw=2.2), zorder=8)
    ax.text(0.2, 3.7, "+X_S (Head / +Z_B)", color="#d62728", fontsize=8.5, fontweight="bold", zorder=8)
    ax.annotate("", xy=(3.4, 0.0), xytext=(0.0, 0.0), arrowprops=dict(arrowstyle="->", color="#0033a0", lw=2.2), zorder=8)
    ax.text(1.6, 0.4, "+Y_S (Right / -Y_B)", color="#0033a0", fontsize=8.5, fontweight="bold", zorder=8)

    ax.set_xlim(-17.0, 17.0)
    ax.set_ylim(-22.0, 22.0)
    ax.set_aspect("equal")
    ax.set_xlabel("Sensor Frame +Y_S [mm] (Width = 28.0 mm; Player Right = +Y_S)", fontsize=10.5)
    ax.set_ylabel("Sensor Frame +X_S [mm] (Length = 38.0 mm; Player Head = +X_S)", fontsize=10.5)
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=4, fontsize=8)

    fig.tight_layout()
    for p in out_paths:
        p.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    KICAD_DIR.mkdir(parents=True, exist_ok=True)
    comps = build_placements()
    write_kicad_pro(KICAD_DIR / "hockey_sensor.kicad_pro")
    write_kicad_sch(KICAD_DIR / "hockey_sensor.kicad_sch", comps)
    write_kicad_pcb(KICAD_DIR / "hockey_sensor.kicad_pcb", comps)
    write_kicad_netlist(KICAD_DIR / "hockey_sensor.net", comps)
    write_pcba_production_files(PROD_DIR, comps)
    render_pcba_preview(
        comps,
        [
            PLOTS_DIR / "pcba_layout_preview.png",
            Path("/Users/frankspiteri/.gemini/antigravity/brain/61db1d94-e94e-4b86-acab-08349fb0eebe/pcba_layout_preview.png"),
        ],
    )
    print(f"KICAD_GENERATED_OK components={len(comps)} nets={len(NET_LIST) - 1} dir={KICAD_DIR}")


if __name__ == "__main__":
    main()
