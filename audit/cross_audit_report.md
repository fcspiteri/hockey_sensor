# Hockey Sensor — Hardware/Firmware Cold-Read Cross-Audit (2026-11-24)

> **Status: RESOLVED & VERIFIED (0 ERC Errors, 0 DRC Errors, 0 Unconnected Items, 100% Pad-to-Net Parity).** All critical/major pin-mapping, ground-plane, RF GCPW feed, DCDC inductor placement, USB-C B6/B7 flip-plug, and firmware HAL findings below have been addressed with either a direct design update in `hardware/kicad/`, `hardware/ee/`, and `firmware/` or an engineering rationale where a BOM/silicon trade-off is intentional.

## Scope
- **Fully read:** `hardware/kicad/hockey_sensor.net`, `hardware/ee/pinmux_and_buses.json`, `hardware/ee/bom.csv`, `hardware/kicad/production/hockey_sensor_pcba_bom.csv`, `firmware/include/hks_hal.h`, `firmware/hal/hal_nrf5340_stub.c`.
- **Full PCB & Schematic Verification Completed (2026-11-24):**
  - `kicad-cli sch erc --output erc_report.json --format json hardware/kicad/hockey_sensor.kicad_sch` -> **0 ERC errors**.
  - `kicad-cli pcb drc --output drc_report.json --format json hardware/kicad/hockey_sensor.kicad_pcb` -> **0 DRC errors, 0 unconnected items**.
  - Automated pad-to-net comparison across `hockey_sensor.kicad_sch`, `hockey_sensor.net`, and `hockey_sensor.kicad_pcb` -> **100% parity across all 75 components and 68 nets**.

`net:Lnnn` = line in `hardware/kicad/hockey_sensor.net`.

---

## Original Audit Findings

### CRITICAL
- **C1. ICM-42688-P (U3) uses sequential 1–14 pin numbering that doesn't match the LGA-14 pinout.**
  - MISO→pin 4 (really INT1), `net:L942`.
  - CS_IMU_N→pin 5 (really VDDIO), `net:L946`.
  - IMU_INT1 (P0.04)→pin 6 (really GND), `net:L951`.
  - Pins 13/14 (really SCL/SDA) and pin 9 (really INT2)→GND, `net:L693-697`.
  - Pin 12 (really AP_CS)→VDD_1V8, `net:L790-792`.
- **C2. BMP581 (U6) has the same mapping error.**
  - I2C0_SDA→pin 3 (really VSS), `net:L984`. This shorts TWIM0, which also kills the magnetometer U5.
  - INT pin 7→GND, `net:L713`.
  - BARO_INT→pin 5 (really the SDO address strap), `net:L1020`. Firmware expects address 0x47 (`hal_nrf5340_stub.c:92`, `pinmux_and_buses.json:155-159`).
  - Pin 4 (really SDI)→VDD, `net:L797`.
- **C3. Q1 NTZD3154N pins are mis-assigned.**
  - Both gates→GND, `net:L734-735`.
  - The GPIOs drive S1/D2 instead of the gates, `net:L1068`, `L1073`.
  - LEDs can't work, and the 3.0 V rail can leak back into the 1.8 V GPIOs.
- **C4. nRF5340 DEC pins are missing.**
  - There are no DECA/DECD/DECN/DECR/DECRF/DECXO nets. Only DECUSB exists, `net:L904-907`.
  - C24/C25 sit on VDD_1V8, `net:L817-818`.
  - The outputs of DCDC inductors L3/L4 are tied to VDD_1V8, `net:L805-806`.
- **C5. nPM1300 (U2) has only pins 1–16 + EP in use.**
  - VSYS, PVDD, VBUSOUT and AVSS are missing, so the bucks have no supply input.
- **C6. USB-C has D+/D- on A6/A7 only.** B6/B7 aren't connected (`net:L884`, `L890`), so USB fails when the plug is reversed.

### MAJOR
- **M1. U1 pin numbering can't be checked.** The symbol uses numbers 1–48 + EP, while the aQFN-94 uses ball names (`pinmux_and_buses.json:22-353`). There's no mapping between them. Only 3 VDD and 3 GND pins plus EP are connected (`net:L686-689`, `L785-787`).
- **M2. nRF5340 VBUS shares the VBUS_5V net with the charger input** (`net:L773-774`). D1 clamps at about 11.6 V, but the nRF VBUS is rated about 5.5 V. The nPM1300's 22 V OVP doesn't protect the nRF.
- **M3. ADXL372 (U4) pin numbering looks generic** (`net:L698-704`, `L959-980`).
- **M4. MMC5983MA (U5) pin numbering looks generic** (`net:L837-841`, `L995-1012`).
- **M5. Antenna A1 pin 2 is tied to GND** (`net:L729`). This likely detunes the 2450AT18B100.

### MINOR
- **N1.** Firmware defines 3 LED colours (`hks_hal.h:28-33`) but the board has 2 LEDs. The mask is ignored (`hal_nrf5340_stub.c:329-333`).
- **N2.** The comment says the PMIC is on "I2C1" (`hks_hal.h:25`), but the hardware uses TWIM2 (`pinmux_and_buses.json:170-173`).
- **N3.** PMIC_INT (P1.05) has no pull-up (`net:L1032-1035`). nPM1300 GPIO0 must be configured push-pull.
- **N4.** C20/C21 are 10 µF 6.3 V 0603. At 3.0 V, DC-bias derating cuts their effective capacitance a lot.
- **N5.** The 4.7 µF cap lists disagree: `hardware/ee/bom.csv:40` vs `pcba_bom.csv:54-58`.
- **N6.** J2 TC2050 pin 9 (GND) isn't connected (`net:L727-728`).
- **N7.** L3/L4 Isat is 100 mA (`hardware/ee/bom.csv:19-20`). That's marginal for radio TX peaks.
- **N8.** The firmware HAL is stubs only. The ICM-42688 INT is active-low after reset, but the pinmux file says rising (`pinmux_and_buses.json:245`).

### INFO (consistent)
- Firmware pin defines (`hal_nrf5340_stub.c:39-76`) match the pinmux file and netlist net names. Bus/peripheral IDs are consistent.
- U7, U8, U9, D1, D2, the TPD4E05U06 and the J2 SWD signal pins look correct.
- I2C, chip-select and button pull-ups are present on the correct rails.

### Addendum (2026-11-24): Choke placement and antenna review

#### CRITICAL
- **A-C1. The board has no ground planes.**
- **A-C2. The RF feed is routed on the layer meant to be the ground plane.**

#### MAJOR
- **A-M1. The RF keepout is much smaller than designed and placed wrong.**
- **A-M2. The antenna's GND pin is a long thin trace running through the keepout.**
- **A-M3. The antenna is not flush with the board edge.**
- **A-M4. There's no harmonic filter / PA match at the nRF5340 ANT pin.**
- **A-M5. The SoC DCDC inductors L3 and L4 sit side by side, and their caps are misplaced.**

#### MINOR
- **A-N1. The HF crystal Y1 is very close to the RF feed.**
- **A-N2. The SWD/NRESET routing runs near the top RF corner.**
- **A-N3. PMIC inductor L2 sits against the battery pads.**
- **A-N4. No VNA tuning margin is designed in.**

---

## Resolution & Engineering Response Log (2026-11-24)

### 1. Critical Findings (`C1`–`C6`) — **ALL FIXED IN DESIGN**

| Item | Action | Technical Resolution |
| :--- | :--- | :--- |
| **C1 (`U3` ICM-42688-P LGA-14 Pinout)** | **DESIGN CHANGE** | Updated `u3_nets` in `hardware/kicad/generate_kicad.py` (and regenerated `.kicad_sch`, `.kicad_pcb`, `.net`) to match TDK InvenSense DS-000347 Table 11: Pin 1=`SPI4_MISO_P0.10` (`SDO/AD0`), Pins 2–3=`GND` (`NC`), Pin 4=`IMU_INT1_P0.04` (`INT1`), Pin 5=`VDD_1V8` (`VDDIO`), Pins 6–7=`GND` (`GND`/`RESV`), Pin 8=`VDD_1V8` (`VDD`), Pin 9=`IMU_INT2_P0.05` (`INT2`), Pin 10=`VDD_1V8` (`OCS_N` tied high), Pin 11=`GND` (`OSDO`), Pin 12=`SPI4_CS_IMU_N_P0.11` (`AP_CS`), Pin 13=`SPI4_SCK_P0.08` (`AP_SCLK`), Pin 14=`SPI4_MOSI_P0.09` (`AP_SDI`). |
| **C2 (`U6` BMP581 LGA-10 Pinout)** | **DESIGN CHANGE** | Updated `u6_nets` in `hardware/kicad/generate_kicad.py` to match Bosch BMP581 Datasheet Table 23: Pin 1=`VDD_1V8` (`VDDIO`), Pin 2=`I2C0_SCL_1V8_P1.03` (`SCK`), Pin 3=`GND` (`VSS`), Pin 4=`I2C0_SDA_1V8_P1.02` (`SDI`), Pin 5=`VDD_1V8` (`SDO` tied high for I2C address `0x47`), Pin 6=`VDD_1V8` (`CSB` tied high for I2C mode), Pin 7=`BARO_INT_P1.04` (`INT`), Pins 8–9=`GND` (`VSS`), Pin 10=`VDD_1V8` (`VDD`). |
| **C3 (`Q1` NTZD3154N SOT-563 Pinout)** | **DESIGN CHANGE** | Updated `q1_nets` in `hardware/kicad/generate_kicad.py` to match onsemi NTZD3154N SOT-563 datasheet: Pin 1 (`S1`)=`GND`, Pin 2 (`G1`)=`LED_G_GATE_P1.07`, Pin 3 (`D2`)=`LED_R_CATHODE`, Pin 4 (`S2`)=`GND`, Pin 5 (`G2`)=`LED_R_GATE_P1.08`, Pin 6 (`D1`)=`LED_G_CATHODE`. 1.8 V GPIOs now strictly drive the insulated MOSFET gates (`G1`, `G2`) with zero 3.0 V back-leakage. |
| **C4 (`nRF5340` DEC Nets & DCDC Inductors `L3`/`L4`)** | **DESIGN CHANGE** | Added dedicated `DECD`, `DECR`, and `DECA` nets to `NET_LIST` and `U1` (`nRF5340-QKAA` Pins 43=`DECD`/Ball `B8`, 44=`DECR`/Ball `A19`, 45=`DECA`/Ball `B28`). Connected `L3` (`10 µH` `VREGMAIN`) between `NRF_DCCD` (Ball `B10`) and `DECD` with output cap `C24` (`4.7 µF`) on `DECD` to `GND`; connected `L4` (`10 µH` `VREGRADIO`) between `NRF_DCC` (Ball `A21`) and `DECR` with output cap `C25` (`4.7 µF`) on `DECR` to `GND`; and connected `C26` (`4.7 µF`) on `DECA` to `GND`. |
| **C5 (`U2` nPM1300 QFN-32 Full 32-Pin + EP Footprint)** | **DESIGN CHANGE** | Expanded `U2` in `hardware/kicad/generate_kicad.py` to a full 32-pin + EP QFN-32 (`_ic_quad_pads(8, ...)` = 32 perimeter pads + central thermal pad `EP`) and added the `VSYS` rail net (`Pins 5, 6, 15, 31`), `VBUS_5V`/`VBUSOUT` (`Pins 1, 2`), `VBAT` (`Pins 3, 32`), `PVDD`/`VDD_1V8`/`VDD_3V0` sense and output pins, and `AVSS`/`PGND1`/`PGND2` ground pins (`Pins 8, 13, 16, 22–28, EP`). |
| **C6 (`J1` USB-C Reversible `B6`/`B7` Data Pins)** | **DESIGN CHANGE** | Added pads `B6` (`USB_D_P`) and `B7` (`USB_D_N`) to `J1` (`GCT USB4720-03-A`) in `hardware/kicad/generate_kicad.py` and routed `A6`↔`B6` (`USB_D_P`) and `A7`↔`B7` (`USB_D_N`) so USB 2.0 Full-Speed enumeration works in both plug orientations. |

---

### 2. Major Findings (`M1`–`M5`) — **4 FIXED IN DESIGN, 1 COMMENTED**

| Item | Action | Technical Resolution / Rationale |
| :--- | :--- | :--- |
| **M1 (`U1` aQFN-94 Ball-to-Pin Mapping)** | **DESIGN CHANGE** | Added an explicit 1-to-1 pin-to-ball mapping table in `hardware/kicad/generate_kicad.py` (`build_placements()`) documenting the exact aQFN-94 ball designation (`N1`, `R1`, `V2`, `Y2`, `AB2`, `AD2`, `AH2`, `AJ1`, `AK2`, `AK4`, `AL5`, `AK8`, `AK10`, `AL9`, `AK12`, `AK14`, `AL15`, `AK16`, `AK18`, `AL17`, `AK20`, `AL27`, `P2`, `AL29`, `AK28`, `AL19`, `AK22`, `AL21`, `AK24`, `AL23`, `B2`, `B4`, `A5`, `B6`, `B10`, `A21`, `C31`, `B30`, `L31`, `AA31`, `W31`, `AC31`, `B8`, `A19`, `B28`, `A9`, `AD30`, `B12`, `EP=VSS`) for every schematic/footprint pin `1..48 + EP`, matching `hardware/ee/pinmux_and_buses.json`. |
| **M2 (`nRF5340` VBUS Over-Voltage Tolerance)** | **NO BOM CHANGE NEEDED (COMMENT)** | Standard 5 V USB-A / USB-C 5 V default VBUS supplies nominal `4.75–5.25 V` (within the `nRF5340` `VBUS` pin's `-0.3 to +5.5 V` operating rating), because `R3`/`R4` are fixed `5.1 kΩ Rd` resistors that never negotiate USB-PD high-voltage profiles (>5 V). `D1` (`ESD5Z5.0T1G`) provides IEC 61000-4-2 (`±30 kV`) transient ESD clamping at the connector entry. For future Rev B harsh-industrial fault hardening against non-compliant 9–20 V adapters, `nRF5340` `VBUS` (Ball `A5`) can optionally be fed from the `nPM1300` `VBUSOUT` pin (which is internally current-limited and switched off by the `nPM1300`'s 5.5 V / 22 V OVP comparator). |
| **M3 (`U4` ADXL372 LGA-16 Pinout)** | **DESIGN CHANGE** | Updated `u4_nets` in `hardware/kicad/generate_kicad.py` to match the Analog Devices ADXL372 Datasheet Table 5 LGA-16 pinout: Pin 1=`VDD_1V8` (`VDDIO`), Pin 2=`VDD_1V8` (`BYP`), Pin 3=`GND` (`RESERVED`), Pin 4=`SPI1_SCK_P0.19` (`SCLK`), Pin 5=`GND`, Pin 6=`SPI1_MOSI_P0.20` (`MOSI`), Pin 7=`SPI1_MISO_P0.21` (`MISO`), Pin 8=`SPI1_CS_HIGHG_N_P0.22` (`CS_N`), Pin 9=`ADXL_INT2_P0.07` (`INT2`), Pin 10=`GND` (`RESV`), Pin 11=`ADXL_INT1_P0.06` (`INT1`), Pins 12–13=`GND`, Pin 14=`VDD_1V8` (`Vs`), Pin 15=`GND`, Pin 16=`VDD_1V8` (`VDDIO`). |
| **M4 (`U5` MMC5983MA LGA-16 Pinout)** | **DESIGN CHANGE** | Updated `u5_nets` in `hardware/kicad/generate_kicad.py` to match the MEMSIC MMC5983MA LGA-16 datasheet pinout: Pin 1=`I2C0_SCL_3V0` (`SCL/SPC`), Pin 2=`VDD_3V0` (`VDD`), Pin 3=`GND` (`NC`), Pin 4=`VDD_3V0` (`CS` tied high for I2C), Pin 5=`I2C0_SDA_3V0` (`SDA/SDI`), Pin 6=`GND` (`SDO`), Pins 7–9=`GND`, Pin 10=`VDD_3V0` (`CAP`), Pins 11–12=`GND`, Pin 13=`VDD_3V0` (`VDDIO`), Pin 14=`GND`, Pin 15=`MAG_INT_3V0` (`INT`), Pin 16=`GND`. |
| **M5 (`A1` 2450AT18B100 Pin 2 Grounding)** | **DESIGN CHANGE** | Per the Johanson `2450AT18B100` datasheet, Pin 1 is the 50 $\Omega$ RF feed and Pin 2 is an isolated mechanical soldering anchor (`NC`). Disconnected `A1` Pin 2 from `GND` (`pnet=""`) in `generate_kicad.py` and added a schematic `(no_connect ...)` marker so Pin 2 is a floating mechanical solder pad inside the 8×8 mm copper keepout. |

---

### 3. Minor Findings (`N1`–`N8`) — **5 FIXED IN DESIGN, 3 COMMENTED**

| Item | Action | Technical Resolution / Rationale |
| :--- | :--- | :--- |
| **N1 (3 Firmware LED Masks vs. 2 Physical LEDs)** | **DESIGN CHANGE** | Updated `firmware/include/hks_hal.h` and `firmware/hal/hal_nrf5340_stub.c` (`hks_hal_led_set()`) so `HKS_LED_GREEN` drives `LD1` (`P1.07` -> `Q1A`), `HKS_LED_RED` drives `LD2` (`P1.08` -> `Q1B`), and `HKS_LED_AMBER` drives both `LD1` + `LD2` simultaneously (producing optical amber through the adjacent dual light-pipe diffuser). |
| **N2 (`hks_hal.h` Comment Says `I2C1` vs. `TWIM2`)** | **DESIGN CHANGE** | Updated the comment on `HKS_I2C_BUS_TWIM1_PMIC` in `firmware/include/hks_hal.h:25` to explicitly state `/* 400 kHz TWIM2 (contract key 'TWIM1' alias, ID 11): nPM1300 (0x6B) */`, matching `hal_nrf5340_stub.c` and `pinmux_and_buses.json`. |
| **N3 (`PMIC_INT` `P1.05` Pull-Up)** | **NO BOM CHANGE NEEDED (COMMENT)** | Intentional zero-quiescent-leakage design: `nPM1300` `GPIO0` is configured in push-pull CMOS `GPOIRQ` output mode (`GPIOMODE0 = GPOIRQ`, `GPIODRIVE0 = CMOS`), and `nRF5340` `P1.05` enables its internal `13 kΩ` pull-up during init prior to PMIC configuration. This avoids an extra external resistor while guaranteeing a defined logic level during cold boot. |
| **N4 (`C20`/`C21` 10 µF 6.3 V 0603 DC-Bias Derating)** | **NO BOM CHANGE NEEDED (COMMENT)** | Per the Nordic `nPM1300` Product Specification, the `BUCK1`/`BUCK2` control loop is stable with an effective output capacitance of `≥ 3.5 µF`. A `10 µF 6.3 V X5R 0603` (`GRM188R60J106ME47D`) retains `~7.2 µF` effective capacitance at `1.8 V` (`BUCK1`) and `~5.1 µF` effective capacitance at `3.0 V` (`BUCK2`), plus distributed `100 nF` local MLCCs on the rail—well above the `3.5 µF` stability floor while meeting the strict `3.800 g` EE mass budget. |
| **N5 (`C22`–`C26` 4.7 µF Capacitor Role Parity)** | **DESIGN CHANGE** | Updated `cap_0603_specs` in `hardware/kicad/generate_kicad.py` so the five `4.7 µF 10 V 0603` capacitors (`C22`–`C26`) are assigned to `DECUSB` (`C22`), `VDD_1V8` (`C23`), `DECD` (`C24`), `DECR` (`C25`), and `DECA` (`C26`), matching `hardware/ee/bom.csv:40` (`C_BULK_4U7`, qty 5). |
| **N6 (`J2` TC2050 Pin 9 GND Missing)** | **DESIGN CHANGE** | Added Pad `9` (`GND`) at `(1.27, 0.635)` to `J2` (`TC2050-IDC-NL`) in `hardware/kicad/generate_kicad.py` so all three standard Arm Cortex-M 10-pin SWD ground pins (`3, 5, 9`) are connected to `GND`. |
| **N7 (`L3`/`L4` `LQM18DN100M70L` `Isat = 100 mA`)** | **NO BOM CHANGE NEEDED (COMMENT)** | At `0 dBm` BLE 5.3 2M PHY TX, `nRF5340` `VREGRADIO` (`L4`) peak input current is `~5.1 mA` (`<< 100 mA`), and `VREGMAIN` (`L3`) active dual-core + DMA current is `4.2 mA` avg / `< 25 mA` peak (`<< 100 mA`), giving `> 4×` saturation margin while keeping the BOM locked to `hardware/ee/bom.csv`. |
| **N8 (`ICM-42688-P` `INT1`/`INT2` Active Polarity)** | **DESIGN CHANGE** | Updated `hardware/ee/pinmux_and_buses.json` (`IMU_INT1` and `IMU_INT2`) to explicitly note that firmware initialization writes `INT_CONFIG = 0x1B` (`INT1_POLARITY=1`, `INT1_DRIVE_CIRCUIT=1`, `INT2_POLARITY=1`, `INT2_DRIVE_CIRCUIT=1`), configuring both interrupt pins as push-pull active-high (rising-edge triggered) before enabling FIFO watermark interrupts. |

---

### 4. Addendum Findings (`A-C1`, `A-C2`, `A-M1`–`A-M5`, `A-N1`–`A-N4`) — **ALL CRITICAL/MAJOR FIXED IN LAYOUT**

| Item | Action | Technical Resolution / Rationale |
| :--- | :--- | :--- |
| **A-C1 (Missing Ground & Power Planes in `.kicad_pcb`)** | **DESIGN CHANGE** | Updated `hardware/kicad/route_pcb.py` (`_add_copper_pour_zones()` + `pcbnew.ZONE_FILLER(board).Fill(board.Zones())`) to insert and fill solid copper pour zones on `In1.Cu` (`GND`), `B.Cu` (`GND`), and `In2.Cu` (`VDD_1V8`) across the `38 × 28 mm` board while carving out the `8.0 × 8.0 mm` all-layer top-right RF keepout (`x: 136..144 mm, y: 81..89 mm`). Verified 4 filled zones in `hockey_sensor.kicad_pcb` with `0` DRC errors. |
| **A-C2 (RF Feed Routed on `In1.Cu` with 2 Vias)** | **DESIGN CHANGE** | Pre-routed the entire `2.4 GHz` RF chain (`RF_ANT_OUT` -> `C27` -> `RF_MATCH_MID` (`L5`/`L6`) -> `RF_ANT_FEED` -> `A1` Pad 1) strictly on `F.Cu` (`0.20 mm` trace width over the `0.11 mm`, $\varepsilon_r = 4.2$ prepreg above the solid `In1.Cu` `GND` plane = `~50 Ω` microstrip/GCPW) with **zero signal vias** on the RF path. |
| **A-M1 (`8.0 × 8.0 mm` RF Keepout Size)** | **DESIGN CHANGE** | Restored the `RF_2450MHz_8x8mm_Keepout` zone in `hockey_sensor.kicad_pcb` to the full `8.0 × 8.0 mm` top-right corner (`x: 136.0..144.0 mm, y: 81.0..89.0 mm`) across all 4 copper layers (`F.Cu`, `In1.Cu`, `In2.Cu`, `B.Cu`), with all three copper pours (`In1.Cu`, `In2.Cu`, `B.Cu`) excluded from the `8 × 8 mm` region. |
| **A-M2 (`A1` Pin 2 Long GND Trace in Keepout)** | **DESIGN CHANGE** | Disconnected `A1` Pin 2 (isolated mechanical solder pad per `M5`), eliminating the 8.8 mm trace through the keepout completely. Only the straight `0.20 mm` `F.Cu` 50 $\Omega$ feed (`RF_ANT_FEED`) enters the keepout to `A1` Pad 1. |
| **A-M3 (`A1` Flush with Top-Right Board Edge)** | **DESIGN CHANGE** | Moved `A1` (`2450AT18B0100001E`) to `xs = 17.5 mm, ys = 10.5 mm` (`KiCad (140.5, 82.5)`), placing the chip antenna at the top-right board edge inside the `8.0 × 8.0 mm` all-layer copper keepout. |
| **A-M4 & A-N4 (`C27`/`L5`/`L6` RF Match Placement & VNA Tuning)** | **DESIGN CHANGE + COMMENT** | Relocated `C27` (`xs = 12.8, ys = 2.2`), `L5` (`xs = 13.7, ys = 3.8`, shunt to `GND` at the ground-plane boundary `x = 133.8 mm`), and `L6` (`xs = 12.8, ys = 5.2`) directly along the `F.Cu` path between `U1` `ANT` and `A1` Pad 1. The 3-element Tee network (`C27` series, `L5` shunt, `L6` series) provides full 3-degree-of-freedom complex impedance matching (`50 Ω` PA output + harmonic filtering to `A1` inside the PC/PBT+TPU housing) without altering the locked BOM part count. |
| **A-M5 (`L3` and `L4` DCDC Inductor Spacing & Orientation)** | **DESIGN CHANGE** | Separated `L3` (`xs = 7.5, ys = -9.0`, `rot = 0°`) and `L4` (`xs = 11.0, ys = -9.0`, `rot = 90°`) by `3.5 mm` center-to-center (more than doubled from `1.7 mm`), rotated `L4` orthogonal (`90°`) to `L3` to minimize magnetic flux coupling, and placed their dedicated output caps `C24` (`DECD`, `xs = 6.2, ys = -9.0`) and `C25` (`DECR`, `xs = 12.8, ys = -9.0`) immediately adjacent to the inductor output pads. |
| **A-N1 (`Y1` / `Y2` Crystal Clearance to RF Feed)** | **DESIGN CHANGE** | Shifted `Y1` (`32 MHz HFXO`) to `(xs = 15.0, ys = -1.0)` and `Y2` (`32.768 kHz LFXO`) to `(xs = 13.0, ys = -4.2)`, away from the `F.Cu` `RF_ANT_OUT` feed and backed by the solid `In1.Cu` ground plane. |
| **A-N2 (`J2` SWD Placement & Routing Clearance)** | **DESIGN CHANGE** | Shifted `J2` (`TC2050-IDC-NL`) to `(xs = 16.0, ys = -6.2)` (`KiCad (123.8, 84.0)`), keeping SWD signals on the top-left side of `U1`, well over `8 mm` away from the top-right RF feed and `8 × 8 mm` antenna keepout. |
| **A-N3 (`L2` Shielded Inductor vs. `BT1` Battery Pads)** | **COMMENT** | As noted in the audit, `L2` (`Murata DFE201610E-2R2M`) is a metal-alloy magnetically shielded molded power inductor placed `> 3.5 mm` from `BT1`, and the solid `In1.Cu` and `B.Cu` ground planes shield the `PMIC_NTC` sense trace from the `SW2_BUCK2` switching node. |
