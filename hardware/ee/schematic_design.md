# `hockey_sensor` — Electrical Engineering Schematic & PCB Design Specification

**Document ID:** `HKS-EE-1.0.0`  
**Contract Authority:** [`../../INTERFACE_CONTRACT.json`](../../INTERFACE_CONTRACT.json) (`v1.0.0`)  
**System Specification:** [`../../PROJECT_SPEC.md`](../../PROJECT_SPEC.md)  
**Pinmux & Bus Map:** [`pinmux_and_buses.json`](./pinmux_and_buses.json)  
**Electrical BOM:** [`bom.csv`](./bom.csv)

---

## 1. Anchor-Free & Zero-GPS Architecture Verification

`hockey_sensor` operates **100% autonomously on the player's chest** inside indoor ice rinks. By design and hardware exclusion:
- **Zero GPS/GNSS**: No GNSS receiver IC, no L1/L5 active/passive patch antenna, no LNA, no SAW filter (`uses_gps = false`).
- **Zero External Anchors / Beacons / UWB**: No UWB transceiver (`DW3000`/`SR150`), no Time-of-Flight ranging radio, no rink-side anchor sync circuitry (`uses_external_anchors = false`, `uses_uwb = false`).
- **On-Body Sensing Suite**: Nordic `nRF5340-QKAA` SoC, `nPM1300-QEAA-R` PMIC/charger/fuel gauge, `ICM-42688-P` 6-DOF IMU (`SPIM4`), `ADXL372BCCZ-RL7` ±200 g impact IMU (`SPIM1`), `MMC5983MA` 3-axis magnetometer (`TWIM0` via `PCA9306` level translator), `BMP581` barometer (`TWIM0`), and `W25N01JWZEIG` 128 MiB SLC QSPI NAND (`QSPI`).

---

## 2. Dual-Buck Power Tree & Charging Architecture (`nPM1300-QEAA-R`)

Although `INTERFACE_CONTRACT.json` defines `system_voltage_v = 1.8 V` for the SoC, digital buses, primary/high-g IMUs, barometer, and QSPI NAND flash, datasheet verification of `MEMSIC MMC5983MA` confirms that `MMC5983MA` requires **`2.8–3.6 V` (`3.0 V` nominal) on both `VDD` and `VDDIO`**. Simultaneously, `nRF5340-QKAA` operating in `1.8 V` Normal Voltage Mode has an absolute maximum GPIO input voltage of `VDD + 0.3 V = 2.1 V`.

To satisfy all IC datasheets without violating `1.8 V` SoC domain limits, the design utilizes **both integrated buck converters** of `U2` (`nPM1300-QEAA-R`, QFN-32):

```text
  [USB-C VBUS 5V] ---> [D1 TVS + C_VBUS 4.7uF 25V] ---> [nPM1300 VBUSIN] (22 V OVP, 500 mA USB current limit)
                                           |
  [502535 450 mAh LiPo] <----------------->+ (VBAT, JEITA 0..45 C via BT1 integrated 10k B=3380K NTC; RT1 DNI)
  (R_int=120mOhm@25C, 360mOhm@0C)          |
                                           +---> [BUCK1: 1.8 V] (L1=2.2 uH, R_VSET1=47 kOhm)
                                           |       |--> nRF5340-QKAA (VDD=VDDH=1.8V Normal Mode)
                                           |       |--> W25N01JWZEIG (VCC=1.8V)
                                           |       |--> ICM-42688-P (VDD=VDDIO=1.8V)
                                           |       |--> ADXL372BCCZ-RL7 (VS=VDDIO=1.8V)
                                           |       |--> BMP581 (VDD=VDDIO=1.8V, SDO=1V8 -> 0x47)
                                           |       |--> PCA9306DCUR VREF1 (1.8 V side)
                                           |       |--> SN74AXC1T45DRLR VCCA (1.8 V side)
                                           |       +--> Q1 (NTZD3154NT1G) 1.8 V Gates (P1.07/P1.08)
                                           |
                                           +---> [BUCK2: 3.0 V] (L2=2.2 uH, R_VSET2=150 kOhm)
                                                   |--> MMC5983MA (VDD=VDDIO=3.0V, SPI_CS=3V0 -> 0x30)
                                                   |--> PCA9306DCUR VREF2 (3.0 V side via 200k PU)
                                                   |--> SN74AXC1T45DRLR VCCB (3.0 V side)
                                                   +--> Status LEDs (LD1 Green, LD2 Red/Amber via 1 kOhm + Q1 low-side N-MOSFET)
```

### 2.1 `nPM1300-QEAA-R` Configuration & Fuel Gauging
- **`BUCK1` (`1.8 V` System Rail)**:
  - Programmed at hardware reset via `R_VSET1 = 47 kΩ` (±1%, 0402) from `VSET1` to `GND`.
  - Inductor `L1 = 2.2 µH` (shielded 2016, `DCR <= 110 mΩ`, `I_sat >= 1.2 A`), output cap `C_BUCK_10U` (`10 µF` `6.3 V` X5R 0603, `GRM188R60J106ME47D`).
  - Efficiency `η ≈ 90%` across `1–20 mA` active logging range (`Mode = Auto` hysteretic/PWM).
- **`BUCK2` (`3.0 V` Magnetometer & Status LED Auxiliary Rail)**:
  - Programmed at hardware reset via `R_VSET2 = 150 kΩ` (±1%, 0402) from `VSET2` to `GND`.
  - Inductor `L2 = 2.2 µH` (identical to `L1`), output cap `C_BUCK_10U` (`10 µF` `6.3 V` X5R 0603, `GRM188R60J106ME47D`).
  - Supplies `MMC5983MA` `VDD` & `VDDIO` (`0.350 mA` nom / `0.450 mA` max @ 50 Hz), the `3.0 V` side of `PCA9306DCUR` and `SN74AXC1T45DRLR`, and the anodes of status LEDs `LD1` (`LTST-C191KGKT`, green, $V_F \approx 2.0\text{ V}$ typ / $2.4\text{ V}$ max) and `LD2` (`LTST-C191KRKT`, red/amber) via `1 kΩ` current-limiting resistors. Can be software-disabled via PMIC I2C during deep standby (`Mode 5a/5b`).
- **Status LED Low-Side Driver (`Q1` `NTZD3154NT1G`)**:
  - Because `LTST-C191KGKT` has $V_F = 2.0\text{ V}$ typ ($2.4\text{ V}$ max), it cannot illuminate reliably on a `1.8 V` rail at `0 °C`. Therefore `LD1`/`LD2` anodes connect to **`BUCK2 = 3.0 V`** through `1 kΩ` resistors, and their cathodes are switched on the low side by **`Q1`** (`onsemi NTZD3154NT1G`, dual logic-level N-MOSFET, $V_{GS(\text{th})} \le 1.0\text{ V}$, `SOT-563`). `nRF5340` `1.8 V` GPIOs `P1.07` (`LED_STATUS_G`) and `P1.08` (`LED_STATUS_R`) drive the `Q1A`/`Q1B` gates (with `100 kΩ` gate pulldowns), keeping `nRF5340` GPIOs strictly `<= 1.8 V` (`<< 2.1 V` AbsMax).
- **Level Translation (`TWIM0` & `MAG_INT`)**:
  - **`U8` (`TI PCA9306DCUR`, VSSOP-8)**: Bidirectional passive FET I2C translator on the `MMC5983MA` branch of `TWIM0`. `VREF1` connects to `1.8 V`, `VREF2` connects to `3.0 V` via `200 kΩ` pull-up tied to `EN` (with `100 nF` filter cap to GND). `4.7 kΩ` pull-ups on `1.8 V` `SCL/SDA` (`TWIM0` shared with `BMP581` at `0x47`) and `4.7 kΩ` pull-ups on `3.0 V` `SCL_3V0/SDA_3V0` to `MMC5983MA` (`0x30`, `SPI_CS` tied to `3.0 V`).
  - **`U9` (`TI SN74AXC1T45DRLR`, SOT-563)**: Dual-supply unidirectional level translator for `MAG_INT` (`DIR = GND` or tied so `B -> A`: `3.0 V` `MMC5983MA INT` -> `1.8 V` `nRF5340 P1.01`).
- **JEITA Cold-Safe LiPo Charging & Fuel Gauge**:
  - `BT1`: `502535` `3.7 V` `450 mAh` LiPo pouch cell with integrated protection PCM (`2.80 V` HW UVLO, over-current, short-circuit) and integrated `10 kΩ` (`B = 3380 K`) NTC thermistor wired directly to `nPM1300 NTC` pin (`RT1` `NCP15XH103F03RC` on PCB is `DNI`, unpopulated backup footprint).
  - Charge current configured via I2C to `225 mA` (`0.5 C`). JEITA profile inhibits charging `< 0 °C` (critical after cold rink sessions until pod warms to locker-room temperature) and derates/terminates cleanly across `0–45 °C`.
  - System low-battery UVLO threshold set to **`3,000 mV` (`3.0 V`)** in PMIC/firmware (`> 540 mV` above `0 °C` 10% SOC loaded terminal voltage).
  - Fuel gauge reads `VBAT`, `IBAT` (via internal sense), and `NTC` temperature over I2C (`0x6B`) at `1 Hz` (`±3%` SOC accuracy, `±10 mV` voltage).
- **`SHPHLD` Ship Mode & Button Control**:
  - Low-profile IP67 tactile dome switch `SW1` (`Panasonic EVPAWCD4A`, `3.0 × 2.0 × 0.60 mm`) pulls `nPM1300 SHPHLD` to `GND`. Because `nPM1300 SHPHLD` is pulled up internally to `VBAT` (`3.7–4.35 V > 2.1 V` SoC GPIO AbsMax) and `nPM1300 GPIOMODE[n]` has no hardware `SHPHLD`-mirror mode, `P1.06 BTN_USER_N` is isolated from `SW1`/`SHPHLD` by passive Schottky diode **`D2`** (`BAT54WS`, `SOD-323`: cathode to `SW1`/`SHPHLD`, anode to `P1.06` with `100 kΩ` pull-up resistor `R_BTN_1V8_100K` to `1.8 V`). When `SW1` is open, `VBAT` reverse-biases `D2` so `P1.06` remains safely at `1.8 V`; when `SW1` is pressed to `GND`, `D2` conducts ($V_F \approx 0.20\text{ V}$ at $16\text{ }\mu\text{A} \ll V_{IL,\max} = 0.54\text{ V}$), pulling `P1.06` cleanly LOW while `nPM1300 GPIO0` (`GPOIRQ -> P1.05 PMIC_INT`) also signals `EVENTSSHPHLD` over `TWIM2`. `nPM1300 GPIO1` is left spare/unconnected. Pressing wakes pod from `370 nA` Ship Mode; long press enters Ship Mode; short press during active mode emits `0x05 EVENT_MARKER` (`BUTTON_MARK`) or starts/stops session.

---

## 3. `nRF5340-QKAA` Core, Decoupling & Clocks

### 3.1 `1.8 V` Normal Voltage Mode & Decoupling
- **Supply Mode**: `VDD` and `VDDH` are shorted together to **`BUCK1` (`1.8 V`)**, placing `nRF5340-QKAA` in Normal Voltage Mode (`VREGH` bypassed).
- **USB `VBUS` Connection**: Even in `1.8 V` Normal Voltage Mode, `nRF5340 VBUS` (`A5`) connects to USB-C `5 V` (`VBUS`) to power the internal `USBREG` LDO (`3.3 V` USB transceiver signaling) with a **`4.7 µF` ceramic capacitor on `DECUSB` (`B6`)**.
- **DC/DC Inductors**: On `nRF5340-QKAA`, **`DCCD` (Ball `B10`, decoupled at `DECD` `A15`)** is `VREGMAIN` and **`DCC` (Ball `A21`, decoupled at `DECR` `A27`)** is `VREGRADIO`. These independent DC/DC buck stages each require a **`10 µH`** `0603` power inductor (`L3 = 10 µH` Murata `LQM18DN100M70L` on `VREGMAIN`/`DCCD` [Ball `B10`], and `L4 = 10 µH` Murata `LQM18DN100M70L` on `VREGRADIO`/`DCC` [Ball `A21`]) per Nordic's `nRF5340` Product Specification reference circuitry.
- **Decoupling**: One `100 nF` X5R 0402 capacitor within `< 2 mm` of every `VDD` ball, plus `4.7 µF` bulk MLCCs on `VDD`, `DECA`, `DECD`, `DECN`, `DECR`.

### 3.2 Clocks (`+/- 50 ppm` Timestamp Compliance)
`INTERFACE_CONTRACT.json §units_and_tolerances.timestamp` mandates `<= ±50 ppm` (`<= 0.36 s` drift over 2 hours). Internal `LFRC` (`~500 ppm`) cannot meet this requirement.
- **`Y2` (`32.768 kHz` LFXO Crystal)**: Connected to `P0.00` (`XL1`, ball `N1`) and `P0.01` (`XL2`, ball `R1`). Tolerance `<= ±20 ppm` (temperature-stable `-40..+85 °C`, `C_L = 9 pF` internal programmable load caps enabled). Guarantees `<= 0.144 s` drift over 2 hours, comfortably inside the `±50 ppm` (`0.36 s`) contract limit.
- **`Y1` (`32 MHz` HFXO Crystal)**: Connected to `XC1` (`C31`) and `XC2` (`B30`), `±10–20 ppm`, required for BLE 5.3 2M PHY and USB 2.0 Full-Speed (`12 Mbps`) clock accuracy.

---

## 4. Isolated Peripheral Bus Topology & Silicon Erratum Resolution

### 4.1 Critical `nRF5340` Shared Peripheral ID 9 Resolution (`SPIM1` vs. `TWIM1`)
On `nRF5340-QKAA`, serial peripherals share hardware resource IDs and base addresses on the Application Core:
- **ID 8 (`0x40008000`)**: `SPIM0` / `TWIM0`
- **ID 9 (`0x40009000`)**: `SPIM1` / `TWIM1` *(mutually exclusive — CANNOT be active simultaneously!)*
- **ID 10 (`0x4000A000`)**: `SPIM4` (dedicated high-speed instance)
- **ID 11 (`0x4000B000`)**: `SPIM2` / `TWIM2`

`INTERFACE_CONTRACT.json §hardware_abstraction` logically labels the `ADXL372` bus `"SPIM1"` and the `nPM1300` PMIC bus `"TWIM1"`. In physical `nRF5340` silicon, enabling both `SPIM1` and `TWIM1` simultaneously causes a hardware peripheral ID 9 collision.
- **Resolution**: Assign `ADXL372` to **`SPIM1` (ID 9, `0x40009000`)** on `P0.19–P0.22`, and route the `nPM1300` PMIC I2C bus (`P0.23 SDA`, `P0.24 SCL`) via the `nRF5340` GPIO crossbar to physical silicon instance **`TWIM2` (ID 11, `0x4000B000`)**.
- In `hardware/ee/pinmux_and_buses.json`, `"TWIM1"` is retained as the JSON bus key for 1-to-1 `INTERFACE_CONTRACT.json` parity, explicitly annotated with `"silicon_instance": "TWIM2"`, `"peripheral_id": 11`, `"base_address": "0x4000B000"`, and `"silicon_erratum_note"`.
- **Every active bus occupies a unique peripheral ID**: `TWIM0` (ID 8), `SPIM1` (ID 9), `SPIM4` (ID 10), `TWIM2` (ID 11), `SPIM3` (ID 12, driving `W25N01JW` on dedicated `QSPI` pins `P0.13..P0.18`).

### 4.2 Bus Assignments & Speeds
1. **`SPIM4` (`16 MHz`, Mode 0/3, `1.8 V`, ID 10)**:
   - **Device**: `U3` (`ICM-42688-P`, LGA-14, up to 24 MHz capable).
   - **Pins**: `P0.08` (`SCK`, `H0H1`), `P0.09` (`MOSI`, `H0H1`), `P0.10` (`MISO`), `P0.11` (`CS_IMU_N`, `10 kΩ` PU to `1.8 V`).
   - **Firmware Note**: Default Zephyr `cpunet` DTS maps `uart0` to `P0.10/P0.11` and `P0.19–P0.22`; board DTS overlay must set `status = "disabled"` on `cpunet` `uart0` so `SPIM4` and `SPIM1` operate cleanly.
   - **Interrupts**: `IMU_INT1` (`P0.04`, 20-sample FIFO watermark @ 100 ms), `IMU_INT2` (`P0.05`, DRDY / WoM).
2. **`SPIM1` (`8 MHz <= 10 MHz` max, Mode 0, `1.8 V`, ID 9 — Isolated)**:
   - **Device**: `U4` (`ADXL372BCCZ-RL7`, LGA-16, hard `10 MHz` SCLK ceiling, Mode 0 `CPOL=0, CPHA=0`).
   - **Pins**: `P0.19` (`SCK`, `H0H1`), `P0.20` (`MOSI`, `H0H1`), `P0.21` (`MISO`), `P0.22` (`CS_HIGHG_N`, `10 kΩ` PU to `1.8 V`).
   - **Interrupts**: `ADXL_INT1` (`P0.06`, Instant-On impact wake `> 12 g`), `ADXL_INT2` (`P0.07`, FIFO watermark).
3. **`TWIM0` (`400 kHz` Fast Mode, `1.8 V ↔ 3.0 V`, ID 8)**:
   - **Pins**: `P1.02` (`I2C0_SDA`, high-drive `S0D1/E0E1`), `P1.03` (`I2C0_SCL`, `S0D1/E0E1`), `4.7 kΩ` pull-ups to `1.8 V`.
   - **`U6` (`BMP581`)**: Direct on `1.8 V` side; **`SDO` tied to `1.8 V`** -> 7-bit I2C address **`0x47`** (`0x46` if `GND`); `BARO_INT` on `P1.04` (25 Hz).
   - **`U5` (`MMC5983MA`)**: Behind `U8` (`PCA9306DCUR`) on `3.0 V` side (`4.7 kΩ` pull-ups to `3.0 V`); `SPI_CS` tied to `3.0 V` -> 7-bit I2C address **`0x30`**; `MAG_INT` (`50 Hz`) translated `3.0 V -> 1.8 V` via `U9` (`SN74AXC1T45DRLR`) to `P1.01`.
4. **`TWIM1` (Contract) / `TWIM2` (Silicon ID 11, `400 kHz`, `1.8 V`)**:
   - **Pins**: `P0.23` (`I2C2_SDA`, `S0D1`), `P0.24` (`I2C2_SCL`, `S0D1`), `4.7 kΩ` pull-ups to `1.8 V`.
   - **Device**: `U2` (`nPM1300-QEAA-R`, 7-bit address **`0x6B`**, 16-bit register addressing).
   - **Signals**: `PMIC_INT` (`P1.05`, `nPM1300 GPIO0` `GPOIRQ`), `BTN_USER_N` (`P1.06`, isolated from `SW1`/`SHPHLD` via Schottky `D2` `BAT54WS` + `100 kΩ` `1.8 V` pull-up; `nPM1300 GPIO1` spare).
5. **`QSPI` Contract Bus / `SPIM3` Silicon (`8 MHz` 1-bit SPI NAND, `1.8 V`, ID 12 `0x4000C000`)**:
   - **Pins**: Dedicated `P0.13–P0.16` (`IO0`/`MOSI`, `IO1`/`MISO`, `IO2`/`/WP = 1V8`, `IO3`/`/HOLD = 1V8`, `H0H1`), `P0.17` (`SCK`, `H0H1`), `P0.18` (`FLASH_CS_N`, `10 kΩ` PU to `1.8 V`).
   - **Device**: `U7` (`Winbond W25N01JWZEIG`, 1 Gbit / 128 MiB SLC NAND, WSON-8 8×6 mm).
   - **M3 Firmware Note**: `W25N01JW` is an SLC SPI NAND (not NOR XIP). Because `nRF5340 QSPI.CINSTRCONF` executes strictly on a single data line (`IO0`/`IO1`, `<= 8` payload bytes) and `QSPI` EasyDMA `READ4O`/`PP4O` hard-codes 24-bit addresses versus `W25N01JW`'s 16-bit column address, M3 firmware drives `W25N01JW` over the shared dedicated pins `P0.13..P0.18` using **`SPIM3` (Peripheral ID 12, `0x4000C000`, `8 MHz` / `8 Mbps` / `~1.0 MB/s` wire rate 1-bit SPI NAND mode — respecting the `nRF5340` `SPIM0..3` `8 MHz` silicon ceiling — with `P0.15 /WP = 1V8` and `P0.16 /HOLD = 1V8`)** — easily sustaining `8,044 B/s` logging and `650 KB/s` USB FS read offload with zero ID conflict against `TWIM0(8)`, `SPIM1(9)`, `SPIM4(10)`, or `TWIM2(11)` — or alternatively hybrid `QSPI` (`CINSTRCONF` control + `READOC=READ 0x03` 1-bit 24-bit `[16b CA + 8b dummy]`) as an optional faster read alternative, with bad-block skip/ECC management across the `998` usable blocks. Contract JSON key `"QSPI"` is retained in `pinmux_and_buses.json` for `INTERFACE_CONTRACT.json` parity.

---

## 5. USB-C FS, ESD Protection, 2.4 GHz RF & SWD Debug

### 5.1 USB-C Full-Speed (`12 Mbps` CDC-ACM) & ESD
- **Receptacle (`J1`)**: `GCT USB4720-03-A` 16-pin mid-mount IP67 waterproof USB-C with pre-installed LIM silicone gasket.
- **USB-C Configuration Channel (`CC1`, `CC2`)**: Dedicated `5.1 kΩ` (±1%, 0402) `Rd` pulldown resistors on both `CC1` and `CC2` to `GND` (ensuring compliant USB-C host/cable `5 V` VBUS presentation).
- **ESD / Surge Protection**:
  - `U10` (`TI TPD4E05U06DQAR`, USON-10): Ultra-low capacitance (`0.42–0.5 pF`) flow-through ESD clamp placed `< 8 mm` from `J1` on `D+`/`D-` (`90 Ω` differential impedance pair routed over unbroken L2 GND).
  - `D1` (`ESD5Z5.0T1G` or bidirectional 5.5 V TVS): Placed directly at `J1 VBUS` pin to clamp hot-plug transients before `nPM1300 VBUSIN`, paired with **`C_VBUS`** (`Murata GRM188R61E475KE11D`, `4.7 µF` `25 V` X5R `0603`) rated safely above `5 V` nominal `VBUS`, `ESD5Z5.0T1G` clamping (`11.6–18.6 V`), and `nPM1300` `22 V` OVP (`6.3 V` MLCCs `C_BUCK_10U` are restricted strictly to `BUCK1 1.8 V` and `BUCK2 3.0 V` outputs).

### 5.2 2.4 GHz BLE 5.3 Chip Antenna & RF Keepout
- **Antenna (`A1`)**: `Johanson 2450AT18B0100001E` (3.2 × 1.6 × 1.3 mm) mounted at top-edge corner of PCB (`+X_S` edge, away from chest/battery copper).
- **Feed & Matching**: `50 Ω` grounded coplanar waveguide (GCPW) from `nRF5340 ANT` (`L31`) through a 3-element Tee matching network (`1.2 pF` series, `2.7 nH` shunt, `3.3 nH` series; tuned with VNA inside final PC+ABS/TPU housing).
- **4-Layer RF Keepout**: Strict **copper-free keepout zone (`8.0 × 8.0 mm`) across all 4 PCB layers (`L1–L4`)** directly beneath and around `A1`, bordered by a `0.5 mm`-pitch ground-stitch via fence.
- **Near-Radio Pin Discipline**: `nRF5340` datasheet-designated near-radio pins (`P1.01`, `P1.04`, `P1.05`, `P1.06`, `P1.07`, `P1.08`) are restricted solely to `S0S1` standard drive and low-frequency signals **`<= 10 kHz`** (`50 Hz` `MAG_INT`, `25 Hz` `BARO_INT`, `<= 1 Hz` `PMIC_INT`, `BTN_USER_N`, status LEDs) to prevent RF desense.

### 5.3 SWD Factory Programming & Debug Header
- **Footprint (`J2`)**: Tag-Connect `TC2050-IDC-NL` (zero-mass PCB pad footprint, no connector installed in production).
- **Signals**: `SWDIO` (`AA31`), `SWDCLK` (`W31`), `NRESET` (`AC31`), `VDD_1V8` sense, `GND`.

---

## 6. 4-Layer PCB Stackup, Layout Rules & Frame `S` Orientation

- **Dimensions & Finish**: `38.0 × 28.0 × 0.80 mm` (`±0.08 mm`), 4-layer FR4 (`Tg >= 150 °C`), `ENIG` finish, single-sided top assembly (`L1` populated, flat `L4` over battery Poron pad).
- **Layer Stackup**:
  - `L1 (Top)`: High-speed SPI/QSPI/USB traces, `50 Ω` RF GCPW, `ICM-42688-P`, `ADXL372`, `MMC5983MA`, `BMP581`, `nRF5340`, `nPM1300`, `W25N01JW`.
  - `L2 (Inner 1)`: Solid unbroken `GND` reference plane (voided only under `8.0 × 8.0 mm` RF keepout).
  - `L3 (Inner 2)`: `1.8 V` (`BUCK1`) and `3.0 V` (`BUCK2`) split power pours + low-speed IRQ/I2C routing.
  - `L4 (Bottom)`: Solid `GND` shield / flat thermal/mechanical surface facing Poron pad (no bottom components; `TC2050` pads accessible during manufacturing).
- **Frame `S` (`R_BS`) Physical Alignment**:
  - `ICM-42688-P`, `ADXL372`, and `MMC5983MA` are placed near the PCB center away from `M1.4` mounting bosses to minimize board-flex stress.
  - All three 3-axis sensors are oriented so their measurement axes align with canonical **Sensor Frame `S`**:
    - **`+X_S`**: Up along `38.0 mm` PCB axis (`+Z_B` toward head)
    - **`+Y_S`**: Player right (`-Y_B`)
    - **`+Z_S`**: Outward normal away from chest (`+X_B`, right-hand rule `+X_S × +Y_S = +Z_S`)
  - Yielding $R_{BS} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$ ($\det(R_{BS}) = +1$). Silkscreen on `L1` marks `+X_S ↑ TOP`, `+Y_S →`, `+Z_S ⊙ OUT` triad and Pin-1 fiducials.
- **Acoustic/Baro Port**: `BMP581` sits directly underneath the enclosure's `2.5 mm` recessed `ePTFE` Gore vent port with a `0.5 mm` silicone perimeter seal ring on `L1`.
