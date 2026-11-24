# `hockey_sensor` — Production System Specification

**Document ID:** `HKS-SPEC-1.0.0`  
**Contract Authority:** [`INTERFACE_CONTRACT.json`](./INTERFACE_CONTRACT.json) (`v1.0.0`)  
**Architecture Reference:** [`docs/architecture.md`](./docs/architecture.md)  

---

## 1. Executive Summary & System Constraints

`hockey_sensor` is a production-grade, chest-worn, anchor-free hockey player tracking system engineered for real indoor ice rink deployment. The system operates **entirely on-body with zero reliance on GPS/GNSS and zero external rink-side anchors, beacons, cameras, or UWB infrastructure**.

### 1.1 Hard System Requirements (Traceable to Request)
| ID | Requirement | Specification Lock |
|---|---|---|
| **R1.1** | Tracked Player Metrics (5) | **Speed** (`m/s`), **Acceleration** (`m/s^2`, `0–200 g`), **On-Ice 2D Position** (`(x, y)` in rink frame `W`), **Shift Length** (`s`), **Distance Skated** (`m`) |
| **R1.2** | Anchor-Free Indoor Operation | `gps_enabled = false`, `external_anchors_enabled = false`; all sensing on-chest (`nRF5340-QKAA` + 9-DOF + high-g + baro) |
| **R2.1** | Under-Jersey Low-Profile Packaging | `52.0 × 36.0 × 10.5 mm`, `~24 g` (`<= 28 g` max), `IP67` chamfered PC+ABS + Shore 70A TPU overmold, worn in compression shirt pocket or 25 mm chest strap |
| **R2.2** | Cold-Rated Battery Life (`>= 2 h`) | `450 mAh` `502535` LiPo derated to `65%` (`292.5 mAh`) at `0 °C` + aging: **`234.3 h`** nominal (`1.249 mA`), **`91.3 h`** worst-case (`3.205 mA`) (`45.6×` margin) |
| **R2.3** | Post-Session Download | `W25N01JW` `128 MiB` (`1 Gbit`) SLC NAND (`6.76 h` nominal / `4.54 h` high-g continuous); USB 2.0 FS CDC-ACM primary (`~59.5 s` / 2h log) + BLE 5.3 2M PHY secondary (`~5.16 min` / 2h log) |

---

## 2. System Architecture Overview

The system splits cleanly into three decoupled domains bridged by `INTERFACE_CONTRACT.json`:
1. **Wearable Pod (`hardware/`)**: Unified `1.8 V` low-power sensor node built around Nordic `nRF5340-QKAA`, `nPM1300` PMIC/charger/fuel gauge, `ICM-42688-P` 6-DOF IMU, `ADXL372` ±200 g impact accelerometer, `MMC5983MA` 3-axis magnetometer, `BMP581` barometer, `W25N01JW` SLC QSPI NAND, and `502535` 450 mAh LiPo + 10k NTC.
2. **Embedded Firmware (`firmware/`)**: Real-time Zephyr/C11 logger running at `1.8 V`, reading `ICM-42688-P` FIFO (`200 Hz`, batched `N=20` at `10 Hz`), monitoring `ADXL372` (`3200 Hz` impact burst `> 12 g` or `400 Hz` continuous), polling `MMC5983MA` (`50 Hz`), `BMP581` (`25 Hz`), and `nPM1300` (`1 Hz`), and committing `CRC32-IEEE`-protected binary `.hks` frames (`uint64_t timestamp_us`) into a wear-leveled `TELEMETRY_RING` (`998` erase blocks = `130,809,856 B`).
3. **Host Downloader & Anchor-Free Analytics Pipeline (`sim/`)**: Downloads `.hks` files via USB CDC-ACM (`LIST_SESSIONS` → `GET_HEADER` → `READ_CHUNK` → `ACK_VERIFY` → `DELETE_SESSION`) or BLE 5.3 2M PHY, verifies frame `CRC32-IEEE` and session `SHA-256`, and runs the 6-layer drift-bounded **15-state Error-State EKF + Rauch-Tung-Striebel (RTS) backward smoother** and shift detector to emit a contract-validated `analytics_report` JSON.

---

## 3. Hardware Specification & Peripheral Bus Map

### 3.1 Core Silicon & Component Selection
| Subsystem | Part Number | Key Parameters | Rationale |
|---|---|---|---|
| **MCU / SoC** | `nRF5340-QKAA` | Dual Arm Cortex-M33 (128/64 MHz App + 64 MHz Net), `1.8 V` rail, USB 2.0 FS, BLE 5.3 2M PHY, `SPIM4` (32 MHz), `QSPI` (96 MHz) | Dedicated high-speed `SPIM4` + `QSPI` + native USB FS without external bridge; low active current (`~3.7 mA` @ 64 MHz DC/DC) |
| **PMIC & Fuel Gauge** | `nPM1300` | Dual 200 mA buck (Buck1 = `1.8 V`, 90–93% eff.), JEITA 10k NTC LiPo charger, algorithm fuel gauge (`±3%`), `370 nA` ship mode | Replaces separate charger + LDO + fuel gauge IC; hardware `SHPHLD` ship-mode wakeup and cold-charge protection (`< 0 °C` charge inhibit) |
| **Primary 6-DOF IMU** | `ICM-42688-P` | `±16 g` (`2048 LSB/g` = `0.0047884033203125 (m/s^2)/LSB`), `±2000 dps` (`16.4 LSB/dps` = `0.001064225153655079 (rad/s)/LSB`), `200 Hz` ODR logged (internal 2 kHz AAF), `0.88 mA` | Low gyro bias drift (`<= 0.005 rad/s` post-cal) and 2 KB hardware FIFO (`20` samples / `100 ms` burst DMA read) |
| **High-g Impact IMU** | `ADXL372` | `±200 g` nominal (`[-2048, +2047]` raw = `[-2008.40, +2007.42] m/s^2`), 12-bit right-aligned `>> 4` (`100 mg/LSB` = `0.980665 (m/s^2)/LSB`), `3200 Hz` impact burst (`22 µA`) / `400 Hz` continuous, 512-sample FIFO, `1.4 µA` instant-on | Captures body checks & board collisions (`20–100+ g`) where `ICM-42688-P` clips at `±16 g`; isolated on `SPIM1` (`<= 10 MHz` max SPI clock) |
| **Magnetometer** | `MMC5983MA` | `±800 µT` (`±8 G`), 18-bit centered (`16384 counts/G` = `0.006103515625 uT/LSB`), auto SET/RESET degaussing, `50 Hz` ODR, `~0.35 mA` | High dynamic range resists saturation near rink dasher-board steel; integrated SET/RESET cancels residual magnetization |
| **Barometer** | `BMP581` | `30–125 kPa`, 24-bit `(signed, 24, 6)` (`1/64 Pa/LSB` = `0.015625 Pa/LSB`), `< 0.1 Pa` RMS noise, `25 Hz` ODR, `~45 µA` | Resolves `0.2–0.5 m` vertical step-over / bench-sit elevation cues (`2.4–6.0 Pa` delta) behind hydrophobic `ePTFE` vent |
| **Telemetry Flash** | `W25N01JW` | `1 Gbit` (`128 MiB` / `134,217,728 B`) `1.8 V` SLC QSPI NAND, `1,024` blocks × `64` pages × `2,048 B` (+ `64 B` ECC), `t_PP ≈ 250 µs` | On-die ECC + SLC endurance (`100k` P/E cycles); stores `6.76 h` nominal (`998` usable `128 KiB` blocks) |
| **Battery** | `502535` LiPo | `3.7 V` nominal (`1.665 Wh`), `450 mAh`, integrated PCM + `10k Ω` (`B=3380 K`) NTC | Thin `5.0 × 25 × 35 mm` pouch fits flat inside `10.5 mm` enclosure; NTC enforces safe cold charging |

### 3.2 `nRF5340-QKAA` Bus & Interrupt Map (`1.8 V` Unified Domain)
| Peripheral | Protocol / Clock | Slave Device(s) | Address / CS | Dedicated Signals |
|---|---|---|---|---|
| **`SPIM4`** | SPI Mode 0/3 @ **`16 MHz`** | `ICM-42688-P` | `CS_IMU_N` | `IMU_INT1` (20-sample FIFO WM @ `100 ms`), `IMU_INT2` (DRDY) |
| **`SPIM1`** | SPI Mode 0 @ **`8 MHz`** *(isolated: ADXL max 10 MHz)* | `ADXL372` | `CS_HIGHG_N` | `ADXL_INT1` (impact wake `> 12 g`), `ADXL_INT2` (FIFO WM) |
| **`TWIM0` (`I2C0`)** | I2C Fast Mode @ **`400 kHz`** | `MMC5983MA`, `BMP581` | **`0x30`**, **`0x47`** *(SDO=1V8; alt `0x46`)* | `MAG_INT` (DRDY), `BARO_INT` (DRDY/FIFO) |
| **`TWIM1` (`I2C1`)** | I2C Fast Mode @ **`400 kHz`** | `nPM1300` | **`0x6B`** *(16-bit reg addr)* | `PMIC_INT`, `SHPHLD`, `10k NTC`, `VBUS` detect |
| **`QSPI`** | Quad SPI SDR @ **`48–96 MHz`** | `W25N01JW` | `FLASH_CS_N` | `FLASH_IO[0..3]`, `FLASH_SCK` |
| **`USBD`** | USB 2.0 Full-Speed **`12 Mbps`** | USB-C IP67 Receptacle | CDC-ACM | `VBUS`, `D+/D-` via `TPD4E05U06` ESD, `CC1/CC2` `5.1 kΩ` |
| **BLE Radio** | BLE 5.3 **`2M PHY`** (2.4 GHz) | `2450AT18B100` chip ant | GATT NUS / L2CAP CoC | Secondary wireless offload (`~125 KB/s`), optional `1 Hz` beacon |

> **Hardware Isolation Invariant:** `ADXL372` MUST NOT share `SPIM4` with `ICM-42688-P` because `ADXL372` has a hard `10 MHz` maximum SCLK specification whereas `ICM-42688-P` runs at `16 MHz` for rapid DMA FIFO drain. `BMP581` uses 7-bit address `0x47` (`SDO` tied to `1.8 V`; alternate `0x46` when `SDO=GND`).

---

## 4. Coordinate Frames, Kinematics & Rink Geometry

### 4.1 Right-Handed Reference Frames ($\hat{X} \times \hat{Y} = \hat{Z}$)
- **Rink World Frame (`W`)**: Fixed to ice surface. Origin `(0, 0, 0)` at center-ice faceoff dot on $z = 0$. `+X_W` toward attacking goal along longitudinal centerline, `+Y_W` toward left boards when facing `+X_W`, `+Z_W` vertically upward normal to ice. Gravity vector: $\mathbf{g}^W = [0,\ 0,\ -9.80665]^T\text{ m/s}^2$.
- **Body / Torso Frame (`B`, FLU)**: Origin at geometric center of sternum pod. `+X_B` forward out of sternum, `+Y_B` player left, `+Z_B` upward along spine toward head. Identity quaternion $q_{WB} = [1, 0, 0, 0]^T$ corresponds to an upright skater at center ice facing `+X_W`.
- **Sensor Chip Frame (`S`)**: Rigidly affixed to the vertical sternum PCB with `+X_S` up (`+Z_B`), `+Y_S` player right (`-Y_B`), and `+Z_S` outward normal away from chest (`+X_B`). Satisfies $\hat{X}_S \times \hat{Y}_S = +\hat{Z}_S$ and maps `S → B` via proper orthogonal rotation $R_{BS} \in SO(3)$:
  $$R_{BS} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}, \quad R_{BS}^T R_{BS} = I_3, \quad \det(R_{BS}) = +1$$

### 4.2 Quaternion & Calibration Equations
- **Quaternion Convention**: Hamilton scalar-first $q_{WB} = [q_w, q_x, q_y, q_z]^T$, unit-norm $\|q_{WB}\|_2 = 1 \pm 10^{-6}$, active Body→World rotation $\mathbf{v}^W = R(q_{WB})\mathbf{v}^B$:
  $$\mathbf{a}^W = R(q_{WB})\mathbf{f}^B + \mathbf{g}^W, \qquad q_{WB}(t + \Delta t) = q_{WB}(t) \otimes \exp\!\left(\frac{1}{2}\boldsymbol{\omega}^B \Delta t\right)$$
- **Sensor Calibration (`0x06 calibration_block`)**:
  - Primary Accel (`ICM-42688-P`): $\mathbf{a}_{\text{cal}}^S = M_a \mathbf{a}_{\text{raw}}^S - \mathbf{b}_a^S$, $\mathbf{a}^B = R_{BS}\,\mathbf{a}_{\text{cal}}^S$
  - Gyroscope (`ICM-42688-P`): $\boldsymbol{\omega}_{\text{cal}}^S = M_g \boldsymbol{\omega}_{\text{raw}}^S - \mathbf{b}_g^S$, $\boldsymbol{\omega}^B = R_{BS}\,\boldsymbol{\omega}_{\text{cal}}^S$
  - High-g Accel (`ADXL372`): $\mathbf{a}_{hg,\text{cal}}^S = M_{hg} \mathbf{a}_{hg,\text{raw}}^S - \mathbf{b}_{hg}^S$, $\mathbf{a}_{hg}^B = R_{BS,hg}\,\mathbf{a}_{hg,\text{cal}}^S$
  - Magnetometer (`MMC5983MA`): $\mathbf{m}_{\text{cal}}^S = S_m(\mathbf{m}_{\text{raw}}^S - \mathbf{h}_m^S)$, $\mathbf{m}^B = R_{BS}\,\mathbf{m}_{\text{cal}}^S$

### 4.3 Rink Geometry Presets & Containment Law
| Preset | Length $L$ (`m`) | Width $W$ (`m`) | Corner Radius $R_c$ (`m`) | Goal Lines (`x`, `m`) | Blue Lines (`x`, `m`) | Default Home Bench Gate (`m`) |
|---|---|---|---|---|---|---|
| `NHL_200X85FT` | `60.96` | `25.908` | `8.5344` (`28 ft`) | `±27.1272` (`3.3528` from boards) | `±7.62` (`22.86` from boards) | `(-6.0, +12.954)` |
| `IIHF_60X30M` | `60.0` | `30.0` | `8.5` | `±26.0` (`4.0` from boards) | `±7.5` (`22.5` from boards) | `(-6.0, +15.0)` |
| `CUSTOM` | `56.0` *(configurable)* | `26.0` | `7.5` | `±24.5` (`3.5` from boards) | `±7.0` (`21.0` from boards) | `(-5.0, +13.0)` |

For $a = L/2 - R_c$, $b = W/2 - R_c$, $d_x = \max(|x| - a, 0)$, $d_y = \max(|y| - b, 0)$, valid player torso COM satisfies $\sqrt{d_x^2 + d_y^2} \le R_c - r_{\text{torso}}$ ($r_{\text{torso}} = 0.25\text{ m}$).

---

## 5. Indoor Anchor-Free Positioning, Shift Detection & Honest Error Bounds

### 5.1 6-Layer Drift-Bounding Pipeline
Because pure IMU double-integration diverges cubically in time without GPS/anchors, `hockey_sensor` bounds drift through six coupled physics & biomechanical layers:
1. **Layer 1 — Dual-IMU Saturation Handoff**: When $\|\mathbf{a}_{\text{ICM}}\|_\infty > 14.5\text{ g}$, the `ADXL372` (`±200 g`) stream substitutes sample-for-sample (time-aligned and transformed via $R_{BS,hg}$), preserving impulse $\Delta\mathbf{v}$ across 20–100 g body checks and board hits.
2. **Layer 2 — 15-State Error-State EKF (ES-EKF)**: $\delta\mathbf{x} = [\delta\mathbf{p}^W, \delta\mathbf{v}^W, \delta\boldsymbol{\theta}^W, \delta\mathbf{b}_a^S, \delta\mathbf{b}_g^S]^T$, mechanized at `200 Hz` with covariance updates at `50–100 Hz`.
3. **Layer 3 — ZUPT / ZARU Stationary Resets**: Windows $\ge 0.4\text{ s}$ with $\|\boldsymbol{\omega}\| < 6^\circ/\text{s}$ and $\text{std}(\|\mathbf{a}\|) < 0.04\text{ g}$ (bench sits, faceoffs, play stoppages) inject $\mathbf{v}^W = \mathbf{0}$ ($\sigma_v = 0.05\text{ m/s}$) and zero-angular-rate gyro bias updates.
4. **Layer 4 — Non-Holonomic Constraints (NHC) + Stride-Speed Pseudo-Measurement**: Enforces ice-plane vertical constraint $v_z^W \approx 0$ ($\sigma_{v_z} = 0.15\text{ m/s}$) and soft lateral constraint $v_y^B \approx 0$ ($\sigma_{v_y} = 0.5\text{–}1.2\text{ m/s}$, relaxed during sharp turns $|\omega_z| \ge 90^\circ/\text{s}$ or hockey stops). During `STRIDING` ($f_{\text{stride}} \in [0.8, 2.5]\text{ Hz}$), lateral torso impulse energy $E_{\text{stride}}$ feeds forward speed $v_{\text{bio}} = \alpha f_{\text{stride}} + \beta\sqrt{E_{\text{stride}}} + \gamma$ ($\sigma = 0.7\text{–}1.0\text{ m/s}$; disabled during `GLIDING`).
5. **Layer 5 — Rink Geometry Clamping, Board Collision & Bench-Gate Anchors**: Projects out-of-bounds states onto the rounded-rectangle perimeter and zeroes outward normal velocity $v_n$; perimeter impacts $> 12\text{ g}$ apply board-wall normal constraint; `BENCH ↔ ON_ICE` transitions inject soft coordinate resets at $(x_{\text{bench}}, y_{\text{bench}})$ ($\sigma_{xy} = 2.5\text{ m}$).
6. **Layer 6 — Adaptive Magnetometer Anomaly Gating + RTS Backward Smoother**: `MMC5983MA` heading updates are accepted only when $\|\mathbf{B}\| \in [0.88, 1.12]\,\|\mathbf{B}_0\|$ and $|\Delta\phi_{\text{dip}}| < 8^\circ$ (rejecting dasher-board rebar and Zamboni-door steel distortions). Post-session Rauch-Tung-Striebel (RTS) backward smoothing runs between bench-gate anchors, cutting mid-shift peak drift by ~50%.

### 5.2 Shift Detection State Machine
Sliding `1.0 s` window (`0.25 s` step) computes stride-band power $P_{\text{stride}}$ (`0.8–2.5 Hz`), dynamic accel RMS $a_{\text{dyn,rms}}$, gyro RMS $\omega_{\text{rms}}$, torso forward lean $\theta_{\text{pitch}}$, and `BMP581` elevation step (`0.2–0.5 m` gate step-over):
- **`IDLE_OFF_ICE → BENCH`**: Mounted on player (`a_dyn,rms > 0.05 g` or vertical orientation + `> 30 s` continuous logging).
- **`BENCH → ON_ICE_SKATING`**: $P_{\text{stride}} > \tau_{\text{stride}}$ AND $a_{\text{dyn,rms}} > 0.15\text{ g}$ sustained $\ge 2.0\text{ s}$ (or baro step-over cue + `1.0 s` skating).
- **`ON_ICE_SKATING ↔ ON_ICE_STOPPAGE`**: $a_{\text{dyn,rms}} < 0.06\text{ g}$ and upright $\ge 2.0\text{ s}$ enters `ON_ICE_STOPPAGE`; stays within the same shift if stoppage $< 45\text{ s}$.
- **`ON_ICE → BENCH`**: $\theta_{\text{pitch}} < 8^\circ$ AND $a_{\text{dyn,rms}} < 0.05\text{ g}$ AND $\omega_{\text{rms}} < 8^\circ/\text{s}$ sustained $\ge 5.0\text{ s}$ (back-dates shift end to start of stillness window).
- Trajectory output enum maps samples to `BENCH`, `GLIDING`, `STRIDING`, `STOPPING`, `IMPACT`, or `UNKNOWN`.

### 5.3 Honest Quantified Error Bounds & Failure Modes (All 5 Tracked Metrics)
| Tracked Metric | Nominal Target (Calibrated) | Worst-Case Real-Rink Bound | Primary Error / Failure Mode |
|---|---|---|---|
| **1. Acceleration (`0–200 g`)** | `±1% FS` (`ICM` `±16 g`), `±2% FS` (`ADXL372` `±200 g`) | `±2% FS` sensor; `-10% to -25%` chest-pad mechanical damping | Soft-tissue / base-layer / shoulder-pad viscoelastic attenuation relative to skeletal sternum impact |
| **2. Shift Length (`s`)** | `±1.5 s` boundary error (`>= 95%` shift F1) | `±2.0–3.0 s` boundary error | Standing completely motionless on-ice `> 45 s` (injury/video review delay) misclassified as `BENCH` |
| **3. Instantaneous Speed (`m/s`)** | `±0.4 m/s` RMSE | `±0.6–0.8 m/s` RMSE (`~8–12%`) | Prolonged passive coasting (`GLIDING`), backward skating, or uncalibrated stride-to-speed coefficient |
| **4. Distance Skated (`m`)** | `±5%` (with `player_height_cm` & 1-min cal) | `±8–12%` per session | Unentered player height / skating efficiency variance (`±10%` scale bias) |
| **5. On-Ice 2D Position (`m`)** | **`2.5–3.5 m` CEP50** per 45 s shift (bench reset + RTS) | **`<= 6.0 m` CEP50** per 45 s shift; **`> 10–25 m` unbounded** if `> 2 min` non-stop without bench/wall resets | Fundamental anchor-free physics ceiling: suitable for **zone/shift heatmaps & possession trends**, NOT sub-meter puck/pass tracking |

---

## 6. Low-Profile Under-Jersey Mechanical & Environmental Packaging

- **Envelope & Mass**: **`52.0 × 36.0 × 10.5 mm`** (chamfered rounded-rectangle profile, `R3` corner fillets, `R2` chest-facing bevel), **`~24 g`** (`<= 28 g` max) — thinner and lighter than a standard hockey puck (`76 × 25 mm`, `170 g`).
- **Construction**: Rigid `PC+ABS` internal chassis encapsulated in Shore `70A` `TPU` impact-absorbing overmold; ultrasonic weld or laser-welded perimeter seal achieving **`IP67`** (sweat, ice melt, locker-room washdown resistant).
- **Barometer Port**: Recessed `BMP581` acoustic/pressure port protected by an `IP67` oleophobic/hydrophobic **`ePTFE` membrane** (`Gore` / `Donaldson` vent) preventing sweat or melted ice blockage.
- **Retention**: Dual-compatible with (a) sewn sternum pocket inside compression base-layer shirt (recommended; minimizes garment bounce) or (b) `25 mm` flat elastic sternum chest strap with anti-slip silicone grip under shoulder pads and jersey. Zero sharp edges or exposed metal fasteners.
- **User Interface**: Flush single tactile dome button (`SHPHLD` wake/start/stop/event mark) + recessed dual-color status LED + IP67 waterproof `USB-C` receptacle with captive TPU plug.

---

## 7. Cold-Rated (`0 °C` + Aging) Battery & Power Budget

### 7.1 Derating & Ohmic Voltage Sag Analysis (`502535` 450 mAh LiPo)
- **Usable Capacity**: Nominal `450 mAh` (`1.665 Wh` at `3.7 V`). Applying conservative additive derating of **`-20%` cold (`0 °C`)** and **`-15%` cycle aging** leaves **`65%` usable capacity = `292.5 mAh`** (multiplicative `0.80 × 0.85 = 306 mAh`).
- **Cold Internal Resistance Check**: Even if cell $R_{\text{int}}$ triples from `120 mΩ` (`25 °C`) to `360 mΩ` (`0 °C`), worst-case active draw `3.205 mA` corresponds to **`0.0071 C` (`C/140`)**, producing only $3.205\text{ mA} \times 0.36\ \Omega = \mathbf{1.15\text{ mV}}$ ohmic sag—eliminating premature cold brownout.
- **Buck Conversion**: $I_{\text{batt}} = I_{1.8\text{V}} \times \frac{1.8\text{ V}}{3.7\text{ V} \times 0.90} = I_{1.8\text{V}} \times 0.54054$.

### 7.2 Mode-by-Mode Power Table
| Operating Mode | Subsystem Breakdown (`@ 1.8 V`) | Total `1.8 V` Current | Battery Current (`@ 3.7 V, 90% eff`) | Power (`mW`) | Runtime / Energy (`292.5 mAh` Derated) |
|---|---|---|---|---|---|
| **1. Nominal Play Logging** | `nRF5340` 20% duty+DMA (`0.950 mA`) + `ICM-42688-P` (`0.880 mA`) + `ADXL372` 3200 Hz (`0.022 mA`) + `MMC5983MA` 50 Hz (`0.350 mA`) + `BMP581` 25 Hz (`0.045 mA`) + `W25N01JW` write avg (`0.050 mA`) + misc (`0.013 mA`) | **`2.310 mA`** | **`1.249 mA`** | `4.62 mW` | **`234.3 h`** (`117×` 2h min) |
| **1b. Worst-Case Logging** | `nRF5340` 100% @ 64 MHz+DMA (`4.200 mA`) + `ICM` (`0.880 mA`) + `ADXL372` (`0.022 mA`) + `MMC5983MA` max (`0.450 mA`) + `BMP581` high-OSR (`0.200 mA`) + `W25N01JW` (`0.150 mA`) + LED (`0.028 mA`) | **`5.930 mA`** | **`3.205 mA`** | `11.86 mW` | **`91.3 h`** (**`45.6×`** 2h min) |
| **2. Play + 1 Hz BLE Beacon** | Mode 1 + Net Core 1 Hz adv (`0.080 mA`) | `2.390 mA` | `1.292 mA` | `4.78 mW` | `226.4 h` |
| **3. BLE 5.3 2M Bulk Offload** | App Core (`3.5 mA`) + Net Core TX (`6.5 mA`) + NAND Read (`4.0 mA`) | `14.000 mA` | `7.568 mA` | `28.00 mW` | 2h log (`38.71 MB` @ `125 KB/s` = `309.6 s` / `5.16 min`) uses **`0.651 mAh`** (`0.22%`) |
| **4. USB FS Bulk Offload** | `USBD` + NAND Read + App Core | Powered by `VBUS` | **`0 mA` net** *(charges up to `225 mA` / `0.5 C` JEITA `0–45 °C`)* | `VBUS` | 2h log (`38.71 MB` @ `650 KB/s` = **`59.5 s`**) |
| **5a. Wake-on-Motion Standby** | `ADXL372` (`1.4 µA`) + `ICM` WoM (`40 µA`) + `nRF5340` RTC (`1.7 µA`) + `W25N01JW` (`10 µA`) | `53.1 µA` | `~35 µA` | `0.13 mW` | `> 8,300 h` |
| **5b. Ship Mode** | `nPM1300` Ship Mode active (`370 nA` typ / `~1.0 µA` max incl. cell leakage) | `0 µA` | `~1.0 µA` | `0.004 mW` | `> 2 yr` shelf life |

---

## 8. Flash Ring Storage & Post-Session Download Specification

### 8.1 `W25N01JW` Partition Map & Capacity
- **Geometry**: `1,024` blocks × `64` pages/block × `2,048 B` main (+ `64 B` ECC spare) = `134,217,728 B` (`128.00 MiB`).
- **Partitions (`128 KiB` Block-Aligned)**:
  - Blocks `0..1` (`256 KiB`): `FACTORY_CAL` (redundant Slot A/B `0x06 CALIBRATION_BLOCK` + device UUID/rev + CRC32).
  - Blocks `2..5` (`512 KiB`): `SESSION_INDEX` (wear-leveled ring journal of session metadata).
  - Blocks `6..1003` (**`998` guaranteed usable blocks = `130,809,856 B` / `124.75 MiB`**): `TELEMETRY_RING` (`20` blocks in `1004..1023` reserved for factory + lifetime BBM bad-block replacement).

### 8.2 Exact Reconciled `.hks` Frame Rates & Capacity
Every `.hks` frame has a **`20 B` packed little-endian header** (`sync=0xAA55`, `msg_type`, `flags`, `payload_len`, `seq`, `uint64_t timestamp_us`, `reserved=0`) + **`N B` payload** + **`4 B` `CRC32-IEEE` (`0xEDB88320`)**:
1. **`0x02 IMU_BATCH`** (`200 Hz`, `N=20` samples/frame @ `10 Hz` frame rate): `20 + (8 + 20×12 = 248) + 4` = **`272 B` frame** × `10 Hz` = **`2,720.00 B/s`**.
2. **`0x04 AUX_ENV`** (`50 Hz` frame rate, carrying `50 Hz` Mag, `25 Hz`-updated Baro, `1 Hz`-updated PMIC): `20 + 28 + 4` = **`52 B` frame** × `50 Hz` = **`2,600.00 B/s`**.
   - **Base Continuous Rate**: `2,720 + 2,600` = **`5,320.00 B/s`**.
3. **`0x03 HIGH_G_BATCH`** (`ADXL372`, `8 + 6×N B` payload):
   - **Nominal (3200 Hz Impact Burst `> 12 g`)**: `N=512` samples (`160 ms`) → `3,080 B` payload / **`3,104 B` frame**; at `60` impacts/hr = **`51.73 B/s`**.
   - **Continuous 400 Hz Mode**: `N=40` samples/frame @ `10 Hz` (`272 B` frame × `10 Hz` = **`2,720.00 B/s`**) or `N=50` @ `8 Hz` (`332 B` frame × `8 Hz` = **`2,656.00 B/s`**).
4. **`0x05 EVENT_MARKER`** (`16 B` payload → **`40 B` frame**, avg `0.1 Hz`): **`4.00 B/s`**.
- **Totals in `130,809,856 B` (`998` Usable Blocks)**:
  - **Nominal (3200 Hz Impact Burst)**: **`5,375.73 B/s`** (`≈ 5,376 B/s`) = **`19.35 MB/hr`** → **`6.76 hours`** continuous retention (`2 h` session = `38.71 MB`).
  - **Continuous 400 Hz High-g**: `8,044.00 B/s` (`N=40 @ 10 Hz` → **`4.52 hours`**) / `7,980.00 B/s` (`N=50 @ 8 Hz` → **`4.55 hours`**, `~8,000 B/s` = **`4.54 hours`**).

### 8.3 Post-Session Download Workflow & Operational Lifecycle
1. **Pre-Session**: Wear pod in compression sternum pocket or 25 mm chest strap; press button (`SHPHLD`) to wake from Ship/Standby mode (solid green LED = ready, UTC synced via host/BLE if available).
2. **On-Ice Recording**: Firmware logs `0x01 SESSION_HEADER` + `0x06 CALIBRATION_BLOCK`, streams `0x02`, `0x04`, burst `0x03`, and `0x05` markers into `TELEMETRY_RING`, and commits `0x07 SESSION_FOOTER` (with `pre_footer_sha256[32]`) on button stop, USB dock, or low-battery cutoff.
3. **Post-Session Offload**: Connect USB-C (`CDC-ACM`, `~650 KB/s`, `~59.5 s` for 2h log while simultaneously recharging) or pair over BLE 5.3 `2M PHY` (`~125 KB/s`, `~5.16 min`, `0.22%` battery). Host executes length-prefixed `CRC32-IEEE` commands (`0xAA55` request sync / `0x55AA` response sync): `LIST_SESSIONS` (`0x01`) → `GET_HEADER` (`0x02`) → `READ_CHUNK` (`0x03`, up to `4096 B`) → `ACK_VERIFY` (`0x04`, CRC32 + SHA-256). Device refuses `DELETE_SESSION` (`0x05`, `force_flag=0`) with `0x05 ERR_NOT_VERIFIED` until `ACK_VERIFY` succeeds.
4. **Post-Processing**: Host decodes `.hks` frames, validates all frame CRCs and session SHA-256, executes the 6-layer ES-EKF + RTS smoother, and emits contract-valid `analytics_report` JSON.
