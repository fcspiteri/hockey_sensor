# `hockey_sensor` — System Architecture & Interface Design Reference

**Contract Authority:** [`../INTERFACE_CONTRACT.json`](../INTERFACE_CONTRACT.json) (`v1.0.0`)  
**Specification:** [`../PROJECT_SPEC.md`](../PROJECT_SPEC.md)  

This document provides the complete, implementation-ready architectural specification consumed by downstream milestones **M2 (`hardware/`)**, **M3 (`firmware/`)**, **M4 (`sim/`)**, and **E2E (`e2e/`)** for the unified `1.8 V` **`nRF5340-QKAA`** + **`nPM1300`** (`0x6B`) wearable platform.

---

## 1. End-to-End System Topology

```text
+---------------------------------------------------------------------------------------------------+
| CHEST-WORN POD (52 x 36 x 10.5 mm, ~24 g, IP67 PC+ABS + 70A TPU, 1.8 V Unified Rail)              |
|                                                                                                   |
|  [502535 450mAh LiPo + 10k NTC] ---> [nPM1300 PMIC/FG] --- TWIM1 (400kHz, 0x6B) ---+              |
|                                            | 1.8V Buck1                            |              |
|  [ICM-42688-P +-16g/2000dps @200Hz] --- SPIM4 (16MHz) -----------------------+     |              |
|  [ADXL372 +-200g @3200Hz burst/400Hz] - SPIM1 (8MHz, isolated <=10MHz) -----+|     |              |
|  [MMC5983MA +-800uT @50Hz (0x30)] ---+                                      ||     |              |
|  [BMP581 Baro @25Hz (0x47, ePTFE)] --+-- TWIM0 (400kHz) --------------------++-----+-> [nRF5340]  |
|  [W25N01JW 128MiB SLC NAND] <----------- QSPI (48-96MHz) -----------------------------|  QKAA    |
|  [USB-C IP67 + TPD4E05U06 ESD] <-------- USBD FS 12Mbps CDC-ACM -----------------------| Dual-M33 |
|  [2450AT18B100 2.4GHz Ant] <------------ BLE 5.3 2M PHY (Net Core) -------------------+          |
+---------------------------------------------------------------------------------------------------+
                                            |
                      Post-Session USB CDC-ACM (Primary) / BLE 2M (Secondary)
                                            v
+---------------------------------------------------------------------------------------------------+
| HOST DOWNLOADER & FUSION PIPELINE (Python 3.13, sim/)                                             |
|  1. Downloader (LIST_SESSIONS -> GET_HEADER -> READ_CHUNK -> ACK_VERIFY -> DELETE_SESSION)        |
|  2. .hks Binary Codec (CRC32-IEEE + SHA-256 verification, C11 <-> Python struct parity)           |
|  3. 6-Layer Anchor-Free Fusion (Dual-IMU handoff -> 15-state ES-EKF -> ZUPT/ZARU -> NHC/Stride    |
|     -> Rink Clamping & Bench-Gate Reset -> Mag Gating & Backward RTS Smoother)                    |
|  4. Output: Contract-Validated analytics_report JSON (Speed, Accel, 2D Pos, Shift Len, Distance)  |
+---------------------------------------------------------------------------------------------------+
```

---

## 2. Coordinate Frames, Transformations & Calibration Math

All coordinate frames are **right-handed** ($\hat{X} \times \hat{Y} = \hat{Z}$):

1. **World / Rink Frame (`W`)**:
   - Origin `(0, 0, 0)` at center-ice faceoff dot on ice plane $z = 0$.
   - `+X_W` toward attacking goal, `+Y_W` toward left boards, `+Z_W` vertically upward.
   - Gravity vector: $\mathbf{g}^W = [0,\ 0,\ -9.80665]^T\text{ m/s}^2$; stationary specific force $\mathbf{f}^W = [0,\ 0,\ +9.80665]^T\text{ m/s}^2$.
2. **Player Body / Torso Frame (`B`, FLU)**:
   - Origin at sternum pod geometric center.
   - `+X_B` forward out of sternum, `+Y_B` player left, `+Z_B` upward along spine.
3. **Sensor Chip Frame (`S`, Canonical Vertical Sternum PCB)**:
   - `+X_S` up along PCB (`+Z_B`), `+Y_S` player right (`-Y_B`), `+Z_S` outward normal away from chest (`+X_B`).
   - Proper orthogonal matrix $R_{BS} \in SO(3)$ ($\|R_{BS}^T R_{BS} - I\|_F \le 10^{-5}$, $\det(R_{BS}) = +1$):
     $$R_{BS} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$$
4. **Quaternion & Specific-Force Kinematics**:
   - Hamilton scalar-first $q_{WB} = [q_w, q_x, q_y, q_z]^T$, $\|q_{WB}\|_2 = 1 \pm 10^{-6}$, active Body→World $\mathbf{v}^W = R(q_{WB})\mathbf{v}^B$:
     $$R(q) = \begin{bmatrix} 1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y) \\ 2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x) \\ 2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2) \end{bmatrix}$$
   - World linear acceleration: $\mathbf{a}^W = R(q_{WB})\mathbf{f}^B + \mathbf{g}^W$.
   - Discrete attitude update: $q_{WB}(t + \Delta t) = q_{WB}(t) \otimes \exp\!\left(\frac{1}{2}\boldsymbol{\omega}^B \Delta t\right)$.

---

## 3. Packed Binary `.hks` Wire Format (`0x01` – `0x07`)

All `.hks` flash frames and USB CDC-ACM binary envelopes are **little-endian**, `__attribute__((packed))`, with **zero implicit padding**.

### 3.1 Common Frame Envelope (`24 + N` Bytes Total, `0 <= N <= 4072`)
- **CRC32-IEEE**: Polynomial `0x04C11DB7` (reflected `0xEDB88320`), `init = 0xFFFFFFFF`, `refin = true`, `refout = true`, `xorout = 0xFFFFFFFF` (`CRC32("123456789") == 0xCBF43926`). Computed over `header[0..19] || payload[0..payload_len-1]`.

| Offset | Size | C11 Type | Field | Semantics |
|---|---|---|---|---|
| `0` | `2` | `uint16_t` | `sync` | Constant `0xAA55` (`0x55 0xAA` on wire) |
| `2` | `1` | `uint8_t` | `msg_type` | `0x01..0x07` |
| `3` | `1` | `uint8_t` | `flags` | `b0`: accel_sat, `b1`: gyro_sat, `b2`: mag_anom, `b3`: low_batt, `b4`: high_g_active, `b5`: utc_synced |
| `4` | `2` | `uint16_t` | `payload_len` | `N` (`0..4072`) |
| `6` | `2` | `uint16_t` | `seq` | Rolling frame sequence counter |
| `8` | `8` | `uint64_t` | `timestamp_us` | Monotonic microseconds (`1e-6 s/LSB`, immune to 71.5-min `uint32_t` rollover) |
| `16` | `4` | `uint32_t` | `reserved` | `0x00000000` (4-byte aligns payload) |
| `20` | `N` | `uint8_t[N]` | `payload` | Message-specific packed payload |
| `20+N` | `4` | `uint32_t` | `crc32_ieee` | IEEE 802.3 CRC32 over `[0 .. 20+N-1]` |

### 3.2 Payload Byte Layouts (`struct.calcsize` & C11 `_Static_assert` Locked)

#### `0x01` `flash_session_header` (`80 B` payload → `104 B` frame, `<4sHHIIQQQHHHHffffBBHffI`)
| Offset | Size | C11 Type | Field | Scale / Notes |
|---|---|---|---|---|
| `0` | `4` | `char[4]` | `magic` | `"HKS1"` |
| `4` | `2` | `uint16_t` | `format_version` | `1` |
| `6` | `2` | `uint16_t` | `hw_rev` | `0x0100` (v1.0) |
| `8` | `4` | `uint32_t` | `fw_version` | `0xMMmmppbb` |
| `12` | `4` | `uint32_t` | `cal_crc32` | CRC32 of active `0x06 calibration_block` payload |
| `16` | `8` | `uint64_t` | `session_id` | Unique 64-bit session ID |
| `24` | `8` | `uint64_t` | `start_utc_ms` | Unix epoch ms (`0` if unsynced) |
| `32` | `8` | `uint64_t` | `device_id` | Nordic FICR 64-bit hardware ID |
| `40` | `2` | `uint16_t` | `imu_odr_hz` | Nominal `200` |
| `42` | `2` | `uint16_t` | `high_g_odr_hz` | Nominal `3200` burst or `400` cont (`0` if off) |
| `44` | `2` | `uint16_t` | `mag_odr_hz` | Nominal `50` |
| `46` | `2` | `uint16_t` | `baro_odr_hz` | Nominal `25` |
| `48` | `4` | `float` | `accel_scale_mps2_per_lsb` | `9.80665 / 2048.0 = 0.0047884033203125` |
| `52` | `4` | `float` | `gyro_scale_rads_per_lsb` | `(pi / 180.0) / 16.4 = 0.001064225153655079` |
| `56` | `4` | `float` | `high_g_scale_mps2_per_lsb` | `0.1 * 9.80665 = 0.980665` |
| `60` | `4` | `float` | `mag_scale_ut_per_lsb` | `100.0 / 16384.0 = 0.006103515625` |
| `64` | `1` | `uint8_t` | `rink_preset` | `0=NHL_200X85FT, 1=IIHF_60X30M, 2=CUSTOM` |
| `65` | `1` | `uint8_t` | `mount_location` | `0=STERNUM_POCKET, 1=CHEST_STRAP` |
| `66` | `2` | `uint16_t` | `player_height_cm` | `1 cm/LSB` (`0` if unspecified) |
| `68` | `4` | `float` | `bench_gate_x_m` | Bench gate X (`W`) |
| `72` | `4` | `float` | `bench_gate_y_m` | Bench gate Y (`W`) |
| `76` | `4` | `uint32_t` | `reserved` | `0` |

#### `0x02` `imu_batch_packet` (`8 + 12*N B` payload; nominal `N=20` → `248 B` payload, `272 B` frame, `<HHhH + N*<hhhhhh`)
- `sample_count: u16` (`1..338`), `odr_hz: u16`, `temp_c_x100: i16`, `reserved: u16` (`8 B`), followed by `N` × `{ax, ay, az, gx, gy, gz}` (`int16_t` in `S`). Sample `k` time = `timestamp_us + k * (1_000_000.0 / odr_hz)`.

#### `0x03` `high_g_batch_packet` (`8 + 6*N B` payload; e.g., `N=32` → `200 B` payload, `224 B` frame, `<HHBBH + N*<hhh`)
- `sample_count: u16` (`1..677`), `odr_hz: u16` (`400|800|1600|3200`), `trigger_mode: u8` (`0=CONTINUOUS, 1=IMPACT_BURST`), `reserved0: u8`, `reserved1: u16` (`8 B`), followed by `N` × `{ax, ay, az}` (`int16_t` in `S`).
- **ADXL372 Bit-Packing Rule**: Hardware registers/FIFO are 12-bit left-justified (`bits[15:4]`). Firmware MUST arithmetic-right-shift by 4 on SPI read (`(int16_t)raw_u16 >> 4`), emitting right-aligned sign-extended `int16_t` in `[-2048, +2047]` at **`0.980665 (m/s^2)/LSB`**. Host codecs MUST NOT shift by 4.

#### `0x04` `aux_env_packet` (`28 B` payload → `52 B` frame at `50 Hz`, `<iiiIhHhHHH`)
- `mx_raw, my_raw, mz_raw: int32_t` (firmware-centered `raw_u18 - 131072` in `[-131072, +131071]`, `0.006103515625 uT/LSB`), `pressure_pa_x64: uint32_t` (`0.015625 Pa/LSB`), `baro_temp_c_x100: int16_t`, `vbatt_mv: uint16_t`, `batt_temp_c_x100: int16_t`, `soc_pct_x10: uint16_t` (`0.1 %/LSB`), `pmic_status: uint16_t` (primary `nPM1300` status flags in bits `[7:0]`), `validity_flags: uint16_t` (`b0`: mag_fresh, `b1`: baro_fresh, `b2`: pmic_fresh, `b3`: mag_set_reset).

#### `0x05` `event_marker_packet` (`16 B` payload → `40 B` frame, `<HHfQ`)
- `event_code: uint16_t` (`1=SESSION_START, 2=BUTTON_MARK, 3=IMPACT_SPIKE, 4=SHIFT_HINT_ON, 5=SHIFT_HINT_OFF, 6=LOW_BATT, 7=BOARD_COLLISION, 8=SESSION_END`), `severity: uint16_t` (`0=INFO..3=CRITICAL`), `value_f32: float`, `detail_u64: uint64_t`.

#### `0x06` `calibration_block` (`288 B` payload → `312 B` frame, `<HHQ9f9f3f9f3f9f9f3f3f9fffI`)
- `cal_version: u16` (`2 B`), `flags: u16` (`2 B`), `cal_utc_ms: u64` (`8 B`), `sensor_to_body_rotation_matrix: float[9]` (`36 B`), `accel_misalign_scale: float[9]` (`36 B`), `accel_bias_mps2: float[3]` (`12 B`), `gyro_misalign_scale: float[9]` (`36 B`), `gyro_bias_rads: float[3]` (`12 B`), `high_g_to_body_rotation_matrix: float[9]` (`36 B`), `high_g_misalign_scale: float[9]` (`36 B`), `high_g_bias_mps2: float[3]` (`12 B`), `mag_hard_iron_ut: float[3]` (`12 B`), `mag_soft_iron_matrix: float[9]` (`36 B`), `baseline_mag_norm_ut: float` (`4 B`), `baseline_mag_dip_deg: float` (`4 B`), `cal_crc32: u32` (`4 B`, CRC32 over bytes `[0..283]`).

#### `0x07` `flash_session_footer` (`72 B` payload → `96 B` frame, `<QIIIIIIQ32s`)
- `end_utc_ms: u64` (`8 B`), `total_imu_samples: u32` (`4 B`), `dropped_imu_samples: u32` (`4 B`), `total_high_g_samples: u32` (`4 B`), `dropped_high_g_samples: u32` (`4 B`), `total_aux_samples: u32` (`4 B`), `end_reason: u32` (`0=BUTTON_STOP, 1=USB_DOCK, 2=LOW_BATT, 3=FLASH_FULL, 4=WATCHDOG_RECOVER`, `4 B`), `total_bytes: u64` (`8 B`), `pre_footer_sha256: uint8_t[32]` (`32 B`).

---

## 4. USB CDC-ACM / BLE Download Protocol

- **Command Envelope (`10 + N B`)**: `sync=0xAA55 (u16)` | `cmd_id (u8)` | `seq (u8)` | `payload_len (u16)` | `payload[N]` | `crc32_ieee (u32)`.
- **Response Envelope (`12 + N B`)**: `sync=0x55AA (u16)` | `cmd_id (u8)` | `status (u8)` | `seq (u8)` | `reserved=0 (u8)` | `payload_len (u16)` | `payload[N]` | `crc32_ieee (u32)`.

| `cmd_id` | Command | Request Payload | Response Payload (`status == 0x00 OK`) |
|---|---|---|---|
| `0x01` | `LIST_SESSIONS` | empty (`0 B`) | `count(u16), rsv(u16)` + `count × 32 B` `{session_id(u64), start_utc_ms(u64), duration_ms(u32), byte_size(u32), sample_count(u32), flags(u32)}` |
| `0x02` | `GET_HEADER` | `session_id(u64)` (`8 B`) | Raw `0x01` header frame (`104 B`) + raw `0x06` calibration frame (`312 B`) (`416 B`; JSON `header_b64`) |
| `0x03` | `READ_CHUNK` | `session_id(u64), offset_bytes(u32), length_bytes(u16 <= 4096)` (`14 B`) | `offset_bytes(u32), actual_len(u16), data[actual_len]` |
| `0x04` | `ACK_VERIFY` | `session_id(u64), computed_file_crc32(u32), computed_sha256(u8[32])` (`44 B`) | `verified_status(u8)` (`1=OK`), `hash_mode(u8)` (`1=CRC32, 2=CRC32+SHA256`) |
| `0x05` | `DELETE_SESSION` | `session_id(u64), force_flag(u8)` (`9 B`) | `freed_bytes(u32)`; fails `0x05 ERR_NOT_VERIFIED` if `force_flag==0` & unverified |
| `0x06` | `GET_DEVICE_STATUS` | empty (`0 B`) | `fw_version(u32), hw_rev(u16), vbatt_mv(u16), soc_pct_x10(u16), recording_state(u8), pmic_status(u8, bits [7:0]), flash_free_kb(u32), utc_ms(u64)` (`24 B`) |
| `0x07` | `SET_UTC_TIME` | `utc_ms(u64)` (`8 B`) | `utc_ms(u64)` (`8 B`, echoed accepted UTC epoch ms) |

**Status Codes**: `0x00 OK`, `0x01 ERR_BAD_CRC`, `0x02 ERR_UNKNOWN_CMD`, `0x03 ERR_SESSION_NOT_FOUND`, `0x04 ERR_OFFSET_OUT_OF_RANGE`, `0x05 ERR_NOT_VERIFIED`, `0x06 ERR_FLASH_IO`, `0x07 ERR_BUSY_RECORDING`, `0x08 ERR_VERIFY_MISMATCH`, `0x09 ERR_INVALID_PARAM`.

---

## 5. Flash Ring Buffer & Cold Power Budget Summary

- **`W25N01JW` (`998` usable `128 KiB` blocks = `130,809,856 B`)**:
  - `272 B @ 10 Hz` (`0x02`) + `52 B @ 50 Hz` (`0x04`) = `5,320 B/s` base + `ADXL372` burst (`3,104 B @ 60/hr = 51.73 B/s`) + `0x05` events (`4.00 B/s`) = **`5,375.73 B/s`** (`≈ 5,376 B/s` = **`19.35 MB/hr`**) → **`6.76 h`** nominal retention.
  - Continuous `400 Hz` `ADXL372` (`N=40 @ 10 Hz` = `2,720 B/s` → `8,044 B/s` = `4.52 h`; `N=50 @ 8 Hz` = `7,980 B/s` = `4.55 h`, `~8,000 B/s` = `4.54 h`).
- **`502535` 450 mAh LiPo (`0 °C` -20% + aging -15% = `292.5 mAh` usable)**:
  - Nominal logging: `2.310 mA @ 1.8 V` (`1.249 mA` batt) → **`234.3 h`**.
  - Worst-case logging: `5.930 mA @ 1.8 V` (`3.205 mA` batt, `C/140`, `1.15 mV` cold sag @ `360 mΩ`) → **`91.3 h`**.
  - BLE 2M offload (`38.71 MB` 2h session @ `125 KB/s` = `309.6 s`): `0.651 mAh` (`0.22%` of derated capacity); USB FS (`650 KB/s` = `59.5 s`): `0 mAh` net battery drain.

---

## 6. 15-State Error-State EKF & RTS Smoother Pipeline

- **Nominal State**: $\mathbf{x} = [\mathbf{p}^W,\ \mathbf{v}^W,\ q_{WB},\ \mathbf{b}_a^S,\ \mathbf{b}_g^S]^T \in \mathbb{R}^{16}$.
- **Error State (15-DOF)**: $\delta\mathbf{x} = [\delta\mathbf{p}^W,\ \delta\mathbf{v}^W,\ \delta\boldsymbol{\theta}^W,\ \delta\mathbf{b}_a^S,\ \delta\mathbf{b}_g^S]^T \in \mathbb{R}^{15}$.
- **Dual-IMU Handoff**: Any sample with $\|\mathbf{a}_{\text{ICM}}\|_\infty > 14.5\text{ g}$ switches seamlessly to `ADXL372` (`±200 g`) transformed by $R_{BS,hg}$ so impact $\Delta\mathbf{v}$ is never clipped.
- **Measurement Updates**:
  1. **ZUPT / ZARU** (stationary $\ge 0.4\text{ s}$): $\mathbf{v}^W = \mathbf{0}$ ($\sigma_v = 0.05\text{ m/s}$) + gyro bias update.
  2. **Non-Holonomic Constraints (NHC)**: $v_z^W = 0$ ($\sigma_{v_z} = 0.15\text{ m/s}$); soft lateral $v_y^B = 0$ ($\sigma_{v_y} = 0.5\text{–}1.2\text{ m/s}$, relaxed during sharp turns / stops).
  3. **Stride-Cadence Speed Coupling**: During `STRIDING` ($f_{\text{stride}} \in [0.8, 2.5]\text{ Hz}$), $v_{\text{bio}} = \alpha f_{\text{stride}} + \beta\sqrt{E_{\text{stride}}} + \gamma$ ($\sigma = 0.7\text{–}1.0\text{ m/s}$).
  4. **Adaptive Mag Gating**: Accept `MMC5983MA` yaw only if $\|\mathbf{B}\| \in [0.88, 1.12]\|\mathbf{B}_0\|$ and $|\Delta\phi_{\text{dip}}| < 8^\circ$.
  5. **Rink Containment & Bench-Gate Reset**: Clamp outside rounded rectangle + zero outward normal $v_n$; apply soft bench-gate coordinate prior $(x_{\text{bench}}, y_{\text{bench}})$ ($\sigma_{xy} = 2.5\text{ m}$) on `BENCH ↔ ON_ICE` transitions.
  6. **Backward RTS Pass**: Smooths state & covariance across each shift interval $[t_{\text{shift\_start}}, t_{\text{shift\_end}}]$ between bench-gate anchors, reducing mid-shift position error to **`2.5–6.0 m` CEP50** per 45 s shift (with `> 10–25 m` drift caveat if skated continuously `> 2 min` without resets).

---

## 7. Downstream Verification Matrix

| Milestone | Verification Gate | Target Output |
|---|---|---|
| **M1 (Current)** | `python3 docs/validate_contract.py` & `python3 -m unittest discover -s docs -p "validate_*.py" -v` | 16/16 schema, struct size, `SO(3)`, `q_WB`, rink containment & bus checks PASS |
| **M2 (Hardware)** | `python3 -m unittest discover -s hardware/tests -v` | Cold `0 °C` battery `>= 2 h`, `W25N01JW` flash `>= 2 h`, `52×36×10.5 mm`, bus isolation PASS |
| **M3 (Firmware)** | `make -C firmware test` (`clang -std=c11 -Wall -Wextra -Werror`) | C11 `_Static_assert` sizes match `INTERFACE_CONTRACT.json` + `.hks` generator CLI PASS |
| **M4 (Host/Sim)** | `python3 -m unittest discover -s sim/tests -v` | C↔Python `.hks` interop, downloader state machine, 6-layer ES-EKF+RTS & contract-valid `analytics_report` PASS |
