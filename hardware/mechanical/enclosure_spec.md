# `hockey_sensor` — Low-Profile Under-Jersey Mechanical & Environmental Enclosure Specification

**Document ID:** `HKS-ME-1.0.0`  
**Contract Authority:** [`../../INTERFACE_CONTRACT.json`](../../INTERFACE_CONTRACT.json) (`v1.0.0`)  
**System Specification:** [`../../PROJECT_SPEC.md`](../../PROJECT_SPEC.md)  
**Dimensional JSON:** [`dimensions.json`](./dimensions.json)  
**Mechanical BOM:** [`bom.csv`](./bom.csv)

---

## 1. Ergonomic Envelope & Under-Jersey Packaging

The `hockey_sensor` pod is engineered to sit unobtrusively over the player's sternum under standard hockey shoulder pads and jersey, eliminating rib/sternum pressure points during board collisions and body checks:
- **Outer Envelope**: **`52.0 × 36.0 × 10.5 mm`** (`±0.15 mm`), well below the `55.0 × 40.0 × 12.0 mm` hard design ceiling and thinner/lighter than a regulation hockey puck (`76 × 25 mm`, `170 g`).
- **Profile & Edge Relief**: **`R3.0 mm`** rounded planform corner fillets and **`R2.0 mm`** chest-facing perimeter bevel prevent garment snagging and soft-tissue bruising under compression.
- **Mass Budget**:
  - **Pod Assembly (`pod_battery + pod_ee + pod_me`)**: **`24.0 g`** (`9.2 g` LiPo battery + `3.8 g` populated PCBA + `11.0 g` enclosure/seals/fasteners), meeting the `24.0 g` contract nominal and **`<= 28.0 g`** spec ceiling.
  - **Optional Chest Strap (`accessory_strap`)**: **`14.5 g`** (`25 mm` flat elastic strap with anti-slip silicone grip and low-profile acetal buckle), yielding **`38.5 g`** total worn mass (`<= 40.0 g` contract hard ceiling).

---

## 2. Dual-Material Construction & IP67 Environmental Sealing

Hockey equipment is exposed to continuous sweat saturation, ice-melt spray, and post-game locker-room washdowns. The pod achieves **`IP67`** (`1.0 m` immersion for `30 min`, dust-tight) through a multi-barrier architecture:

1. **Rigid Inner Structural Shell (`PC+ABS`, `4.2 g`)**:
   - Two-piece injection-molded polycarbonate/ABS chassis (`E_flex ≈ 2.3 GPa`) ultrasonically or laser-welded along a tongue-and-groove perimeter shear joint.
   - Provides structural crush resistance during high-energy checks and isolates the `0.80 mm` 4-layer FR-4 PCB from housing deflection.
2. **Perimeter Silicone O-Ring (`0.4 g`)**:
   - Secondary compression elastomer O-ring seated inside the inner shell gland for redundant moisture protection around the weld seam.
3. **Shock-Absorbing Outer Overmold (Shore `70A` `TPU`, `4.6 g`)**:
   - Low-durometer thermoplastic polyurethane overmolded over the `PC+ABS` core (`0.70–0.80 mm` Z faces, `1.50 mm` X/Y bumpers).
   - Seals the tactile `SHPHLD` dome button (`SW1`), encapsulates the flush translucent light-pipe LED window (`LD1/LD2`), and eliminates all exposed sharp edges or metal fasteners against the sternum.
4. **Waterproof `USB-C` Port (`GCT USB4720-03-A` + Captive TPU Cap, `0.6 g`)**:
   - Mid-mount IP67 `USB-C` receptacle with pre-installed LIM silicone interfacial gasket mated to an internal flange, backed by a tethered/captive Shore `70A` `TPU` dust/sweat plug during on-ice play.
5. **Barometer Hydrophobic/Oleophobic `ePTFE` Vent (`0.1 g`)**:
   - Recessed `2.5 mm` acoustic/pressure port centered directly above `U6` (`BMP581`) covered by an adhesive-backed `IP67` oleophobic/hydrophobic `ePTFE` membrane (`Gore` / `Donaldson`), allowing rapid barometric pressure equalization (`< 0.1 Pa` RMS tracking of bench step-over elevation) while blocking liquid sweat, soap, and ice melt.

---

## 3. >200 g Impact Survivability & Battery Swell Protection

The pod is designed to survive **`> 200 g`** peak impact shocks (slapshots, board impacts, and direct body checks transmitted through shoulder pad chest plates):
- **PCB Boss Retention**: The `38.0 × 28.0 × 0.80 mm` 4-layer FR-4 PCB is rigidly anchored at four corners via `4× M1.4` stainless steel screws threaded into brass heat-set inserts molded into `PC+ABS` standoffs (`1.1 g` fasteners/pad/dome combined), keeping `ICM-42688-P` and `ADXL372` in the low-strain center zone away from mounting bosses.
- **Solder Joint Reinforcement**: Corner-stake epoxy/underfill (`MAT1`) reinforces large QFN/WSON packages (`nRF5340-QKAA`, `nPM1300-QEAA-R`, `W25N01JWZEIG`) against board-whip shear.
- **Single-Sided PCB & Poron Battery Isolation**: All EE components mount on the top (`+Z_S`) side of the PCB, leaving the bottom (`L4`) face completely flat. A **`0.35 mm` Poron / closed-cell silicone isolation pad** separates the PCB from the `502535` LiPo pouch inside a dedicated **`38.5 × 26.5 × 5.6 mm`** recessed battery pocket (formed by a `0.25 mm` stepped PCB standoff recess above the `5.35 mm` nominal cell+Poron stack), absorbing `> 200 g` shock energy and ensuring `5.40 mm` EOL swollen cell + `0.15 mm` compressed Poron = `5.55 mm < 5.60 mm` (zero pinch force).

---

## 4. Dimensional Tolerance Stackups (Z-Axis & X/Y Plane)

### 4.1 Z-Axis Thickness Stackup (Front Outward `+Z_S` to Chest-Facing `-Z_S`)
| Z-Stack Layer | Nominal (`mm`) | Plus Tol (`mm`) | Minus Tol (`mm`) | Engineering Notes |
|---|---:|---:|---:|---|
| Front Shore `70A` `TPU` overmold | `0.80` | `+0.10` | `-0.10` | Impact cushion + chamfered bezel |
| Front `PC+ABS` structural wall | `0.85` | `+0.08` | `-0.08` | Rigid face + recessed `ePTFE` vent seat |
| Front IC zone + air clearance | `1.25` | `+0.10` | `-0.10` | Tallest SMT component `1.30 mm` (`2450AT18B` in stepped pocket) / `1.06 mm` sensors / `0.60 mm` `SW1` (`EVPAWCD4A`) |
| 4-layer FR-4 PCB (`ENIG`) | `0.80` | `+0.08` | `-0.08` | Single-sided assembly, flat `L4` back |
| Poron isolation pad / swell gap | `0.35` | `+0.05` | `-0.05` | Compresses to `0.15 mm` inside `5.60 mm` pocket (`5.40 + 0.15 = 5.55 mm < 5.60 mm`, zero pinch) |
| `502535` 450 mAh LiPo pouch cell | `5.00` | `+0.40` | `-0.30` | EOL swelling allowance (`5.40 mm` max) |
| Rear `PC+ABS` backing wall | `0.75` | `+0.08` | `-0.08` | Rigid chest-side structural plate |
| Rear Shore `70A` `TPU` overmold | `0.70` | `+0.10` | `-0.10` | `R2.0 mm` chest-facing bevel + anti-slip |
| **Total Pod Thickness (Z)** | **`10.50`** | **RSS `+0.46` / WC `+0.99`** | **RSS `-0.38` / WC `-0.89`** | **RSS Max `10.96 mm`; Worst-Case `11.49 mm <= 12.0 mm`** |

### 4.2 X/Y Planform Cavity Stackup
- **Length (`X_S`, `52.0 mm`)**: `2 × (1.50 mm TPU + 1.20 mm PC+ABS + 0.80 mm internal rib) = 7.00 mm` wall budget + `45.00 mm` internal cavity, housing the `35.0 mm` battery cell (`+2.0 mm` PCM/flex wire allowance = `37.0 mm` swollen max) and `38.0 mm` PCB (`8.0 × 8.0 mm` RF keepout at `+X_S` top end).
- **Width (`Y_S`, `36.0 mm`)**: `2 × (1.50 mm TPU + 1.20 mm PC+ABS + 0.80 mm internal rib) = 7.00 mm` wall budget + `29.00 mm` internal cavity, housing the `25.0 mm` wide battery (`25.5 mm` max) and `28.0 mm` wide PCB (`0.50 mm` lateral clearance per side).

---

## 5. Dual Retention Options (`STERNUM_POCKET` & `CHEST_STRAP`)

Both retention modes defined in `INTERFACE_CONTRACT.json §schemas.flash_session_header.properties.mount_location` are supported with zero exposed metal hardware:
1. **`STERNUM_POCKET` (Recommended)**: Pod slides into a snug, moisture-wicking sewn sternum pocket integrated into a hockey compression base-layer shirt, minimizing garment bounce and keeping pod mass at `24.0 g`.
2. **`CHEST_STRAP` (`25 mm` Flat Elastic Strap)**: Two low-profile molded `TPU/PC+ABS` strap slots along the pod perimeter accept a `25 mm` wide flat elastic chest strap (`14.5 g`) lined with medical-grade anti-slip silicone ribs and a flat acetal side-release buckle positioned laterally under the arm (never over the spine or sternum).

---

## 6. Coordinate Frame `S` Orientation & Exterior Markings

To guarantee `canonical_R_BS` ($\det = +1$) alignment between the physical sensors and the host ES-EKF pipeline (`INTERFACE_CONTRACT.json §coordinate_frames.frames.S`), both the enclosure exterior and internal PCB carry permanent orientation cues:
- **`+X_S` (`+Z_B`, Up)**: Aligned with the `52.0 mm` long axis toward the player's head. Marked by a molded/laser-etched **`TOP / +X_S ↑`** arrow on the front TPU face.
- **`+Y_S` (`-Y_B`, Player Right)**: Aligned with the `36.0 mm` width axis toward the player's right when worn vertically on the sternum.
- **`+Z_S` (`+X_B`, Outward)**: Normal to the front face away from the chest (`+X_S × +Y_S = +Z_S`). Front face is embossed **`FRONT (+Z_S) — OUTWARD`**; chest-facing beveled back face is embossed **`CHEST SIDE (-Z_S)`**.
- **Rotation Matrix Verification**:
  $$R_{BS} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}, \quad R_{BS}^T R_{BS} = I_3, \quad \det(R_{BS}) = +1$$
- **Keyed Assembly**: Asymmetric internal `PC+ABS` alignment pins ensure the `38.0 × 28.0 mm` PCB can only be seated inside the chassis in the canonical Frame `S` orientation.
