# `hockey_sensor` — KiCad 8 Project & Turnkey PCBA Files

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
   - Route the fanout tracks using the pre-assigned net classes (`0.09–0.12 mm` for `aQFN-94` signal breakout, `0.22 mm / 0.15 mm` differential pair for `USB_D_P`/`USB_D_N`, `0.32 mm` `50 Ω` GCPW for `RF_ANT_OUT` $	o$ `C_RF1`/`L5`/`L6` $	o$ `A1`).
3. Run **Inspect $	o$ Design Rules Checker (DRC)** and export Gerbers + Drill files via **File $	o$ Fabrication Outputs**.
