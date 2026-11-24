"""Comprehensive unittest suite for hockey_sensor Milestone M2 (hardware/**)."""

from __future__ import annotations

import copy
import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

HARDWARE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = HARDWARE_DIR.parent
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

import power_budget  # noqa: E402

CONTRACT_PATH = REPO_ROOT / "INTERFACE_CONTRACT.json"
PINMUX_PATH = HARDWARE_DIR / "ee" / "pinmux_and_buses.json"
EE_BOM_PATH = HARDWARE_DIR / "ee" / "bom.csv"
SCHEMATIC_PATH = HARDWARE_DIR / "ee" / "schematic_design.md"
ME_DIMS_PATH = HARDWARE_DIR / "mechanical" / "dimensions.json"
ME_BOM_PATH = HARDWARE_DIR / "mechanical" / "bom.csv"
ENCLOSURE_SPEC_PATH = HARDWARE_DIR / "mechanical" / "enclosure_spec.md"
POWER_BUDGET_SCRIPT = HARDWARE_DIR / "power_budget.py"


def _mat3_det(m: list[list[float]]) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def _mat3_ortho_err(m: list[list[float]]) -> float:
    max_err = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(m[k][i] * m[k][j] for k in range(3))
            max_err = max(max_err, abs(dot - (1.0 if i == j else 0.0)))
    return max_err


class TestHardwareFilesAndPinmux(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = power_budget.load_json_strict(CONTRACT_PATH)
        cls.pinmux = power_budget.load_json_strict(PINMUX_PATH)
        cls.ee_rows = power_budget.load_ee_bom(EE_BOM_PATH)
        cls.me_dims = power_budget.load_json_strict(ME_DIMS_PATH)
        cls.me_rows = power_budget.load_me_bom(ME_BOM_PATH)

    def test_all_required_hardware_deliverables_exist_and_nonempty(self) -> None:
        for path in (
            CONTRACT_PATH,
            SCHEMATIC_PATH,
            PINMUX_PATH,
            EE_BOM_PATH,
            ENCLOSURE_SPEC_PATH,
            ME_DIMS_PATH,
            ME_BOM_PATH,
            POWER_BUDGET_SCRIPT,
        ):
            self.assertTrue(path.is_file(), f"Missing deliverable: {path}")
            self.assertGreater(path.stat().st_size, 0, f"Empty deliverable: {path}")

    def test_pinmux_contract_parity_and_zero_gps_anchors_uwb(self) -> None:
        hw = self.contract["hardware_abstraction"]
        inv = self.pinmux["invariants"]
        self.assertFalse(inv["uses_gps"])
        self.assertFalse(inv["uses_external_anchors"])
        self.assertFalse(inv["uses_uwb"])
        self.assertEqual(inv["system_voltage_v"], hw["system_voltage_v"])
        self.assertTrue(inv["spi_high_g_isolated_from_primary_imu"])

        self.assertEqual(self.pinmux["deployment_mode"], hw["deployment_mode"])
        self.assertEqual(self.pinmux["mcu"]["part_number"], hw["mcu"]["part_number"])
        self.assertEqual(self.pinmux["mcu"]["usb_fs"], hw["mcu"]["usb_fs"])
        self.assertEqual(self.pinmux["mcu"]["ble_phy"], hw["mcu"]["ble_phy"])

        # LFXO <= 20 ppm meets +/- 50 ppm contract timestamp tolerance
        clocks = self.pinmux["mcu"]["clocks"]
        self.assertEqual(clocks["lfxo_hz"], 32768)
        self.assertLessEqual(clocks["lfxo_tolerance_ppm"], 20)
        self.assertEqual(clocks["lfxo_pins"]["XL1"]["pin"], "P0.00")
        self.assertEqual(clocks["lfxo_pins"]["XL2"]["pin"], "P0.01")
        self.assertEqual(clocks["hfxo_mhz"], 32.0)

    def test_pinmux_zero_pin_collisions_and_dedicated_peripherals(self) -> None:
        assigned: dict[str, list[str]] = {}
        balls: dict[str, list[str]] = {}

        def _reg(sig_name: str, entry: dict[str, Any]) -> None:
            pin = entry["pin"]
            assigned.setdefault(pin, []).append(sig_name)
            if "ball" in entry:
                balls.setdefault(entry["ball"], []).append(sig_name)

        for k, v in self.pinmux["mcu"]["clocks"]["lfxo_pins"].items():
            _reg(f"lfxo.{k}", v)
        for k, v in self.pinmux["mcu"]["clocks"]["hfxo_pins"].items():
            _reg(f"hfxo.{k}", v)
        for entry in self.pinmux["mcu"]["reserved_unused_nfc_pins"]:
            _reg("reserved_nfc", entry)
        for bus_name, bus in self.pinmux["buses"].items():
            for sig, entry in bus.get("pins", {}).items():
                _reg(f"buses.{bus_name}.{sig}", entry)
        for sig, entry in self.pinmux["gpio_signals"].items():
            _reg(f"gpio.{sig}", entry)
        for sig, entry in self.pinmux["debug_interface"]["pins"].items():
            _reg(f"debug.{sig}", entry)

        pin_dups = {p: srcs for p, srcs in assigned.items() if len(srcs) > 1}
        ball_dups = {b: srcs for b, srcs in balls.items() if len(srcs) > 1}
        self.assertEqual(pin_dups, {}, f"Duplicate nRF5340 pin assignments: {pin_dups}")
        self.assertEqual(ball_dups, {}, f"Duplicate aQFN-94 ball assignments: {ball_dups}")

        buses = self.pinmux["buses"]
        # QSPI dedicated P0.13..P0.18
        qspi_pins = {v["pin"] for v in buses["QSPI"]["pins"].values()}
        self.assertEqual(qspi_pins, {"P0.13", "P0.14", "P0.15", "P0.16", "P0.17", "P0.18"})

        # SPIM4 dedicated high-speed P0.08..P0.11, isolated from SPIM1
        spim4_pins = {v["pin"] for v in buses["SPIM4"]["pins"].values()}
        self.assertTrue(spim4_pins.issubset({"P0.08", "P0.09", "P0.10", "P0.11", "P0.12"}))
        self.assertEqual(buses["SPIM4"]["clock_hz"], 16_000_000)
        self.assertEqual(buses["SPIM4"]["devices"][0]["part_number"], "ICM-42688-P")

        # ADXL372 isolated on SPIM1 <= 10 MHz
        self.assertEqual(buses["SPIM1"]["clock_hz"], 8_000_000)
        self.assertLessEqual(buses["SPIM1"]["clock_hz"], 10_000_000)
        self.assertEqual(buses["SPIM1"]["devices"][0]["part_number"], "ADXL372")

        # TWIM0 devices & addresses
        twim0_devs = {d["part_number"]: d for d in buses["TWIM0"]["devices"]}
        self.assertEqual(twim0_devs["MMC5983MA"]["i2c_addr_hex"], "0x30")
        self.assertEqual(twim0_devs["MMC5983MA"]["rail_v"], 3.0)
        self.assertEqual(twim0_devs["BMP581"]["i2c_addr_hex"], "0x47")
        self.assertEqual(buses["TWIM0"]["level_translator_ic"], "PCA9306DCUR")

        # TWIM1 key retained for contract parity, silicon_instance TWIM2 (ID 11, 0x4000B000)
        pmic_bus = buses["TWIM1"]
        self.assertEqual(pmic_bus["silicon_instance"], "TWIM2")
        self.assertEqual(pmic_bus["peripheral_id"], 11)
        self.assertEqual(pmic_bus["base_address"], "0x4000B000")
        self.assertIn("SPIM1", pmic_bus["silicon_erratum_note"])
        self.assertEqual(pmic_bus["devices"][0]["i2c_addr_hex"], "0x6B")

        # QSPI contract bus key retained; silicon_instance SPIM3 (ID 12, 0x4000C000) on P0.13..P0.18
        # Note: nRF5340 SPIM0..SPIM3 are hardware-capped at 8 MHz max SCLK
        qspi_bus = buses["QSPI"]
        self.assertEqual(qspi_bus["silicon_instance"], "SPIM3")
        self.assertEqual(qspi_bus["peripheral_id"], 12)
        self.assertEqual(qspi_bus["base_address"], "0x4000C000")
        self.assertIn("CINSTRCONF", qspi_bus["silicon_note"])
        if qspi_bus.get("silicon_instance") == "SPIM3":
            self.assertLessEqual(qspi_bus["clock_hz"], 8_000_000)
            self.assertLessEqual(qspi_bus["max_clock_hz"], 8_000_000)
            self.assertNotIn("16-32 MHz", qspi_bus["silicon_note"])
            self.assertNotIn("32MHz", qspi_bus["silicon_note"])

        # BTN_USER_N uses passive Schottky D2 isolation, not unsupported nPM1300 GPIO1 SHPHLD mirror
        btn_sig = self.pinmux["gpio_signals"]["BTN_USER_N"]
        self.assertIn("BAT54WS", btn_sig["function"])
        self.assertNotIn("nPM1300 GPIO1", btn_sig["function"])
        self.assertIn("Spare", self.pinmux["pmic_dedicated_signals"]["GPIO1"])

        # Unique active nRF5340 Peripheral IDs (no SPIM1/TWIM1 ID 9 clash)
        periph_ids = [
            buses[b]["peripheral_id"]
            for b in ("SPIM4", "SPIM1", "TWIM0", "TWIM1", "QSPI")
        ]
        self.assertEqual(len(periph_ids), len(set(periph_ids)))

        # Near-radio pins <= 10 kHz
        near_radio_pins = {"P1.01", "P1.04", "P1.05", "P1.06", "P1.07", "P1.08"}
        for bname in ("SPIM4", "SPIM1", "TWIM0", "TWIM1", "QSPI"):
            for sig, entry in buses[bname]["pins"].items():
                self.assertNotIn(
                    entry["pin"],
                    near_radio_pins,
                    f"High-speed bus {bname}.{sig} placed on near-radio pin {entry['pin']}",
                )
        for sig, entry in self.pinmux["gpio_signals"].items():
            if entry["pin"] in near_radio_pins:
                self.assertLessEqual(entry["max_freq_hz"], 10_000)
                self.assertEqual(entry["drive"], "S0S1")

        # Power rails BUCK1 = 1.8 V (47k), BUCK2 = 3.0 V (150k)
        self.assertEqual(self.pinmux["power_rails"]["BUCK1_1V8"]["r_vset_ohms"], 47000)
        self.assertEqual(self.pinmux["power_rails"]["BUCK2_3V0"]["r_vset_ohms"], 150000)
        self.assertEqual(buses["USBD"]["cc_pulldown_ohms"], 5100)


class TestBOMAndMechanicalPackaging(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = power_budget.load_json_strict(CONTRACT_PATH)
        cls.ee_rows = power_budget.load_ee_bom(EE_BOM_PATH)
        cls.me_dims = power_budget.load_json_strict(ME_DIMS_PATH)
        cls.me_rows = power_budget.load_me_bom(ME_BOM_PATH)

    def test_ee_bom_core_parts_zero_gps_and_mass_reconciliation(self) -> None:
        parts = {r["part_number"] for r in self.ee_rows}
        for required_part in (
            "nRF5340-QKAA",
            "nPM1300-QEAA-R",
            "ICM-42688-P",
            "ADXL372BCCZ-RL7",
            "MMC5983MA",
            "BMP581",
            "W25N01JWZEIG",
            "PCA9306DCUR",
            "SN74AXC1T45DRLR",
            "TPD4E05U06DQAR",
            "USB4720-03-A",
            "2450AT18B0100001E",
            "502535-450mAh-LiPo",
            "EVPAWCD4A",
            "LQM18DN100M70L",
            "BAT54WS",
            "GRM188R61E475KE11D",
            "GRM188R60J106ME47D",
            "NTZD3154NT1G",
            "LTST-C191KGKT",
            "LTST-C191KRKT",
        ):
            self.assertIn(required_part, parts, f"Missing EE BOM part: {required_part}")

        self.assertNotIn("EVQP2202M", parts)
        self.assertNotIn("LQW15AN15NG00", parts)
        self.assertNotIn("LQM18PN100MGH", parts)
        lqm_qty = sum(r["qty"] for r in self.ee_rows if r["part_number"] == "LQM18DN100M70L")
        self.assertEqual(lqm_qty, 2)

        by_ref = {r["refdes"]: r for r in self.ee_rows}
        self.assertEqual(by_ref["U7"]["bus_or_function"], "QSPI/SPIM3_8MHz")
        self.assertNotIn("SPIM3_32MHz", by_ref["U7"]["bus_or_function"])
        self.assertNotIn("16-32MHz", by_ref["U7"]["notes"])
        self.assertEqual(by_ref["L3"]["bus_or_function"], "NRF5340_VREGMAIN_DCCD")
        self.assertEqual(by_ref["L4"]["bus_or_function"], "NRF5340_VREGRADIO_DCC")
        schematic_text = SCHEMATIC_PATH.read_text(encoding="utf-8")
        self.assertNotIn("SPIM3_32MHz", schematic_text)
        self.assertNotIn("16–32 MHz", schematic_text)
        self.assertNotIn("16-32 MHz", schematic_text)
        self.assertIn("`DCCD`", schematic_text)
        self.assertIn("`DCC`", schematic_text)
        self.assertEqual(by_ref["D2"]["part_number"], "BAT54WS")
        self.assertEqual(by_ref["R_BTN_1V8_100K"]["qty"], 1)
        self.assertEqual(by_ref["R_BTN_1V8_100K"]["rail_v"], 1.8)
        self.assertEqual(by_ref["C_BUCK_10U"]["qty"], 2)
        self.assertEqual(by_ref["C_VBUS"]["part_number"], "GRM188R61E475KE11D")
        self.assertEqual(by_ref["C_VBUS"]["qty"], 1)
        self.assertIn("25V", by_ref["C_VBUS"]["notes"])
        self.assertEqual(by_ref["LD1"]["rail_v"], 3.0)
        self.assertEqual(by_ref["LD2"]["rail_v"], 3.0)
        self.assertIn("duty cycle", by_ref["LD1"]["notes"])
        self.assertIn("duty cycle", by_ref["LD2"]["notes"])
        self.assertEqual(by_ref["RT1"]["qty"], 0)
        self.assertEqual(by_ref["RT1"]["total_mass_g"], 0.0)

        # Zero GPS/anchor/UWB parts
        self.assertEqual(power_budget._detect_forbidden_rf_parts(self.ee_rows), [])

        # Check qty * unit_mass == total_mass in EE & ME BOMs
        for r in self.ee_rows:
            self.assertAlmostEqual(r["qty"] * r["unit_mass_g"], r["total_mass_g"], places=4)
        for r in self.me_rows:
            self.assertAlmostEqual(r["qty"] * r["unit_mass_g"], r["total_mass_g"], places=4)

        ee_non_batt = sum(r["total_mass_g"] for r in self.ee_rows if r["subsystem_key"] != "battery")
        ee_batt = sum(r["total_mass_g"] for r in self.ee_rows if r["subsystem_key"] == "battery")
        self.assertAlmostEqual(ee_non_batt, 3.8, places=3)
        self.assertAlmostEqual(ee_batt, 9.2, places=3)

        me_by_scope: dict[str, float] = {}
        for r in self.me_rows:
            sc = r["assembly_scope"]
            me_by_scope[sc] = me_by_scope.get(sc, 0.0) + r["total_mass_g"]
        self.assertAlmostEqual(me_by_scope["pod_ee"], ee_non_batt, places=3)
        self.assertAlmostEqual(me_by_scope["pod_battery"], ee_batt, places=3)
        self.assertAlmostEqual(me_by_scope["pod_me"], 11.0, places=3)
        pod_mass = me_by_scope["pod_ee"] + me_by_scope["pod_battery"] + me_by_scope["pod_me"]
        self.assertAlmostEqual(pod_mass, 24.0, places=3)
        self.assertLessEqual(pod_mass, 28.0)
        self.assertAlmostEqual(pod_mass + me_by_scope["accessory_strap"], 38.5, places=3)
        self.assertLessEqual(pod_mass + me_by_scope["accessory_strap"], 40.0)

    def test_mechanical_envelope_stackup_environmental_and_frame_s(self) -> None:
        hw_env = self.contract["hardware_abstraction"]["enclosure_envelope_mm"]
        env = self.me_dims["envelope_mm"]

        self.assertEqual(env["length_mm"], hw_env["length_mm"])
        self.assertEqual(env["width_mm"], hw_env["width_mm"])
        self.assertEqual(env["thickness_mm"], hw_env["thickness_mm"])
        self.assertLessEqual(env["length_mm"], 55.0)
        self.assertLessEqual(env["width_mm"], 40.0)
        self.assertLessEqual(env["thickness_mm"], 12.0)

        layers = self.me_dims["tolerance_stackup_z_mm"]["layers"]
        z_nom = sum(x["nominal_mm"] for x in layers)
        z_wc_max = sum(x["nominal_mm"] + x["plus_tol_mm"] for x in layers)
        z_rss_max = z_nom + math.sqrt(sum(x["plus_tol_mm"] ** 2 for x in layers))
        self.assertAlmostEqual(z_nom, 10.5, places=3)
        self.assertAlmostEqual(z_rss_max, self.me_dims["tolerance_stackup_z_mm"]["rss_max_mm"], places=2)
        self.assertAlmostEqual(z_wc_max, self.me_dims["tolerance_stackup_z_mm"]["worst_case_max_mm"], places=2)
        self.assertLessEqual(z_wc_max, 12.0)

        ei = self.me_dims["environmental_and_impact"]
        self.assertEqual(ei["ip_rating"], hw_env["ip_rating"])
        self.assertGreaterEqual(ei["impact_survivability_g"], 200.0)
        self.assertIn("ePTFE", ei["barometer_vent"]["membrane_material"])
        self.assertEqual(ei["barometer_vent"]["sensor"], "BMP581")

        # Frame S rotation matrix R_BS verification
        canonical_r_bs = self.contract["coordinate_frames"]["frames"]["S"]["canonical_R_BS"]
        me_r_bs = self.me_dims["coordinate_frame_markings"]["R_BS"]
        self.assertEqual(me_r_bs, canonical_r_bs)
        self.assertLess(_mat3_ortho_err(me_r_bs), 1e-6)
        self.assertAlmostEqual(_mat3_det(me_r_bs), 1.0, places=6)
        markings = self.me_dims["coordinate_frame_markings"]["markings"]
        for axis in ("+X_S", "+Y_S", "+Z_S"):
            self.assertIn(axis, markings)

        batt_comp = self.me_dims["battery_compartment_mm"]
        self.assertGreaterEqual(batt_comp["cavity_inner_mm"][2], 5.6)
        for s_val, c_val in zip(batt_comp["cell_max_swollen_mm"], batt_comp["cavity_inner_mm"]):
            self.assertLess(s_val, c_val)


class TestPowerAndStorageBudgetCalculator(unittest.TestCase):
    def test_cli_json_check_outputs_and_gates(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(POWER_BUDGET_SCRIPT), "--json", "--check"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=f"stderr={proc.stderr}")
        report = json.loads(proc.stdout, parse_constant=power_budget._reject_non_standard_json_constant)
        self.assertTrue(report["checks"]["passed"])
        self.assertEqual(report["checks"]["failures"], [])

        # Cold-derated capacity & multi-mode runtimes
        self.assertAlmostEqual(report["battery"]["usable_capacity_add_mah"], 292.5, places=3)
        self.assertAlmostEqual(report["battery"]["usable_capacity_mul_mah"], 306.0, places=3)
        self.assertGreaterEqual(report["operating_modes"]["nominal_play"]["runtime_add_h"], 2.0)
        self.assertGreaterEqual(report["operating_modes"]["worst_case_logging"]["runtime_add_h"], 2.0)
        self.assertGreater(report["operating_modes"]["nominal_play"]["runtime_add_h"], 100.0)
        self.assertGreater(report["operating_modes"]["worst_case_logging"]["runtime_add_h"], 50.0)
        self.assertLess(report["operating_modes"]["ble_2m_bulk_offload"]["offload_pct_usable_cap"], 1.0)
        self.assertEqual(report["operating_modes"]["usb_fs_bulk_offload"]["net_battery_drain_ma"], 0.0)

        # Ohmic sag & UVLO margin
        sag = report["ohmic_sag_cold_0c"]
        self.assertLess(sag["continuous_wc_sag_mv"], 5.0)
        self.assertLess(sag["transient_peak_sag_mv"], 25.0)
        self.assertGreater(sag["uvlo_margin_wc_mv"], 500.0)
        self.assertGreater(sag["uvlo_margin_transient_mv"], 500.0)

        # W25N01JW NAND ring & retention
        nand = report["nand_storage"]
        self.assertEqual(nand["usable_ring_bytes"], 130_809_856)
        self.assertAlmostEqual(nand["nominal_burst_3200hz"]["bytes_per_sec"], 5375.7333, places=3)
        self.assertGreaterEqual(nand["nominal_burst_3200hz"]["retention_hours"], 2.0)
        self.assertAlmostEqual(nand["continuous_400hz_n40"]["bytes_per_sec"], 8044.0, places=2)
        self.assertGreaterEqual(nand["continuous_400hz_n40"]["retention_hours"], 2.0)
        self.assertAlmostEqual(nand["continuous_400hz_n50"]["bytes_per_sec"], 7980.0, places=2)
        self.assertGreaterEqual(nand["continuous_400hz_n50"]["retention_hours"], 2.0)

    def test_dynamic_parsing_mutations_and_negative_gates(self) -> None:
        base_contract = power_budget.load_json_strict(CONTRACT_PATH)
        base_dims = power_budget.load_json_strict(ME_DIMS_PATH)
        with EE_BOM_PATH.open("r", encoding="utf-8", newline="") as f:
            base_ee_fieldnames = list(csv.DictReader(f).fieldnames or [])
        with EE_BOM_PATH.open("r", encoding="utf-8", newline="") as f:
            base_ee_rows = list(csv.DictReader(f))

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            c_path = tmp / "contract.json"
            b_path = tmp / "ee_bom.csv"
            d_path = tmp / "me_dims.json"

            # 1. Halve battery capacity_mah (450 -> 225) and double ICM-42688-P i_nom_ma (0.88 -> 1.76)
            mut_contract = copy.deepcopy(base_contract)
            mut_contract["hardware_abstraction"]["battery"]["capacity_mah"] = 225.0
            c_path.write_text(json.dumps(mut_contract), encoding="utf-8")
            d_path.write_text(json.dumps(base_dims), encoding="utf-8")

            mut_rows = copy.deepcopy(base_ee_rows)
            for row in mut_rows:
                if row["part_number"] == "ICM-42688-P":
                    row["i_nom_ma"] = "1.760"
            with b_path.open("w", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=base_ee_fieldnames)
                writer.writeheader()
                writer.writerows(mut_rows)

            proc_mut = subprocess.run(
                [
                    sys.executable,
                    str(POWER_BUDGET_SCRIPT),
                    "--contract",
                    str(c_path),
                    "--ee-bom",
                    str(b_path),
                    "--me-dims",
                    str(d_path),
                    "--me-bom",
                    str(ME_BOM_PATH),
                    "--json",
                    "--check",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            base_rep = power_budget.compute_budgets(
                base_contract,
                power_budget.load_ee_bom(EE_BOM_PATH),
                base_dims,
                power_budget.load_me_bom(ME_BOM_PATH),
            )
            base_1v8_nom = base_rep["rails_ma"]["nominal"]["1.8"]

            self.assertEqual(proc_mut.returncode, 0, msg=proc_mut.stderr)
            rep_mut = json.loads(proc_mut.stdout)
            self.assertAlmostEqual(rep_mut["battery"]["usable_capacity_add_mah"], 146.25, places=3)
            self.assertAlmostEqual(rep_mut["rails_ma"]["nominal"]["1.8"], base_1v8_nom + 0.880, places=3)
            # Runtime must recalculate deterministically (not hardcoded ~210.59 h)
            self.assertLess(rep_mut["operating_modes"]["nominal_play"]["runtime_add_h"], 100.0)

            # 2. Negative gate: capacity_mah = 5.0 forces < 2.0 h -> --check must fail (exit != 0)
            low_cap = copy.deepcopy(base_contract)
            low_cap["hardware_abstraction"]["battery"]["capacity_mah"] = 5.0
            c_path.write_text(json.dumps(low_cap), encoding="utf-8")
            proc_low_cap = subprocess.run(
                [
                    sys.executable,
                    str(POWER_BUDGET_SCRIPT),
                    "--contract",
                    str(c_path),
                    "--ee-bom",
                    str(EE_BOM_PATH),
                    "--me-dims",
                    str(ME_DIMS_PATH),
                    "--json",
                    "--check",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(proc_low_cap.returncode, 0)
            self.assertFalse(json.loads(proc_low_cap.stdout)["checks"]["passed"])

            # 3. Negative gate: usable_ring_blocks = 200 forces NAND retention < 2.0 h -> --check must fail
            low_nand = copy.deepcopy(base_contract)
            low_nand["hardware_abstraction"]["flash"]["usable_ring_blocks"] = 200
            c_path.write_text(json.dumps(low_nand), encoding="utf-8")
            proc_low_nand = subprocess.run(
                [
                    sys.executable,
                    str(POWER_BUDGET_SCRIPT),
                    "--contract",
                    str(c_path),
                    "--ee-bom",
                    str(EE_BOM_PATH),
                    "--me-dims",
                    str(ME_DIMS_PATH),
                    "--json",
                    "--check",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(proc_low_nand.returncode, 0)
            self.assertFalse(json.loads(proc_low_nand.stdout)["checks"]["passed"])

            # 4. Negative gate: gps_enabled = True -> --check must fail
            gps_on = copy.deepcopy(base_contract)
            gps_on["constants"]["gps_enabled"] = True
            c_path.write_text(json.dumps(gps_on), encoding="utf-8")
            proc_gps = subprocess.run(
                [
                    sys.executable,
                    str(POWER_BUDGET_SCRIPT),
                    "--contract",
                    str(c_path),
                    "--ee-bom",
                    str(EE_BOM_PATH),
                    "--me-dims",
                    str(ME_DIMS_PATH),
                    "--json",
                    "--check",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(proc_gps.returncode, 0)
            self.assertFalse(json.loads(proc_gps.stdout)["checks"]["passed"])

            def _run_cli(
                contract_p: Path = CONTRACT_PATH,
                ee_p: Path = EE_BOM_PATH,
                dims_p: Path = ME_DIMS_PATH,
                me_p: Path = ME_BOM_PATH,
            ) -> tuple[int, dict[str, Any]]:
                res = subprocess.run(
                    [
                        sys.executable,
                        str(POWER_BUDGET_SCRIPT),
                        "--contract",
                        str(contract_p),
                        "--ee-bom",
                        str(ee_p),
                        "--me-dims",
                        str(dims_p),
                        "--me-bom",
                        str(me_p),
                        "--json",
                        "--check",
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                return res.returncode, json.loads(res.stdout)

            # 5. Negative gate (C5b): underscore-joined MPN & notes-only forbidden RF part
            rf_rows = copy.deepcopy(base_ee_rows)
            rf_row = copy.deepcopy(rf_rows[0])
            rf_row["refdes"] = "U99"
            rf_row["subsystem_key"] = "aux_rx"
            rf_row["category"] = "Receiver"
            rf_row["part_number"] = "uBlox_MAX_M10S"
            rf_row["manufacturer"] = "uBlox"
            rf_row["bus_or_function"] = "UART1"
            rf_row["notes"] = "GPS receiver"
            rf_row["i_nom_ma"] = "0.000"
            rf_row["i_max_ma"] = "0.000"
            rf_row["i_standby_ua"] = "0.0"
            rf_row["qty"] = "1"
            rf_row["unit_mass_g"] = "0.000"
            rf_row["total_mass_g"] = "0.000"
            rf_rows.append(rf_row)
            with b_path.open("w", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=base_ee_fieldnames)
                writer.writeheader()
                writer.writerows(rf_rows)
            rc, rep = _run_cli(ee_p=b_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # 6. Negative gate (C6b): rail_v=0.0 with non-zero current must fail
            zero_rail_rows = copy.deepcopy(base_ee_rows)
            zero_rail_rows[0]["rail_v"] = "0.0"
            zero_rail_rows[0]["i_nom_ma"] = "500.0"
            zero_rail_rows[0]["i_max_ma"] = "500.0"
            with b_path.open("w", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=base_ee_fieldnames)
                writer.writeheader()
                writer.writerows(zero_rail_rows)
            rc, rep = _run_cli(ee_p=b_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # 7. Negative gate (C11c): envelope_mm length_mm=60 > 55 even when max_allowed_length_mm=65
            bad_env = copy.deepcopy(base_dims)
            bad_env["envelope_mm"]["length_mm"] = 60.0
            bad_env["envelope_mm"]["max_allowed_length_mm"] = 65.0
            d_path.write_text(json.dumps(bad_env), encoding="utf-8")
            rc, rep = _run_cli(dims_p=d_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # 8. Negative gate (C12c): dimensions.json breakdown_g > 28 g with ME_BOM_PATH present
            bad_mass = copy.deepcopy(base_dims)
            bad_mass["mass_budget_g"]["breakdown_g"]["inner_shell_pc_abs"] = 25.0
            d_path.write_text(json.dumps(bad_mass), encoding="utf-8")
            rc, rep = _run_cli(dims_p=d_path, me_p=ME_BOM_PATH)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # 9. Negative gate (C13a & C13b): R_BS det=-1 and non-orthogonal R_BS
            bad_rbs_det = copy.deepcopy(base_dims)
            bad_rbs_det["coordinate_frame_markings"]["R_BS"][0][2] = -1.0
            d_path.write_text(json.dumps(bad_rbs_det), encoding="utf-8")
            rc, rep = _run_cli(dims_p=d_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            bad_rbs_ortho = copy.deepcopy(base_dims)
            bad_rbs_ortho["coordinate_frame_markings"]["R_BS"] = [
                [1.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
            ]
            d_path.write_text(json.dumps(bad_rbs_ortho), encoding="utf-8")
            rc, rep = _run_cli(dims_p=d_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # 10. Negative gate (C14): battery_compartment_mm swollen cell > cavity_inner_mm
            bad_batt = copy.deepcopy(base_dims)
            bad_batt["battery_compartment_mm"]["cell_max_swollen_mm"] = [40.0, 28.0, 6.0]
            d_path.write_text(json.dumps(bad_batt), encoding="utf-8")
            rc, rep = _run_cli(dims_p=d_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

    def test_hardened_bom_edge_cases_and_zero_current_json(self) -> None:
        with EE_BOM_PATH.open("r", encoding="utf-8", newline="") as f:
            ee_reader = csv.DictReader(f)
            ee_fieldnames = list(ee_reader.fieldnames or [])
            base_ee_rows = list(ee_reader)
        with ME_BOM_PATH.open("r", encoding="utf-8", newline="") as f:
            me_reader = csv.DictReader(f)
            me_fieldnames = list(me_reader.fieldnames or [])
            base_me_rows = list(me_reader)
        base_contract = power_budget.load_json_strict(CONTRACT_PATH)

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            c_path = tmp / "contract.json"
            ee_path = tmp / "ee_bom.csv"
            me_path = tmp / "me_bom.csv"

            def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
                with path.open("w", encoding="utf-8", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(rows)

            def _run_cli(
                contract_p: Path = CONTRACT_PATH,
                ee_p: Path = EE_BOM_PATH,
                me_p: Path = ME_BOM_PATH,
            ) -> tuple[int, dict[str, Any]]:
                res = subprocess.run(
                    [
                        sys.executable,
                        str(POWER_BUDGET_SCRIPT),
                        "--contract",
                        str(contract_p),
                        "--ee-bom",
                        str(ee_p),
                        "--me-dims",
                        str(ME_DIMS_PATH),
                        "--me-bom",
                        str(me_p),
                        "--json",
                        "--check",
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                rep = json.loads(res.stdout, parse_constant=power_budget._reject_non_standard_json_constant)
                return res.returncode, rep

            # (a1) C15c: ME BOM negative qty must fail power_budget.py --check
            me_neg_qty = copy.deepcopy(base_me_rows)
            me_neg_qty[2]["qty"] = "-1"
            _write_csv(me_path, me_fieldnames, me_neg_qty)
            rc, rep = _run_cli(me_p=me_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (a2) C15d: ME BOM qty * unit_mass_g != total_mass_g must fail
            me_bad_mass = copy.deepcopy(base_me_rows)
            me_bad_mass[2]["qty"] = "2"
            _write_csv(me_path, me_fieldnames, me_bad_mass)
            rc, rep = _run_cli(me_p=me_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (b) C19a: DNI (qty=0) EE BOM row with non-zero current must fail
            ee_dni_curr = copy.deepcopy(base_ee_rows)
            for r in ee_dni_curr:
                if r["refdes"] == "RT1":
                    r["rail_v"] = "1.8"
                    r["i_nom_ma"] = "5.0"
                    r["i_max_ma"] = "5.0"
            _write_csv(ee_path, ee_fieldnames, ee_dni_curr)
            rc, rep = _run_cli(ee_p=ee_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (c1) C19b: negative r_int_0c_mohm on BT1 must fail
            ee_neg_r0 = copy.deepcopy(base_ee_rows)
            for r in ee_neg_r0:
                if r["refdes"] == "BT1":
                    r["r_int_0c_mohm"] = "-5000"
            _write_csv(ee_path, ee_fieldnames, ee_neg_r0)
            rc, rep = _run_cli(ee_p=ee_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (c2) negative r_int_25c_mohm and uvlo_mv <= 0 must fail
            ee_neg_r25 = copy.deepcopy(base_ee_rows)
            for r in ee_neg_r25:
                if r["refdes"] == "BT1":
                    r["r_int_25c_mohm"] = "-10"
                    r["uvlo_mv"] = "0"
            _write_csv(ee_path, ee_fieldnames, ee_neg_r25)
            rc, rep = _run_cli(ee_p=ee_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (c3) negative contract nominal_v / capacity_mah must fail cleanly
            bad_batt_cfg = copy.deepcopy(base_contract)
            bad_batt_cfg["hardware_abstraction"]["battery"]["nominal_v"] = -3.7
            bad_batt_cfg["hardware_abstraction"]["battery"]["capacity_mah"] = -450.0
            c_path.write_text(json.dumps(bad_batt_cfg), encoding="utf-8")
            rc, rep = _run_cli(contract_p=c_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])

            # (d) C18: all i_nom_ma=0.0 must emit valid strict JSON (no inf/NaN) and fail check
            ee_zero_nom = copy.deepcopy(base_ee_rows)
            for r in ee_zero_nom:
                r["i_nom_ma"] = "0.000"
            _write_csv(ee_path, ee_fieldnames, ee_zero_nom)
            rc, rep = _run_cli(ee_p=ee_path)
            self.assertNotEqual(rc, 0)
            self.assertFalse(rep["checks"]["passed"])
            self.assertIsNone(rep["operating_modes"]["nominal_play"]["runtime_add_h"])
            self.assertIsNone(rep["operating_modes"]["nominal_play"]["runtime_mul_h"])


if __name__ == "__main__":
    unittest.main()
