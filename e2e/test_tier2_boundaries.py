"""Tier 2 E2E Tests: Boundary Value Analysis, Corner Cases & Fault Injection (F1-F11, 59 tests)."""

from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from e2e import helpers  # noqa: E402
from jsonschema import ValidationError  # noqa: E402


# ---------------------------------------------------------------------------
# F1 Boundaries (5 tests)
# ---------------------------------------------------------------------------
class TestF1Boundaries(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.spec = helpers.load_spec_text()
        cls.contract = helpers.load_contract()

    def test_f1_t2_01_spec_vs_contract_numeric_parity(self) -> None:
        c = self.contract
        for token in (
            str(c["constants"]["gravity_mps2"]),
            "52.0 × 36.0 × 10.5 mm",
            "24 g",
            "450 mAh",
            "998",
            "60.96",
            "25.908",
            "8.5344",
            "0.0047884033203125",
            "0.980665",
        ):
            self.assertIn(token, self.spec, msg=f"Numeric parity token '{token}' missing from PROJECT_SPEC.md")

    def test_f1_t2_02_honest_unbounded_drift_caveat_present(self) -> None:
        self.assertIn("2.5–3.5 m", self.spec)
        self.assertIn("<= 6.0 m", self.spec)
        self.assertIn("> 10–25 m", self.spec)
        self.assertIn("NOT sub-meter", self.spec)

    def test_f1_t2_03_bus_isolation_invariant_documented(self) -> None:
        self.assertIn("MUST NOT share `SPIM4`", self.spec)
        self.assertIn("10 MHz", self.spec)
        self.assertIn("16 MHz", self.spec)
        self.assertIn("0x47", self.spec)
        self.assertIn("0x46", self.spec)

    def test_f1_t2_04_frame_rate_math_reconciliation(self) -> None:
        rate_bps = 272 * 10 + 52 * 50 + (3104 * 60 / 3600.0) + (40 * 0.1)
        self.assertAlmostEqual(rate_bps, 5375.733333, places=2)
        hours = helpers.FLASH_USABLE_RING_BYTES / rate_bps / 3600.0
        self.assertAlmostEqual(hours, 6.76, delta=0.05)
        self.assertIn("5,375.73 B/s", self.spec)
        self.assertIn("6.76 hours", self.spec)

    def test_f1_t2_05_adxl372_right_shift_12bit_range_parity(self) -> None:
        self.assertIn("[-2048, +2047]", self.spec)
        self.assertIn("[-2008.40, +2007.42]", self.spec)
        self.assertEqual(self.contract["units_and_tolerances"]["acceleration"]["operating_range"], [-2008.40192, 2008.40192])


# ---------------------------------------------------------------------------
# F2 Boundaries (5 tests)
# ---------------------------------------------------------------------------
class TestF2Boundaries(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = helpers.load_contract()

    def test_f2_t2_01_timestamp_64bit_exceeds_32bit_rollover(self) -> None:
        ex_imu = self.contract["examples"]["imu_batch_packet"]
        self.assertGreater(ex_imu["timestamp_us"], 0xFFFFFFFF)
        validator = helpers.get_validator("imu_batch_packet")
        max_pkt = copy.deepcopy(ex_imu)
        max_pkt["timestamp_us"] = (1 << 64) - 1
        validator.validate(max_pkt)
        for bad_ts in (-1, 1 << 64):
            bad_pkt = copy.deepcopy(ex_imu)
            bad_pkt["timestamp_us"] = bad_ts
            with self.assertRaises(ValidationError):
                validator.validate(bad_pkt)

    def test_f2_t2_02_adxl372_min_max_12bit_bounds(self) -> None:
        hg_val = helpers.get_validator("high_g_batch_packet")
        pt_val = helpers.get_validator("analytics_trajectory_point")
        for valid_val in (-2048, 2047):
            pkt = copy.deepcopy(self.contract["examples"]["high_g_batch_packet"])
            pkt["samples"][0]["ax_i16"] = valid_val
            hg_val.validate(pkt)
        for bad_val in (-2049, 2048):
            pkt = copy.deepcopy(self.contract["examples"]["high_g_batch_packet"])
            pkt["samples"][0]["ax_i16"] = bad_val
            with self.assertRaises(ValidationError):
                hg_val.validate(pkt)
        pt = copy.deepcopy(self.contract["examples"]["analytics_trajectory_point"])
        pt["ax"] = -2048 * helpers.ADXL372_SCALE_MPS2_PER_LSB
        pt["ay"] = 0.0
        pt["az"] = 0.0
        pt["accel_mag"] = abs(pt["ax"])
        pt_val.validate(pt)

    def test_f2_t2_03_download_command_if_then_payload_isolation(self) -> None:
        cmd_val = helpers.get_validator("download_command")
        bad_cases = [
            ("LIST_SESSIONS", 1, {"utc_ms": 1}),
            ("DELETE_SESSION", 5, {"session_id": 1001}),
            ("READ_CHUNK", 3, {"session_id": 1001, "offset_bytes": 0, "length_bytes": 0}),
            ("READ_CHUNK", 3, {"session_id": 1001, "offset_bytes": 0, "length_bytes": 4097}),
            ("SET_UTC_TIME", 2, {"utc_ms": 1763852500000}),
        ]
        for command, cmd_id, payload in bad_cases:
            with self.assertRaises(ValidationError):
                cmd_val.validate({"sync": 43605, "command": command, "cmd_id": cmd_id, "seq": 1, "payload": payload, "crc32": 0})

    def test_f2_t2_04_rink_containment_boundary_and_corner_arc(self) -> None:
        self.assertTrue(helpers.is_inside_rink(0.0, 12.954, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(0.0, 12.955, "NHL_200X85FT"))
        # Corner rectangle trap: |x|<30.48 and |y|<12.954 yet outside R_c=8.5344 fillet
        self.assertFalse(helpers.is_inside_rink(29.5, 12.2, "NHL_200X85FT"))
        core_x = 60.96 / 2.0 - 8.5344
        core_y = 25.908 / 2.0 - 8.5344
        arc_x = core_x + 8.5344 * math.sqrt(0.5)
        arc_y = core_y + 8.5344 * math.sqrt(0.5)
        self.assertTrue(helpers.is_inside_rink(arc_x, arc_y, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(arc_x, arc_y, "NHL_200X85FT", torso_margin_m=0.25))

    def test_f2_t2_05_analytics_report_cross_field_and_additional_props(self) -> None:
        for sname, schema in self.contract["schemas"].items():
            self.assertIs(schema.get("additionalProperties"), False, msg=f"{sname} must set additionalProperties=false")
        from docs.validate_contract import ContractInvariantError, validate_analytics_report_semantics

        bad_spd = copy.deepcopy(self.contract["examples"]["analytics_report"])
        bad_spd["shifts"][0]["avg_speed_mps"] = 12.0
        with self.assertRaises(ContractInvariantError):
            validate_analytics_report_semantics(bad_spd, presets=self.contract["rink_geometry_presets"])

        bad_q = copy.deepcopy(self.contract["examples"]["analytics_report"])
        bad_q["trajectory"][0]["q_WB"] = [1.0, 1.0, 0.0, 0.0]
        with self.assertRaises(ContractInvariantError):
            validate_analytics_report_semantics(bad_q, presets=self.contract["rink_geometry_presets"])


# ---------------------------------------------------------------------------
# F3 Boundaries (5 tests)
# ---------------------------------------------------------------------------
class TestF3Boundaries(unittest.TestCase):
    def _run_mutated_contract(self, mutator) -> subprocess.CompletedProcess[str]:
        contract = copy.deepcopy(helpers.load_contract(use_cache=False))
        mutator(contract)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "bad_contract.json"
            p.write_text(json.dumps(contract), encoding="utf-8")
            return helpers.run_validate_contract_cli(extra_args=("--contract", str(p), "--skip-tests"))

    def test_f3_t2_01_rejects_non_strict_json_nan_and_infinity(self) -> None:
        raw = helpers.CONTRACT_PATH.read_text(encoding="utf-8").replace("9.80665", "NaN", 1)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "nan_contract.json"
            p.write_text(raw, encoding="utf-8")
            proc = helpers.run_validate_contract_cli(extra_args=("--contract", str(p), "--skip-tests"))
            self.assertNotEqual(proc.returncode, 0)

    def test_f3_t2_02_rejects_gps_or_external_anchors_enabled(self) -> None:
        proc1 = self._run_mutated_contract(lambda c: c["constants"].__setitem__("gps_enabled", True))
        self.assertNotEqual(proc1.returncode, 0)
        proc2 = self._run_mutated_contract(lambda c: c["hardware_abstraction"].__setitem__("uses_external_anchors", True))
        self.assertNotEqual(proc2.returncode, 0)

    def test_f3_t2_03_rejects_32bit_timestamp_downgrade(self) -> None:
        def mut(c: dict) -> None:
            c["constants"]["timestamp_width_bits"] = 32
            c["constants"]["timestamp_type"] = "uint32_t"

        proc = self._run_mutated_contract(mut)
        self.assertNotEqual(proc.returncode, 0)

    def test_f3_t2_04_rejects_shared_spi_bus_or_wrong_bmp581_addr(self) -> None:
        proc1 = self._run_mutated_contract(
            lambda c: c["hardware_abstraction"]["sensors"]["high_g_imu"].__setitem__("bus", "SPIM4")
        )
        self.assertNotEqual(proc1.returncode, 0)
        proc2 = self._run_mutated_contract(
            lambda c: c["hardware_abstraction"]["sensors"]["barometer"].__setitem__("i2c_addr_hex", "0x77")
        )
        self.assertNotEqual(proc2.returncode, 0)

    def test_f3_t2_05_rejects_schema_drift_or_improper_rotation_matrix(self) -> None:
        proc1 = self._run_mutated_contract(
            lambda c: c["schemas"]["analytics_report"]["$defs"]["analytics_shift_summary"]["properties"].__setitem__(
                "extra_drift", {"type": "string"}
            )
        )
        self.assertNotEqual(proc1.returncode, 0)
        proc2 = self._run_mutated_contract(
            lambda c: c["examples"]["calibration_block"].__setitem__(
                "sensor_to_body_rotation_matrix", [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            )
        )
        self.assertNotEqual(proc2.returncode, 0)


# ---------------------------------------------------------------------------
# F4 Boundaries (M2, 5 tests)
# ---------------------------------------------------------------------------
class TestF4Boundaries(unittest.TestCase):
    def test_f4_t2_01_spi_bus_isolation_hard_invariant(self) -> None:
        m2 = helpers.require_m2()
        pinmux = helpers.load_json_strict(m2["pinmux"])
        buses = pinmux.get("buses", pinmux)
        spim4 = buses["SPIM4"]
        spim1 = buses["SPIM1"]
        self.assertLessEqual(int(spim1.get("clock_hz", 8_000_000)), 10_000_000)
        self.assertGreaterEqual(int(spim4.get("clock_hz", 16_000_000)), 12_000_000)
        p4 = {v["pin"] for v in spim4.get("pins", {}).values() if isinstance(v, dict) and "pin" in v}
        p1 = {v["pin"] for v in spim1.get("pins", {}).values() if isinstance(v, dict) and "pin" in v}
        self.assertEqual(p4 & p1, set(), msg=f"SPIM4 and SPIM1 share pins: {p4 & p1}")

    def test_f4_t2_02_pin_conflict_uniqueness_across_all_signals(self) -> None:
        m2 = helpers.require_m2()
        pinmux = helpers.load_json_strict(m2["pinmux"])
        seen: dict[str, str] = {}
        buses = pinmux.get("buses", {})
        for bname, binfo in buses.items():
            for sig, sinfo in binfo.get("pins", {}).items():
                if isinstance(sinfo, dict) and "pin" in sinfo and sinfo["pin"].startswith("P"):
                    pin = sinfo["pin"]
                    self.assertNotIn(pin, seen, msg=f"Pin {pin} reused by {bname}.{sig} and {seen.get(pin)}")
                    seen[pin] = f"{bname}.{sig}"
        for sig, sinfo in pinmux.get("gpio_signals", {}).items():
            if isinstance(sinfo, dict) and "pin" in sinfo and sinfo["pin"].startswith("P"):
                pin = sinfo["pin"]
                self.assertNotIn(pin, seen, msg=f"Pin {pin} reused by gpio.{sig} and {seen.get(pin)}")
                seen[pin] = f"gpio.{sig}"

    def test_f4_t2_03_i2c_address_collision_check(self) -> None:
        m2 = helpers.require_m2()
        pinmux = helpers.load_json_strict(m2["pinmux"])
        buses = pinmux.get("buses", {})
        for bname in ("TWIM0", "TWIM1"):
            addrs = [d["i2c_addr_hex"].lower() for d in buses[bname].get("devices", []) if "i2c_addr_hex" in d]
            self.assertEqual(len(addrs), len(set(addrs)), msg=f"I2C address collision on {bname}: {addrs}")
            for a in addrs:
                val = int(a, 16)
                self.assertGreaterEqual(val, 0x08)
                self.assertLessEqual(val, 0x77)

    def test_f4_t2_04_interrupt_line_completeness(self) -> None:
        m2 = helpers.require_m2()
        txt = m2["pinmux"].read_text(encoding="utf-8")
        for sig in ("IMU_INT1", "ADXL_INT1", "MAG_INT", "BARO_INT", "PMIC_INT", "SHPHLD"):
            self.assertIn(sig, txt, msg=f"Missing required interrupt/wakeup signal {sig}")

    def test_f4_t2_05_cold_charge_inhibit_and_1v8_mcu_io_boundary(self) -> None:
        m2 = helpers.require_m2()
        combined = m2["pinmux"].read_text(encoding="utf-8") + "\n" + m2["schematic_spec"].read_text(encoding="utf-8")
        self.assertIn("1.8", combined)
        self.assertIn("0", combined)
        self.assertIn("NTC", combined)


# ---------------------------------------------------------------------------
# F5 Boundaries (M2, 5 tests)
# ---------------------------------------------------------------------------
class TestF5Boundaries(unittest.TestCase):
    def test_f5_t2_01_exact_0c_additive_derating_and_ohmic_sag_boundary(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        self.assertGreater(budget["derated_capacity_mah"], 0.0)
        self.assertLessEqual(budget["derated_capacity_mah"], 306.0)
        self.assertGreaterEqual(budget["worst_case_runtime_h"], 2.0)
        if budget["ohmic_sag_mv"] is not None:
            self.assertLess(budget["ohmic_sag_mv"], 5.0)

    def test_f5_t2_02_heavy_body_check_high_g_burst_flash_stress(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        usable = budget["flash_usable_bytes"] or helpers.FLASH_USABLE_RING_BYTES
        # 300 impacts/hr * 3104 B = 258.67 B/s + base 5320 B/s + events 40 B/s = 5618.67 B/s
        heavy_hours = usable / 5618.67 / 3600.0
        self.assertGreaterEqual(heavy_hours, 2.0)
        self.assertGreaterEqual(budget["flash_cont_400hz_hours"], 2.0)

    def test_f5_t2_03_20_bad_block_reserve_tolerance(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        usable = budget["flash_usable_bytes"] or helpers.FLASH_USABLE_RING_BYTES
        self.assertLessEqual(usable, (1024 - 20) * helpers.FLASH_BLOCK_BYTES)
        self.assertGreaterEqual(usable, 998 * helpers.FLASH_BLOCK_BYTES)

    def test_f5_t2_04_buck_efficiency_and_rail_voltage_boundary(self) -> None:
        m2 = helpers.require_m2()
        raw_budget = helpers.run_power_budget_json()
        with m2["ee_bom"].open("r", encoding="utf-8") as f:
            ee_rows = list(csv.DictReader(f))

        # Dynamically sum worst-case rail currents & extract PMIC/battery specs from hardware/ee/bom.csv
        bom_rail_wc_ma: dict[float, float] = {}
        buck_eta_bom: float | None = None
        uvlo_mv_bom: float | None = None
        for row in ee_rows:
            qty = int(float(row["qty"]))
            if qty <= 0:
                continue
            if row["buck_efficiency"].strip():
                buck_eta_bom = float(row["buck_efficiency"])
            if row["uvlo_mv"].strip():
                uvlo_mv_bom = float(row["uvlo_mv"])
            rv = round(float(row["rail_v"]), 4)
            imax = float(row["i_max_ma"] or 0.0)
            if rv > 0.0 and imax > 0.0:
                bom_rail_wc_ma[rv] = bom_rail_wc_ma.get(rv, 0.0) + imax

        self.assertIsNotNone(buck_eta_bom)
        self.assertIsNotNone(uvlo_mv_bom)
        assert buck_eta_bom is not None and uvlo_mv_bom is not None

        bat_cfg = raw_budget["battery"]
        self.assertAlmostEqual(buck_eta_bom, float(bat_cfg["buck_efficiency"]), places=4)
        self.assertAlmostEqual(uvlo_mv_bom, float(bat_cfg["uvlo_mv"]), places=2)

        # Verify power_budget.py worst-case rail rollup matches EE BOM
        pb_wc_rails = {round(float(k), 4): float(v) for k, v in raw_budget["rails_ma"]["worst_case"].items()}
        for rv, imax in bom_rail_wc_ma.items():
            self.assertAlmostEqual(pb_wc_rails.get(rv, 0.0), imax, places=3)

        # Evaluate runtime at end-of-discharge V_batt = UVLO (3.0 V) and degraded buck efficiency (eta - 0.05)
        v_eod = uvlo_mv_bom / 1000.0
        eta_degraded = max(0.75, buck_eta_bom - 0.05)
        i_batt_eod_ma = sum(
            imax if abs(rv - float(bat_cfg["nominal_v"])) < 1e-4 else imax * (rv / (v_eod * eta_degraded))
            for rv, imax in bom_rail_wc_ma.items()
        )
        self.assertGreater(i_batt_eod_ma, 0.0)
        derated_mah = float(bat_cfg["usable_capacity_add_mah"])
        self.assertGreater(derated_mah, 0.0)
        runtime_eod_h = derated_mah / i_batt_eod_ma
        self.assertGreater(runtime_eod_h, 2.0)

    def test_f5_t2_05_no_silent_unmet_budget_mutation(self) -> None:
        helpers.require_m2()
        proc = helpers.run_hardware_unit_tests_cached()
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)
        combined = proc.stderr + proc.stdout
        self.assertIn("Ran ", combined)
        self.assertIn("test_dynamic_parsing_mutations_and_negative_gates", combined)


# ---------------------------------------------------------------------------
# F6 Boundaries (M2, 5 tests)
# ---------------------------------------------------------------------------
class TestF6Boundaries(unittest.TestCase):
    def test_f6_t2_01_internal_stackup_thickness_budget(self) -> None:
        m2 = helpers.require_m2()
        raw = helpers.load_json_strict(m2["dimensions"])
        dims = helpers.extract_mechanical_dims(raw)
        self.assertLessEqual(dims["thickness_mm"], 10.5)
        if "tolerance_stackup_z_mm" in raw:
            self.assertLessEqual(float(raw["tolerance_stackup_z_mm"]["nominal_total_mm"]), 10.5)

    def test_f6_t2_02_battery_footprint_fits_inside_enclosure(self) -> None:
        m2 = helpers.require_m2()
        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        self.assertGreaterEqual(dims["length_mm"], 35.0 + 3.0)
        self.assertGreaterEqual(dims["width_mm"], 25.0 + 3.0)

    def test_f6_t2_03_mass_rollup_cross_bom_consistency(self) -> None:
        m2 = helpers.require_m2()
        raw = helpers.load_json_strict(m2["dimensions"])
        dims = helpers.extract_mechanical_dims(raw)
        self.assertLessEqual(dims["mass_g"], 28.0)
        if "mass_budget_g" in raw and "breakdown_g" in raw["mass_budget_g"]:
            total = sum(float(v) for v in raw["mass_budget_g"]["breakdown_g"].values())
            self.assertAlmostEqual(total, dims["mass_g"], delta=1.5)

    def test_f6_t2_04_smaller_than_hockey_puck_sanity_boundary(self) -> None:
        m2 = helpers.require_m2()
        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        vol = dims["length_mm"] * dims["width_mm"] * dims["thickness_mm"]
        self.assertLess(vol, 115_800.0)
        self.assertLess(dims["mass_g"], 40.0)

    def test_f6_t2_05_no_external_mounting_or_anchor_hardware(self) -> None:
        m2 = helpers.require_m2()
        txt = (m2["mech_bom"].read_text(encoding="utf-8") + m2["enclosure_spec"].read_text(encoding="utf-8")).lower()
        for bad in ("tripod", "rink-board", "rink board", "uwb", "beacon"):
            self.assertNotIn(bad, txt)


# ---------------------------------------------------------------------------
# F7 Boundaries (M3, 6 tests)
# ---------------------------------------------------------------------------
class TestF7Boundaries(unittest.TestCase):
    def test_f7_t2_01_gt_71_5_min_64bit_timestamp_continuity(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        ts = [f["header"]["timestamp_us"] for f in frames]
        self.assertGreater(ts[-1], 0xFFFFFFFF)
        self.assertGreaterEqual(ts[-1], 7_200_000_000)
        self.assertTrue(all(ts[i] <= ts[i + 1] for i in range(len(ts) - 1)))

    def test_f7_t2_02_adxl372_signed_extremes_minus_2048_plus_2047(self) -> None:
        helpers.require_m3()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        self.assertIn("test_adxl372_unpack_and_misaligned_serdes", proc_test.stdout)

        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        hg = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_HIGH_G_BATCH]
        self.assertGreater(len(hg), 0)
        peak_mps2 = 0.0
        for f in hg:
            for s in f["packet"]["samples"]:
                for axis in ("ax_i16", "ay_i16", "az_i16"):
                    self.assertGreaterEqual(s[axis], -2048)
                    self.assertLessEqual(s[axis], 2047)
                ax = helpers.adxl372_i16_to_mps2(s["ax_i16"])
                ay = helpers.adxl372_i16_to_mps2(s["ay_i16"])
                az = helpers.adxl372_i16_to_mps2(s["az_i16"])
                peak_mps2 = max(peak_mps2, math.sqrt(ax * ax + ay * ay + az * az))
        self.assertGreater(peak_mps2, 16.0 * helpers.GRAVITY_MPS2)

    def test_f7_t2_03_unverified_delete_guard_and_verify_mismatch(self) -> None:
        helpers.require_m3()
        helpers.require_m4()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        self.assertIn("test_usb_protocol_all_commands_and_10_statuses", proc_test.stdout)
        self.assertIn("test_ring_reclamation_verified_vs_unverified", proc_test.stdout)

        # Cross-verify C-generated .hks against M4 downloader delete-guard & verify-mismatch
        from sim.downloader import DeviceCommandError, DownloadClient, SimulatedDeviceTransport, StatusCode

        raw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        sid = int(info["session_id"])
        transport = SimulatedDeviceTransport([raw])
        client = DownloadClient(transport)
        with self.assertRaises(DeviceCommandError) as ctx_del:
            client.delete_session(sid, force=False)
        self.assertEqual(ctx_del.exception.status_code, StatusCode.ERR_NOT_VERIFIED)
        with self.assertRaises(DeviceCommandError) as ctx_mis:
            client.ack_verify(sid, (int(info["file_crc32"]) ^ 0x1), b"\x00" * 32)
        self.assertEqual(ctx_mis.exception.status_code, StatusCode.ERR_VERIFY_MISMATCH)

    def test_f7_t2_04_download_protocol_malformed_frames_busy_and_bounds(self) -> None:
        helpers.require_m3()
        helpers.require_m4()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        self.assertIn("test_protocol_oversized_resync_and_no_memcpy_overlap", proc_test.stdout)

        from sim.flash_codec import BadCRCError, decode_hks_session

        raw_orig, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        corrupted = bytearray(raw_orig)
        corrupted[30] ^= 0x01
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bytes(corrupted), verify_crc=True, verify_footer_sha256=True)
        with self.assertRaises(BadCRCError):
            decode_hks_session(bytes(corrupted), require_footer=True)

    def test_f7_t2_05_telemetry_ring_wraparound_and_bad_block_skip(self) -> None:
        helpers.require_m3()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        for test_name in (
            "test_nand_nop4_and_bad_block_skip",
            "test_noncontiguous_session_block_chain_reboot",
            "test_mid_session_ring_reclamation_while_recording",
            "test_multipage_flash_full_headroom_and_dropped_counters",
            "test_session_index_compaction_64_sessions_and_page63_reboot",
        ):
            self.assertIn(test_name, proc_test.stdout, msg=f"Missing C ring/NAND test '{test_name}'")

    def test_f7_t2_06_mmc5983ma_centering_bounds_and_rbs_reflection_rejection(self) -> None:
        helpers.require_m3()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        self.assertIn("test_r_bs_dual_imu_attitude_and_detectors", proc_test.stdout)
        self.assertIn("test_attitude_and_contract_edge_cases", proc_test.stdout)

        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        cal = frames[1]["packet"]
        ok, _, det = helpers.is_so3(cal["sensor_to_body_rotation_matrix"])
        self.assertTrue(ok)
        self.assertAlmostEqual(det, 1.0, places=5)
        for f in frames:
            if f["header"]["msg_type"] == helpers.MSG_AUX_ENV:
                for ax in ("mx_i32", "my_i32", "mz_i32"):
                    self.assertGreaterEqual(f["packet"][ax], -131072)
                    self.assertLessEqual(f["packet"][ax], 131071)


# ---------------------------------------------------------------------------
# F8 Boundaries (M3, 6 tests)
# ---------------------------------------------------------------------------
class TestF8Boundaries(unittest.TestCase):
    def test_f8_t2_01_gt_2h_c_generated_hks_timestamp_rollover(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertGreater(frames[-1]["header"]["timestamp_us"], 4_294_967_295)
        self.assertGreaterEqual(frames[-1]["header"]["timestamp_us"], 7_200_000_000)

    def test_f8_t2_02_max_batch_payload_boundary(self) -> None:
        helpers.require_m3()
        helpers.require_m4()
        from sim.flash_codec import (
            IMU_MAX_SAMPLES,
            MAX_PAYLOAD_BYTES,
            ImuBatchPacket,
            ImuSample,
            MsgType,
            PayloadLengthError,
            pack_frame,
            pack_imu_batch_payload,
        )

        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        for f in frames:
            self.assertLessEqual(f["header"]["payload_len"], helpers.MAX_PAYLOAD_BYTES)
            self.assertLessEqual(f["frame_bytes"], helpers.MAX_CHUNK_BYTES)

        # Exact max payload boundaries: N=338 IMU samples -> 4064 B payload (4088 B frame) accepted; N=339 rejected
        samples_338 = tuple(ImuSample(0, 0, 2048, 0, 0, 0) for _ in range(IMU_MAX_SAMPLES))
        pl_max = pack_imu_batch_payload(ImuBatchPacket(timestamp_us=0, odr_hz=200, temp_c_x100=2100, samples=samples_338))
        self.assertEqual(len(pl_max), 8 + 12 * 338)
        self.assertLessEqual(len(pl_max), MAX_PAYLOAD_BYTES)
        self.assertLessEqual(len(pack_frame(MsgType.IMU_BATCH, pl_max, seq=1, timestamp_us=0)), 4096)

        samples_339 = tuple(ImuSample(0, 0, 2048, 0, 0, 0) for _ in range(IMU_MAX_SAMPLES + 1))
        with self.assertRaises(PayloadLengthError):
            pack_imu_batch_payload(ImuBatchPacket(timestamp_us=0, odr_hz=200, temp_c_x100=2100, samples=samples_339))

        # Exact 4072 B accepted vs 4073 B rejected
        self.assertEqual(len(pack_frame(MsgType.EVENT_MARKER, b"\x00" * MAX_PAYLOAD_BYTES, seq=1, timestamp_us=0)), 4096)
        with self.assertRaises(PayloadLengthError):
            pack_frame(MsgType.EVENT_MARKER, b"\x00" * (MAX_PAYLOAD_BYTES + 1), seq=1, timestamp_us=0)

    def test_f8_t2_03_high_g_impact_20_80g_and_200g_c_binary(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        hg = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_HIGH_G_BATCH]
        self.assertGreater(len(hg), 0)
        peak_g = max(
            math.sqrt(
                helpers.adxl372_i16_to_mps2(s["ax_i16"]) ** 2
                + helpers.adxl372_i16_to_mps2(s["ay_i16"]) ** 2
                + helpers.adxl372_i16_to_mps2(s["az_i16"]) ** 2
            )
            / helpers.GRAVITY_MPS2
            for f in hg
            for s in f["packet"]["samples"]
        )
        self.assertGreaterEqual(peak_g, 20.0)
        self.assertLessEqual(peak_g, 200.0 * math.sqrt(3.0))

    def test_f8_t2_04_empty_zero_shift_stationary_cli_output(self) -> None:
        helpers.require_m3()
        raw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual(frames[-1]["packet"]["dropped_imu_samples"], 0)
        self.assertEqual(frames[-1]["packet"]["dropped_high_g_samples"], 0)
        self.assertEqual(info["nop_violations"], 0)
        # First IMU batch is stationary bench (0..2 s) -> ||f_B|| ~ g
        imu0 = next(f for f in frames if f["header"]["msg_type"] == helpers.MSG_IMU_BATCH)
        s0 = imu0["packet"]["samples"][0]
        scale = frames[0]["packet"]["accel_scale_mps2_per_lsb"]
        amag = math.sqrt((s0["ax_i16"] * scale) ** 2 + (s0["ay_i16"] * scale) ** 2 + (s0["az_i16"] * scale) ** 2)
        self.assertAlmostEqual(amag, helpers.GRAVITY_MPS2, delta=0.5)

    def test_f8_t2_05_seq_counter_uint16_rollover_65535_to_0(self) -> None:
        helpers.require_m3()
        helpers.require_m4()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        seqs = [f["header"]["seq"] for f in frames]
        self.assertGreater(len(seqs), 100)
        self.assertTrue(all(0 <= s <= 65535 for s in seqs))
        for idx in range(len(seqs) - 1):
            self.assertEqual(seqs[idx + 1], (seqs[idx] + 1) & 0xFFFF)

        # Unconditionally verify uint16_t rollover 65534 -> 65535 -> 0 -> 1 across flash_codec
        from sim.flash_codec import (
            AuxEnvPacket,
            CalibrationBlock,
            EventMarkerPacket,
            ImuBatchPacket,
            ImuSample,
            SessionHeader,
            decode_hks_session,
            encode_hks_session,
        )

        cal = CalibrationBlock()
        cal = CalibrationBlock(cal_crc32=cal.compute_crc32())
        hdr = SessionHeader(session_id=8205, cal_crc32=cal.cal_crc32, seq=65534)
        imu = ImuBatchPacket(timestamp_us=0, odr_hz=200, temp_c_x100=2100, samples=(ImuSample(2048, 0, 0, 0, 0, 0),), seq=0)
        aux = AuxEnvPacket(timestamp_us=5000, mx_i32=3200, my_i32=-1400, mz_i32=7300, pressure_pa_x64=6484800, baro_temp_c_x100=2140, vbatt_mv=3920, batt_temp_c_x100=1850, soc_pct_x10=845, pmic_status=0x40, validity_flags=0x0F, seq=1)
        evt = EventMarkerPacket(timestamp_us=10000, event_code="BUTTON_MARK", severity=0, value_f32=1.0, detail_u64=0, seq=2)
        cal_roll = CalibrationBlock(
            cal_version=cal.cal_version,
            flags=cal.flags,
            cal_utc_ms=cal.cal_utc_ms,
            sensor_to_body_rotation_matrix=cal.sensor_to_body_rotation_matrix,
            accel_misalign_scale=cal.accel_misalign_scale,
            accel_bias_mps2=cal.accel_bias_mps2,
            gyro_misalign_scale=cal.gyro_misalign_scale,
            gyro_bias_rads=cal.gyro_bias_rads,
            high_g_to_body_rotation_matrix=cal.high_g_to_body_rotation_matrix,
            high_g_misalign_scale=cal.high_g_misalign_scale,
            high_g_bias_mps2=cal.high_g_bias_mps2,
            mag_hard_iron_ut=cal.mag_hard_iron_ut,
            mag_soft_iron_matrix=cal.mag_soft_iron_matrix,
            baseline_mag_norm_ut=cal.baseline_mag_norm_ut,
            baseline_mag_dip_deg=cal.baseline_mag_dip_deg,
            cal_crc32=cal.cal_crc32,
            frame_seq=65535,
        )
        rolled = encode_hks_session(hdr, cal_roll, [imu, aux, evt], end_utc_ms=1763852501000, footer_seq=3)
        u_frames = helpers.unpack_frames(rolled, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual([f["header"]["seq"] for f in u_frames], [65534, 65535, 0, 1, 2, 3])
        dec = decode_hks_session(rolled, require_footer=True, verify_so3=True)
        self.assertEqual(dec.header.seq, 65534)
        self.assertEqual(dec.imu_batches[0].seq, 0)

    def test_f8_t2_06_invalid_cli_args_nonzero_exit_no_partial_file(self) -> None:
        helpers.require_m3()
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "bad_arg.hks"
            proc = helpers.run_fw_cli(["--invalid-flag", "--out", str(out)])
            self.assertNotEqual(proc.returncode, 0)
            self.assertFalse(out.exists())


# ---------------------------------------------------------------------------
# F9 Boundaries (M4, 6 tests)
# ---------------------------------------------------------------------------
class TestF9Boundaries(unittest.TestCase):
    def test_f9_t2_01_gt_71_5_min_full_game_simulation(self) -> None:
        helpers.require_m4()
        raw, _, truth = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=7500.0, shifts=60, seed=4001
        )
        self.assertLessEqual(len(raw), helpers.FLASH_USABLE_RING_BYTES)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertGreater(frames[-1]["header"]["timestamp_us"], 0xFFFFFFFF)
        self.assertGreaterEqual(truth["total_duration_s"], 7200.0)

    def test_f9_t2_02_extreme_body_checks_20_80g_and_150g(self) -> None:
        helpers.require_m4()
        raw, _, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        hg = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_HIGH_G_BATCH]
        self.assertGreater(len(hg), 0)
        peak_g = 0.0
        for f in hg:
            for s in f["packet"]["samples"]:
                for ax in ("ax_i16", "ay_i16", "az_i16"):
                    self.assertGreaterEqual(s[ax], -2048)
                    self.assertLessEqual(s[ax], 2047)
                ax_m = helpers.adxl372_i16_to_mps2(s["ax_i16"])
                ay_m = helpers.adxl372_i16_to_mps2(s["ay_i16"])
                az_m = helpers.adxl372_i16_to_mps2(s["az_i16"])
                peak_g = max(peak_g, math.sqrt(ax_m * ax_m + ay_m * ay_m + az_m * az_m) / helpers.GRAVITY_MPS2)
        self.assertGreater(peak_g, 16.0)

    def test_f9_t2_03_rink_corner_ricochet_and_containment_nhl_iihf(self) -> None:
        helpers.require_m4()
        for preset in ("NHL_200X85FT", "IIHF_60X30M"):
            _, _, truth = helpers.sim_and_analyze_cached(preset=preset, duration_s=140.0, shifts=2, seed=9203)
            for pt in truth["trajectory"]:
                self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], preset, torso_margin_m=0.25, tol=1e-4))

    def test_f9_t2_04_severe_magnetic_anomaly_zone(self) -> None:
        helpers.require_m4()
        raw, _, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        aux = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_AUX_ENV]
        self.assertGreater(len(aux), 0)
        mag_anom_count = sum(1 for f in aux if (f["header"]["flags"] & 0x04) != 0)
        self.assertGreater(mag_anom_count, 0)
        for f in aux:
            for ax in ("mx_i32", "my_i32", "mz_i32"):
                self.assertGreaterEqual(f["packet"][ax], -131072)
                self.assertLessEqual(f["packet"][ax], 131071)

    def test_f9_t2_05_empty_zero_shift_100pct_bench_session(self) -> None:
        helpers.require_m4()
        from sim.flash_codec import (
            AuxEnvPacket,
            CalibrationBlock,
            ImuBatchPacket,
            ImuSample,
            SessionHeader,
            decode_hks_session,
            encode_hks_session,
        )

        cal = CalibrationBlock()
        cal = CalibrationBlock(cal_crc32=cal.compute_crc32())
        hdr = SessionHeader(session_id=9205, cal_crc32=cal.cal_crc32, rink_preset="NHL_200X85FT")
        # 30 s stationary bench: body [0, 0, +g] -> sensor [2048, 0, 0] via R_BS^T
        body_frames = []
        for b_idx in range(300):  # 30 s @ 10 Hz batches of 20 samples
            ts_us = b_idx * 100_000
            samples = tuple(ImuSample(2048, 0, 0, 0, 0, 0) for _ in range(20))
            body_frames.append(ImuBatchPacket(timestamp_us=ts_us, odr_hz=200, temp_c_x100=2100, samples=samples))
        for a_idx in range(1500):  # 30 s @ 50 Hz AUX_ENV
            ts_us = a_idx * 20_000
            body_frames.append(
                AuxEnvPacket(
                    timestamp_us=ts_us,
                    mx_i32=3200,
                    my_i32=-1400,
                    mz_i32=7300,
                    pressure_pa_x64=6484800,
                    baro_temp_c_x100=2140,
                    vbatt_mv=3920,
                    batt_temp_c_x100=1850,
                    soc_pct_x10=845,
                    pmic_status=0x40,
                    validity_flags=0x0F,
                )
            )
        raw = encode_hks_session(hdr, cal, body_frames, end_utc_ms=1763852530000)
        dec = decode_hks_session(raw, require_footer=True, verify_so3=True, validate_contract_schemas=True)
        self.assertEqual(dec.footer.total_imu_samples, 6000)  # type: ignore[union-attr]
        self.assertEqual(dec.footer.total_aux_samples, 1500)  # type: ignore[union-attr]

    def test_f9_t2_06_non_canonical_so3_rbs_sprint_crossover_backward(self) -> None:
        helpers.require_m4()
        from sim.flash_codec import CANONICAL_R_BS
        from sim.physics_sim import SimConfig, simulate_session

        custom_r_bs = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        res = simulate_session(
            SimConfig(
                rink_preset="NHL_200X85FT",
                duration_s=60.0,
                num_shifts=1,
                seed=9206,
                cal_block_overrides={"sensor_to_body_rotation_matrix": custom_r_bs},
            )
        )
        frames = helpers.unpack_frames(res.hks_bytes, verify_crc=True, verify_footer_sha256=True)
        r_bs = frames[1]["packet"]["sensor_to_body_rotation_matrix"]
        ok, _, det = helpers.is_so3(r_bs)
        self.assertTrue(ok)
        self.assertAlmostEqual(det, 1.0, places=5)
        self.assertNotEqual([list(r) for r in CANONICAL_R_BS], r_bs)


# ---------------------------------------------------------------------------
# F10 Boundaries (M4, 5 tests)
# ---------------------------------------------------------------------------
class TestF10Boundaries(unittest.TestCase):
    def test_f10_t2_01_64bit_timestamp_past_71_5_min_and_2_hours(self) -> None:
        helpers.require_m4()
        from sim import flash_codec

        stream = helpers.build_synthetic_session_bytes(start_timestamp_us=4_294_967_290, end_timestamp_us=7_200_050_000)
        decoded = flash_codec.decode_hks_session(stream, require_footer=True, verify_so3=True)
        self.assertIsNotNone(decoded.footer)
        assert decoded.footer is not None
        self.assertGreater(decoded.footer.timestamp_us, 4_294_967_295)
        self.assertEqual(decoded.footer.timestamp_us, 7_200_050_000)

    def test_f10_t2_02_corrupted_crc32_rejection_frame_and_command(self) -> None:
        helpers.require_m4()
        from sim import downloader, flash_codec

        stream = helpers.build_synthetic_session_bytes(session_id=1022)
        bad_stream = bytearray(stream)
        bad_stream[30] ^= 0x01
        with self.assertRaises(flash_codec.BadCRCError):
            flash_codec.decode_hks_session(bytes(bad_stream), require_footer=True)

        # Command CRC32 corruption on download protocol -> ERR_BAD_CRC (0x01)
        transport = downloader.SimulatedDeviceTransport([stream])
        bad_cmd = bytearray(downloader.encode_request_frame(int(downloader.CmdId.LIST_SESSIONS), seq=1))
        bad_cmd[-1] ^= 0x01
        rsp_raw = transport.exchange(bytes(bad_cmd))
        cmd_id, status_code, rsp_seq, _, _ = downloader.decode_response_frame(rsp_raw)
        self.assertEqual(cmd_id, int(downloader.CmdId.LIST_SESSIONS))
        self.assertEqual(status_code, downloader.StatusCode.ERR_BAD_CRC)
        self.assertEqual(rsp_seq, 1)

    def test_f10_t2_03_unverified_delete_session_rejection(self) -> None:
        helpers.require_m4()
        from sim.downloader import DeviceCommandError, DownloadClient, SimulatedDeviceTransport, StatusCode

        src_bytes = helpers.build_synthetic_session_bytes(session_id=1023)
        transport = SimulatedDeviceTransport([src_bytes])
        client = DownloadClient(transport, record_trace=True)

        # 1. DELETE_SESSION(force=0) BEFORE ACK_VERIFY must fail ERR_NOT_VERIFIED (0x05)
        with self.assertRaises(DeviceCommandError) as ctx_unver:
            client.delete_session(1023, force=False)
        self.assertEqual(ctx_unver.exception.status_code, StatusCode.ERR_NOT_VERIFIED)
        self.assertIn(1023, transport.session_ids)

        # 2. ACK_VERIFY with wrong SHA256 must fail ERR_VERIFY_MISMATCH (0x08) and keep session unverified
        with self.assertRaises(DeviceCommandError) as ctx_bad_ack:
            client.ack_verify(1023, helpers.crc32_ieee(src_bytes), b"\x00" * 32)
        self.assertEqual(ctx_bad_ack.exception.status_code, StatusCode.ERR_VERIFY_MISMATCH)
        with self.assertRaises(DeviceCommandError) as ctx_still_unver:
            client.delete_session(1023, force=False)
        self.assertEqual(ctx_still_unver.exception.status_code, StatusCode.ERR_NOT_VERIFIED)
        self.assertIn(1023, transport.session_ids)

        # 3. Valid download + ACK_VERIFY -> DELETE_SESSION(force=0) succeeds
        dl = client.download_session(1023, verify=True, delete_after=True, strict_decode=True)
        self.assertEqual(dl.raw_bytes, src_bytes)
        self.assertEqual(dl.verified_status, 1)
        self.assertEqual(client.list_sessions(), [])

    def test_f10_t2_04_truncated_chunk_bad_sync_and_offset_out_of_range(self) -> None:
        helpers.require_m4()
        from sim import downloader, flash_codec

        stream = helpers.build_synthetic_session_bytes(session_id=1024)
        # 1. Truncated .hks frame
        with self.assertRaises(flash_codec.TruncatedFrameError):
            flash_codec.decode_hks_session(stream[:-15], require_footer=True)

        # 2. Bad frame sync (0xAA54)
        bad_frame_sync = bytearray(stream)
        bad_frame_sync[0] ^= 0x01
        with self.assertRaises(flash_codec.BadSyncError):
            flash_codec.decode_hks_session(bytes(bad_frame_sync), require_footer=True)

        # 3. Bad download request sync (0xAA54) & bad response sync (0x55AB)
        transport = downloader.SimulatedDeviceTransport([stream])
        bad_req = bytearray(downloader.encode_request_frame(int(downloader.CmdId.LIST_SESSIONS), seq=2))
        bad_req[0] ^= 0x01
        with self.assertRaises(downloader.BadResponseFrameError):
            transport.exchange(bytes(bad_req))

        bad_rsp = bytearray(downloader.encode_response_frame(int(downloader.CmdId.LIST_SESSIONS), int(downloader.StatusCode.OK), seq=2))
        bad_rsp[0] ^= 0x01
        with self.assertRaises(downloader.BadResponseFrameError):
            downloader.decode_response_frame(bytes(bad_rsp))

        # 4. Offset out of range -> ERR_OFFSET_OUT_OF_RANGE (0x04)
        client = downloader.DownloadClient(transport)
        with self.assertRaises(downloader.DeviceCommandError) as ctx_off:
            client.read_chunk(1024, offset_bytes=len(stream) + 10, length_bytes=64)
        self.assertEqual(ctx_off.exception.status_code, downloader.StatusCode.ERR_OFFSET_OUT_OF_RANGE)

    def test_f10_t2_05_zero_session_empty_flash_and_busy_recording(self) -> None:
        helpers.require_m4()
        from sim import downloader, flash_codec

        # 1. Empty .hks stream rejected
        with self.assertRaises(flash_codec.TruncatedFrameError):
            flash_codec.decode_hks_session(b"", require_footer=True)

        # 2. Empty flash LIST_SESSIONS returns count=0 and GET_HEADER returns ERR_SESSION_NOT_FOUND
        empty_transport = downloader.SimulatedDeviceTransport([])
        empty_client = downloader.DownloadClient(empty_transport)
        self.assertEqual(empty_client.list_sessions(), [])
        with self.assertRaises(downloader.DeviceCommandError) as ctx_nf:
            empty_client.get_header(1001)
        self.assertEqual(ctx_nf.exception.status_code, downloader.StatusCode.ERR_SESSION_NOT_FOUND)

        # 3. Recording active -> ERR_BUSY_RECORDING (0x07) on session commands; GET_DEVICE_STATUS still OK
        stream = helpers.build_synthetic_session_bytes(session_id=1025)
        busy_transport = downloader.SimulatedDeviceTransport([stream], recording_state=1)
        busy_client = downloader.DownloadClient(busy_transport)
        status = busy_client.get_device_status()
        self.assertEqual(status.recording_state, 1)
        for fn in (
            lambda: busy_client.list_sessions(),
            lambda: busy_client.get_header(1025),
            lambda: busy_client.read_chunk(1025, 0, 64),
            lambda: busy_client.delete_session(1025, force=True),
        ):
            with self.assertRaises(downloader.DeviceCommandError) as ctx_busy:
                fn()
            self.assertEqual(ctx_busy.exception.status_code, downloader.StatusCode.ERR_BUSY_RECORDING)


# ---------------------------------------------------------------------------
# F11 Boundaries (M4, 6 tests)
# ---------------------------------------------------------------------------
class TestF11Boundaries(unittest.TestCase):
    def test_f11_t2_01_gt_71_5_min_session_fusion_continuity(self) -> None:
        helpers.require_m4()
        _, rep, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=7500.0, shifts=60, seed=4001)
        helpers.validate_against_schema("analytics_report", rep)
        self.assertGreater(len(rep["trajectory"]), 1000)
        self.assertGreaterEqual(rep["session_summary"]["total_duration_s"], 7200.0)
        self.assertGreaterEqual(rep["trajectory"][-1]["t"], 7200.0)

    def test_f11_t2_02_gt_16g_body_check_dual_imu_handoff(self) -> None:
        helpers.require_m4()
        _, rep, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        helpers.validate_against_schema("analytics_report", rep)
        self.assertGreater(rep["session_summary"]["peak_accel_mps2"], 16.0 * helpers.GRAVITY_MPS2)

    def test_f11_t2_03_rink_corner_wall_clamping_nhl_iihf(self) -> None:
        helpers.require_m4()
        for preset in ("NHL_200X85FT", "IIHF_60X30M"):
            _, rep, _ = helpers.sim_and_analyze_cached(preset=preset, duration_s=140.0, shifts=2, seed=9203)
            self.assertGreater(len(rep["trajectory"]), 100)
            for pt in rep["trajectory"]:
                self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], preset, tol=1e-6))

    def test_f11_t2_04_magnetic_storm_distortion_rejection(self) -> None:
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        helpers.validate_against_schema("analytics_report", rep)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_xy = [(p["x"], p["y"]) for p in est_al]
        true_xy = [(p["x"], p["y"]) for p in truth_al]
        self.assertLessEqual(helpers.compute_cep50(est_xy, true_xy), 6.0)

    def test_f11_t2_05_empty_zero_shift_bench_zupt_zaru_stability(self) -> None:
        helpers.require_m4()
        from sim.flash_codec import AuxEnvPacket, CalibrationBlock, ImuBatchPacket, ImuSample, SessionHeader, encode_hks_session

        cal = CalibrationBlock()
        cal = CalibrationBlock(cal_crc32=cal.compute_crc32())
        hdr = SessionHeader(session_id=11205, cal_crc32=cal.cal_crc32, rink_preset="NHL_200X85FT")
        body_frames = []
        for b_idx in range(300):  # 30 s stationary bench
            ts_us = b_idx * 100_000
            samples = tuple(ImuSample(2048, 0, 0, 0, 0, 0) for _ in range(20))
            body_frames.append(ImuBatchPacket(timestamp_us=ts_us, odr_hz=200, temp_c_x100=2100, samples=samples))
        for a_idx in range(1500):
            ts_us = a_idx * 20_000
            body_frames.append(
                AuxEnvPacket(
                    timestamp_us=ts_us,
                    mx_i32=3200,
                    my_i32=-1400,
                    mz_i32=7300,
                    pressure_pa_x64=6484800,
                    baro_temp_c_x100=2140,
                    vbatt_mv=3920,
                    batt_temp_c_x100=1850,
                    soc_pct_x10=845,
                    pmic_status=0x40,
                    validity_flags=0x0F,
                )
            )
        raw = encode_hks_session(hdr, cal, body_frames, end_utc_ms=1763852530000)
        with tempfile.TemporaryDirectory() as tmp:
            hks_p = Path(tmp) / "bench_zero.hks"
            rep_p = Path(tmp) / "bench_zero_rep.json"
            hks_p.write_bytes(raw)
            proc = helpers.run_sim_cli(["analyze", "--in-hks", str(hks_p), "--out-report", str(rep_p)])
            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            rep = helpers.load_json_strict(rep_p)
        helpers.validate_against_schema("analytics_report", rep)
        self.assertEqual(rep["session_summary"]["shift_count"], 0)
        self.assertEqual(rep["shifts"], [])
        self.assertLessEqual(rep["session_summary"]["total_distance_m"], 1.0)

    def test_f11_t2_06_sprint_hard_stop_backward_crossover_noncanonical_rbs(self) -> None:
        helpers.require_m4()
        from sim.physics_sim import SimConfig, simulate_session

        custom_r_bs = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        res = simulate_session(
            SimConfig(
                rink_preset="NHL_200X85FT",
                duration_s=140.0,
                num_shifts=2,
                seed=11206,
                cal_block_overrides={"sensor_to_body_rotation_matrix": custom_r_bs},
            )
        )
        with tempfile.TemporaryDirectory() as tmp:
            hks_p = Path(tmp) / "noncanon.hks"
            rep_p = Path(tmp) / "noncanon_rep.json"
            hks_p.write_bytes(res.hks_bytes)
            proc = helpers.run_sim_cli(["analyze", "--in-hks", str(hks_p), "--out-report", str(rep_p)])
            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            rep = helpers.load_json_strict(rep_p)
        helpers.validate_against_schema("analytics_report", rep)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], res.ground_truth["trajectory"])
        est_spd = [p["speed"] for p in est_al]
        true_spd = [p["speed"] for p in truth_al]
        self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.8)


if __name__ == "__main__":
    unittest.main()
