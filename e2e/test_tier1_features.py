"""Tier 1 E2E Tests: Single-Feature Happy-Path & Contract Conformance (F1-F11, 55 tests)."""

from __future__ import annotations

import base64
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from e2e import helpers  # noqa: E402


# ---------------------------------------------------------------------------
# F1: System Specification (PROJECT_SPEC.md) — 5 tests
# ---------------------------------------------------------------------------
class TestF1SystemSpecification(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.spec = helpers.load_spec_text()
        cls.contract = helpers.load_contract()

    def test_f1_t1_01_spec_exists_and_header_identity(self) -> None:
        self.assertTrue(helpers.SPEC_PATH.is_file())
        self.assertGreater(len(self.spec.strip()), 1000)
        self.assertIn("HKS-SPEC-1.0.0", self.spec)
        self.assertIn("INTERFACE_CONTRACT.json", self.spec)
        self.assertIn("v1.0.0", self.spec)
        self.assertIn("docs/architecture.md", self.spec)

    def test_f1_t1_02_all_five_tracked_metrics_documented(self) -> None:
        for term in ("Speed", "Acceleration", "On-Ice 2D Position", "Shift Length", "Distance Skated"):
            self.assertIn(term, self.spec)
        for unit_or_bound in ("m/s", "0–200 g", "m/s^2", "CEP50", "±1.5 s", "±5%"):
            self.assertIn(unit_or_bound, self.spec)

    def test_f1_t1_03_anchor_free_zero_gps_declaration(self) -> None:
        self.assertIn("gps_enabled = false", self.spec)
        self.assertIn("external_anchors_enabled = false", self.spec)
        self.assertIn("zero reliance on GPS", self.spec)
        self.assertIn("6-Layer Drift-Bounding Pipeline", self.spec)

    def test_f1_t1_04_hardware_power_flash_packaging_locks(self) -> None:
        for token in (
            "52.0 × 36.0 × 10.5 mm",
            "~24 g",
            "<= 28 g",
            "IP67",
            "450 mAh",
            "292.5 mAh",
            "234.3 h",
            "91.3 h",
            "W25N01JW",
            "998",
            "6.76 h",
        ):
            self.assertIn(token, self.spec)

    def test_f1_t1_05_post_session_download_and_frames_cross_check(self) -> None:
        arch = helpers.load_arch_text()
        for token in (
            "Rink World Frame (`W`)",
            "Body / Torso Frame (`B`, FLU)",
            "Sensor Chip Frame (`S`)",
            "R_{BS}",
            "q_{WB}",
            "0xAA55",
            "0x55AA",
            "CRC32-IEEE",
            "0xEDB88320",
            "uint64_t timestamp_us",
            "LIST_SESSIONS",
            "GET_HEADER",
            "READ_CHUNK",
            "ACK_VERIFY",
            "DELETE_SESSION",
        ):
            self.assertIn(token, self.spec)
        for cmd_name in self.contract["download_protocol"]["commands"]:
            self.assertIn(cmd_name, arch)


# ---------------------------------------------------------------------------
# F2: Interface Contract (INTERFACE_CONTRACT.json) — 5 tests
# ---------------------------------------------------------------------------
class TestF2InterfaceContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = helpers.load_contract(use_cache=False)

    def test_f2_t1_01_strict_rfc8259_json_parse(self) -> None:
        raw_text = helpers.CONTRACT_PATH.read_text(encoding="utf-8")
        parsed = helpers.loads_json_strict(raw_text)
        self.assertIsInstance(parsed, dict)
        self.assertEqual(parsed["contract_version"], "1.0.0")
        self.assertEqual(parsed["project"], "hockey_sensor")
        for key in (
            "contract_version",
            "schema_standard",
            "project",
            "constants",
            "coordinate_frames",
            "units_and_tolerances",
            "hardware_abstraction",
            "binary_flash_format",
            "download_protocol",
            "rink_geometry_presets",
            "schemas",
            "examples",
        ):
            self.assertIn(key, parsed)

    def test_f2_t1_02_all_13_embedded_schemas_and_examples(self) -> None:
        expected = {
            "calibration_block",
            "flash_session_header",
            "imu_batch_packet",
            "high_g_batch_packet",
            "aux_env_packet",
            "event_marker_packet",
            "flash_session_footer",
            "download_command",
            "download_response",
            "analytics_trajectory_point",
            "analytics_shift_summary",
            "analytics_session_summary",
            "analytics_report",
        }
        self.assertEqual(set(self.contract["schemas"].keys()), expected)
        self.assertEqual(set(self.contract["examples"].keys()), expected)
        for name in expected:
            with self.subTest(schema=name):
                helpers.validate_against_schema(name, self.contract["examples"][name])

    def test_f2_t1_03_constants_and_coordinate_frame_invariants(self) -> None:
        c = self.contract["constants"]
        self.assertFalse(c["gps_enabled"])
        self.assertFalse(c["external_anchors_enabled"])
        self.assertEqual(c["timestamp_width_bits"], 64)
        self.assertEqual(c["timestamp_type"], "uint64_t")
        self.assertEqual(c["sync_marker_u16"], "0xAA55")
        self.assertEqual(c["response_sync_marker_u16"], "0x55AA")
        self.assertEqual(c["crc32_polynomial_reflected"], "0xEDB88320")

        r_bs = self.contract["coordinate_frames"]["frames"]["S"]["canonical_R_BS"]
        ok, ortho_err, det = helpers.is_so3(r_bs)
        self.assertTrue(ok)
        self.assertLessEqual(ortho_err, 1e-5)
        self.assertAlmostEqual(det, 1.0, places=6)
        q_id = self.contract["coordinate_frames"]["frames"]["B"]["identity_quaternion_q_WB"]
        self.assertTrue(helpers.is_unit_quaternion(q_id))

    def test_f2_t1_04_binary_flash_format_header_and_7_message_types(self) -> None:
        bff = self.contract["binary_flash_format"]
        self.assertEqual(bff["frame_header_bytes"], 20)
        self.assertEqual(bff["frame_crc_bytes"], 4)
        self.assertEqual(struct.calcsize(helpers.FRAME_HDR_FMT), 20)
        ts_field = next(f for f in bff["header_fields"] if f["name"] == "timestamp_us")
        self.assertEqual((ts_field["c_type"], ts_field["offset"], ts_field["size"]), ("uint64_t", 8, 8))
        mtypes = bff["message_types"]
        self.assertEqual(len(mtypes), 7)
        self.assertEqual(struct.calcsize(helpers.SESSION_HEADER_FMT), mtypes["SESSION_HEADER"]["payload_bytes"])
        self.assertEqual(struct.calcsize(helpers.AUX_ENV_FMT), mtypes["AUX_ENV"]["payload_bytes"])
        self.assertEqual(struct.calcsize(helpers.EVENT_MARKER_FMT), mtypes["EVENT_MARKER"]["payload_bytes"])
        self.assertEqual(struct.calcsize(helpers.CALIBRATION_BLOCK_FMT), mtypes["CALIBRATION_BLOCK"]["payload_bytes"])
        self.assertEqual(struct.calcsize(helpers.SESSION_FOOTER_FMT), mtypes["SESSION_FOOTER"]["payload_bytes"])

    def test_f2_t1_05_download_protocol_commands_and_status_codes(self) -> None:
        dp = self.contract["download_protocol"]
        self.assertEqual(len(dp["commands"]), 7)
        self.assertEqual(len(set(dp["commands"].values())), 7)
        self.assertEqual(len(dp["status_codes"]), 10)
        self.assertEqual(len(set(dp["status_codes"].values())), 10)
        self.assertEqual(dp["max_chunk_bytes"], 4096)

        rep_defs = self.contract["schemas"]["analytics_report"]["$defs"]
        for sub in ("analytics_trajectory_point", "analytics_shift_summary", "analytics_session_summary"):
            standalone = {k: v for k, v in self.contract["schemas"][sub].items() if k not in ("$schema", "$id")}
            embedded = {k: v for k, v in rep_defs[sub].items() if k not in ("$schema", "$id")}
            self.assertEqual(standalone, embedded)


# ---------------------------------------------------------------------------
# F3: Contract Automated Validator (docs/validate_contract.py) — 5 tests
# ---------------------------------------------------------------------------
class TestF3ContractValidator(unittest.TestCase):
    def test_f3_t1_01_cli_default_run_exits_zero(self) -> None:
        proc = helpers.run_validate_contract_cli(extra_args=())
        self.assertEqual(proc.returncode, 0, msg=f"stderr={proc.stderr}\nstdout={proc.stdout}")
        self.assertIn("CONTRACT_VALID: INTERFACE_CONTRACT.json v1.0.0", proc.stdout)
        self.assertIn("OK", proc.stderr + proc.stdout)

    def test_f3_t1_02_cli_skip_tests_fast_path(self) -> None:
        proc = helpers.run_validate_contract_cli(extra_args=("--skip-tests",))
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)
        self.assertIn("13 schemas, 13 examples, semantic invariants OK", proc.stdout)

    def test_f3_t1_03_cli_explicit_contract_and_meta_schema_paths(self) -> None:
        proc = helpers.run_validate_contract_cli(
            extra_args=(
                "--contract",
                str(helpers.CONTRACT_PATH),
                "--meta-schema",
                str(helpers.META_SCHEMA_PATH),
                "--skip-tests",
            )
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)
        self.assertIn("CONTRACT_VALID:", proc.stdout)

    def test_f3_t1_04_unittest_positive_suite_in_module(self) -> None:
        from docs.validate_contract import TestInterfaceContractPositive

        suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestInterfaceContractPositive)
        with open(os.devnull, "w", encoding="utf-8") as devnull:
            result = unittest.TextTestRunner(stream=devnull, verbosity=0).run(suite)
        self.assertTrue(result.wasSuccessful(), msg=f"failures={result.failures}, errors={result.errors}")
        self.assertGreaterEqual(result.testsRun, 5)

    def test_f3_t1_05_python_api_validate_contract_files(self) -> None:
        contract, meta_schema = helpers.validate_contract_fast()
        self.assertEqual(contract["contract_version"], "1.0.0")
        self.assertIn("$schema", meta_schema)


# ---------------------------------------------------------------------------
# F4: Hardware EE Design, Pinmux & Zero-GPS Compliance (M2) — 5 tests
# ---------------------------------------------------------------------------
class TestF4HardwareEEDesign(unittest.TestCase):
    def test_f4_t1_01_ee_artifacts_exist_and_parse(self) -> None:
        m2 = helpers.require_m2()
        with m2["ee_bom"].open("r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertGreater(len(rows), 5)
        pinmux = helpers.load_json_strict(m2["pinmux"])
        self.assertIsInstance(pinmux, dict)
        schematic_txt = m2["schematic_spec"].read_text(encoding="utf-8")
        self.assertGreater(len(schematic_txt.strip()), 200)

    def test_f4_t1_02_core_silicon_presence_in_bom(self) -> None:
        m2 = helpers.require_m2()
        bom_txt = m2["ee_bom"].read_text(encoding="utf-8")
        for part in (
            "nRF5340",
            "nPM1300",
            "ICM-42688-P",
            "ADXL372",
            "MMC5983MA",
            "BMP581",
            "W25N01JW",
            "502535",
            "TPD4E05U06",
            "2450AT18B",
        ):
            self.assertIn(part, bom_txt, msg=f"Missing required COTS part '{part}' in hardware/ee/bom.csv")

    def test_f4_t1_03_zero_gps_zero_uwb_zero_anchors_in_bom_and_pinmux(self) -> None:
        m2 = helpers.require_m2()
        hits = helpers.scan_zero_gps_compliance(m2["ee_bom"], m2["pinmux"])
        self.assertEqual(hits, [], msg=f"Forbidden GPS/UWB/anchor parts found: {hits}")
        pinmux = helpers.load_json_strict(m2["pinmux"])
        inv = pinmux.get("invariants", pinmux)
        if "uses_gps" in inv:
            self.assertIs(inv["uses_gps"], False)
        if "uses_external_anchors" in inv:
            self.assertIs(inv["uses_external_anchors"], False)

    def test_f4_t1_04_bus_pinmux_and_i2c_addresses_match_contract(self) -> None:
        m2 = helpers.require_m2()
        pinmux = helpers.load_json_strict(m2["pinmux"])
        buses = pinmux.get("buses", pinmux)
        for bus_name in ("SPIM4", "SPIM1", "TWIM0", "TWIM1", "QSPI", "USBD"):
            self.assertIn(bus_name, buses)
        pin_json = json.dumps(buses)
        self.assertIn("ICM-42688-P", json.dumps(buses["SPIM4"]))
        self.assertIn("ADXL372", json.dumps(buses["SPIM1"]))
        self.assertIn("0x30", json.dumps(buses["TWIM0"]))
        self.assertTrue("0x47" in json.dumps(buses["TWIM0"]) or "0x46" in json.dumps(buses["TWIM0"]))
        self.assertIn("0x6B", json.dumps(buses["TWIM1"]))
        self.assertIn("W25N01JW", pin_json)

    def test_f4_t1_05_unified_1v8_rail_esd_usb_cc_compliance(self) -> None:
        m2 = helpers.require_m2()
        pinmux_txt = m2["pinmux"].read_text(encoding="utf-8")
        schem_txt = m2["schematic_spec"].read_text(encoding="utf-8")
        bom_txt = m2["ee_bom"].read_text(encoding="utf-8")
        combined = pinmux_txt + "\n" + schem_txt + "\n" + bom_txt
        self.assertIn("1.8", combined)
        self.assertIn("TPD4E05U06", combined)
        self.assertTrue("5.1" in combined or "5100" in combined)
        self.assertIn("NTC", combined)


# ---------------------------------------------------------------------------
# F5: Cold-Rated Power & Flash Storage Budget (M2) — 5 tests
# ---------------------------------------------------------------------------
class TestF5ColdPowerAndFlashBudget(unittest.TestCase):
    def test_f5_t1_01_hardware_unit_tests_pass(self) -> None:
        helpers.require_m2()
        proc = helpers.run_hardware_unit_tests_cached()
        self.assertEqual(proc.returncode, 0, msg=f"stdout={proc.stdout}\nstderr={proc.stderr}")

    def test_f5_t1_02_power_budget_cli_json_output(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        for key in (
            "derated_capacity_mah",
            "nominal_runtime_h",
            "worst_case_runtime_h",
            "flash_nominal_hours",
            "flash_cont_400hz_hours",
        ):
            self.assertIsNotNone(budget.get(key), msg=f"Missing '{key}' in normalized power_budget.py --json output")

    def test_f5_t1_03_cold_derated_battery_runtime_exceeds_2_hours(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        self.assertGreater(budget["derated_capacity_mah"], 0.0)
        self.assertLessEqual(budget["derated_capacity_mah"], 306.0)
        self.assertGreaterEqual(budget["nominal_runtime_h"], 2.0)
        self.assertGreaterEqual(budget["worst_case_runtime_h"], 2.0)

    def test_f5_t1_04_flash_ring_capacity_exceeds_2_hours(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        if budget["flash_usable_bytes"] is not None:
            self.assertGreaterEqual(budget["flash_usable_bytes"], 130_000_000)
        self.assertGreaterEqual(budget["flash_nominal_hours"], 2.0)
        self.assertGreaterEqual(budget["flash_cont_400hz_hours"], 2.0)

    def test_f5_t1_05_ble_and_usb_offload_energy_and_time_budget(self) -> None:
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        if budget["ble_offload_mah"] is not None:
            self.assertLess(budget["ble_offload_mah"], 5.0)
            self.assertGreater(budget["ble_offload_mah"], 0.0)
        proc = subprocess.run(
            [sys.executable, str(helpers.require_m2()["power_budget"])],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)


# ---------------------------------------------------------------------------
# F6: Low-Profile Under-Jersey Mechanical Spec (M2) — 5 tests
# ---------------------------------------------------------------------------
class TestF6LowProfileMechanicalSpec(unittest.TestCase):
    def test_f6_t1_01_mechanical_artifacts_exist_and_parse(self) -> None:
        m2 = helpers.require_m2()
        dims = helpers.load_json_strict(m2["dimensions"])
        self.assertIsInstance(dims, dict)
        with m2["mech_bom"].open("r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertGreater(len(rows), 2)
        enc = m2["enclosure_spec"].read_text(encoding="utf-8")
        self.assertGreater(len(enc.strip()), 200)

    def test_f6_t1_02_envelope_dimensions_match_contract(self) -> None:
        m2 = helpers.require_m2()
        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        self.assertLessEqual(dims["length_mm"], 52.0)
        self.assertLessEqual(dims["width_mm"], 36.0)
        self.assertLessEqual(dims["thickness_mm"], 10.5)

    def test_f6_t1_03_mass_and_ip67_rating_compliance(self) -> None:
        m2 = helpers.require_m2()
        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        self.assertLessEqual(dims["mass_g"], 28.0)
        self.assertEqual(dims["ip_rating"], "IP67")

    def test_f6_t1_04_under_jersey_retention_and_materials(self) -> None:
        m2 = helpers.require_m2()
        combined = m2["enclosure_spec"].read_text(encoding="utf-8") + "\n" + m2["mech_bom"].read_text(encoding="utf-8")
        self.assertIn("PC+ABS", combined)
        self.assertIn("TPU", combined)
        self.assertIn("70A", combined)
        self.assertTrue("STERNUM_POCKET" in combined or "sternum" in combined.lower())
        self.assertTrue("CHEST_STRAP" in combined or "chest strap" in combined.lower())

    def test_f6_t1_05_hydrophobic_baro_vent_and_ip67_usb_plug(self) -> None:
        m2 = helpers.require_m2()
        combined = m2["enclosure_spec"].read_text(encoding="utf-8") + "\n" + m2["mech_bom"].read_text(encoding="utf-8")
        self.assertIn("ePTFE", combined)
        self.assertIn("USB", combined)
        self.assertIn("IP67", combined)


# ---------------------------------------------------------------------------
# F7: Portable C11 Firmware Core & HAL (M3) — 5 tests
# ---------------------------------------------------------------------------
class TestF7FirmwareCoreAndHal(unittest.TestCase):
    def test_f7_t1_01_strict_c11_clean_build_and_test(self) -> None:
        helpers.require_m3()
        with tempfile.TemporaryDirectory() as tmp:
            fw_copy = Path(tmp) / "firmware"
            shutil.copytree(helpers.FIRMWARE_DIR, fw_copy, ignore=shutil.ignore_patterns("build"))
            proc_clean = subprocess.run(
                ["make", "-C", str(fw_copy), "clean"],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc_clean.returncode, 0, msg=proc_clean.stderr)
            proc_all_test = subprocess.run(
                ["make", "-C", str(fw_copy), "all", "test"],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                proc_all_test.returncode,
                0,
                msg=f"stdout={proc_all_test.stdout}\nstderr={proc_all_test.stderr}",
            )
            self.assertIn("ALL 13 M3 TESTS PASSED", proc_all_test.stdout)
            self.assertTrue((fw_copy / "build" / "hks_fw_sim_cli").is_file())
        # Workspace binary must remain intact for subsequent tests
        self.assertTrue((helpers.FIRMWARE_DIR / "build" / "hks_fw_sim_cli").is_file())

    def test_f7_t1_02_crc32_ieee_vector_and_frame_verification(self) -> None:
        helpers.require_m3()
        proc_test = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc_test.returncode, 0, msg=proc_test.stderr)
        self.assertIn("test_crc32_and_sha256_vectors", proc_test.stdout)

        raw, info, gen = helpers.run_fw_sim_cached(usb_selftest=True)
        self.assertEqual(gen.returncode, 0, msg=gen.stderr)
        self.assertEqual(helpers.crc32_ieee(raw), info["file_crc32"])
        self.assertEqual(hashlib.sha256(raw).hexdigest().lower(), info["file_sha256"].lower())
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertGreater(len(frames), 10)

    def test_f7_t1_03_flash_ring_session_write_and_readback(self) -> None:
        helpers.require_m3()
        raw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        self.assertEqual(info["nop_violations"], 0)
        self.assertEqual(len(raw), info["total_bytes"])
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual(frames[0]["header"]["msg_type"], helpers.MSG_SESSION_HEADER)
        self.assertEqual(frames[1]["header"]["msg_type"], helpers.MSG_CALIBRATION_BLOCK)
        self.assertEqual(frames[-1]["header"]["msg_type"], helpers.MSG_SESSION_FOOTER)
        self.assertEqual(frames[-1]["packet"]["total_bytes"], len(raw))

    def test_f7_t1_04_download_protocol_happy_path_state_machine(self) -> None:
        helpers.require_m3()
        helpers.require_m4()
        with tempfile.TemporaryDirectory() as tmp:
            hks_path = Path(tmp) / "f7_dl.hks"
            info_path = Path(tmp) / "f7_info.json"
            tx_path = Path(tmp) / "tx.json"
            proc = helpers.run_fw_cli(["--out", str(hks_path), "--info-json", str(info_path), "--usb-selftest"])
            self.assertEqual(proc.returncode, 0, msg=f"stdout={proc.stdout}\nstderr={proc.stderr}")
            self.assertIn("usb_selftest=PASS", proc.stdout)
            self.assertTrue(hks_path.is_file())
            self.assertTrue(info_path.is_file())
            info = helpers.load_json_strict(info_path)
            self.assertEqual(info["nop_violations"], 0)

            # Verify C-generated .hks through opaque-box sim.cli download + trace
            dl_hks = Path(tmp) / "dl.hks"
            dl = helpers.run_sim_cli(
                [
                    "download",
                    "--transport",
                    "sim",
                    "--hks-source",
                    str(hks_path),
                    "--out-hks",
                    str(dl_hks),
                    "--delete-after",
                    "--out-trace",
                    str(tx_path),
                ]
            )
            self.assertEqual(dl.returncode, 0, msg=dl.stderr)
            self.assertTrue(tx_path.is_file())
            tx = helpers.load_json_strict(tx_path)
            self.assertIsInstance(tx, list)
            self.assertGreaterEqual(len(tx), 4)
            for pair in tx:
                helpers.validate_against_schema("download_command", pair["command"])
                helpers.validate_against_schema("download_response", pair["response"])
            self.assertEqual(dl_hks.read_bytes(), hks_path.read_bytes())

    def test_f7_t1_05_sensor_rbs_adxl372_shift_and_impact_events(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        mtypes = {f["header"]["msg_type"] for f in frames}
        self.assertIn(helpers.MSG_HIGH_G_BATCH, mtypes)
        self.assertIn(helpers.MSG_EVENT_MARKER, mtypes)
        ok_so3, _, det = helpers.is_so3(frames[1]["packet"]["sensor_to_body_rotation_matrix"])
        self.assertTrue(ok_so3)
        self.assertAlmostEqual(det, 1.0, places=5)
        evt_codes = {f["packet"]["event_code"] for f in frames if f["header"]["msg_type"] == helpers.MSG_EVENT_MARKER}
        self.assertIn("IMPACT_SPIKE", evt_codes)
        self.assertTrue("SHIFT_HINT_ON" in evt_codes or "SHIFT_HINT_OFF" in evt_codes)
        hg_frames = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_HIGH_G_BATCH]
        peak_hg = max(
            math.sqrt(s["ax_i16"] ** 2 + s["ay_i16"] ** 2 + s["az_i16"] ** 2) * helpers.ADXL372_SCALE_MPS2_PER_LSB
            for f in hg_frames
            for s in f["packet"]["samples"]
        )
        self.assertGreater(peak_hg, 16.0 * helpers.GRAVITY_MPS2)


# ---------------------------------------------------------------------------
# F8: Firmware Unit Tests & C Binary .hks CLI Generator (M3) — 5 tests
# ---------------------------------------------------------------------------
class TestF8FirmwareUnitTestsAndCli(unittest.TestCase):
    def test_f8_t1_01_firmware_make_test_deterministic_pass(self) -> None:
        helpers.require_m3()
        proc = helpers.run_firmware_make_test_cached()
        self.assertEqual(proc.returncode, 0, msg=f"stdout={proc.stdout}\nstderr={proc.stderr}")
        self.assertIn("ALL 13 M3 TESTS PASSED", proc.stdout)

    def test_f8_t1_02_c_cli_hks_generation_and_framing(self) -> None:
        helpers.require_m3()
        raw, info, gen = helpers.run_fw_sim_cached(usb_selftest=True)
        self.assertEqual(gen.returncode, 0, msg=gen.stderr)
        self.assertIn("HKS_SIM_CLI_OK", gen.stdout)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual(frames[0]["frame_bytes"], 104)
        self.assertEqual(frames[0]["packet"]["magic"], "HKS1")
        self.assertEqual(frames[1]["frame_bytes"], 312)
        self.assertEqual(frames[-1]["frame_bytes"], 96)
        self.assertGreater(info["imu_frames"], 0)
        self.assertGreater(info["high_g_frames"], 0)
        self.assertGreater(info["aux_frames"], 0)
        self.assertGreater(info["event_frames"], 0)

    def test_f8_t1_03_c_to_python_schema_validation_all_7_packets(self) -> None:
        helpers.require_m3()
        schema_map = {
            1: "flash_session_header",
            2: "imu_batch_packet",
            3: "high_g_batch_packet",
            4: "aux_env_packet",
            5: "event_marker_packet",
            6: "calibration_block",
            7: "flash_session_footer",
        }
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        seen = set()
        for f in frames:
            mt = f["header"]["msg_type"]
            seen.add(mt)
            helpers.validate_against_schema(schema_map[mt], f["packet"])
        self.assertEqual(seen, {1, 2, 3, 4, 5, 6, 7})

    def test_f8_t1_04_footer_pre_footer_sha256_and_sample_counts(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        footer_offset = frames[-1]["offset"]
        self.assertEqual(
            frames[-1]["packet"]["pre_footer_sha256"].lower(),
            hashlib.sha256(raw[:footer_offset]).hexdigest().lower(),
        )
        imu_sum = sum(f["packet"]["sample_count"] for f in frames if f["header"]["msg_type"] == 2)
        hg_sum = sum(f["packet"]["sample_count"] for f in frames if f["header"]["msg_type"] == 3)
        aux_sum = sum(1 for f in frames if f["header"]["msg_type"] == 4)
        self.assertEqual(frames[-1]["packet"]["total_imu_samples"], imu_sum)
        self.assertEqual(frames[-1]["packet"]["total_high_g_samples"], hg_sum)
        self.assertEqual(frames[-1]["packet"]["total_aux_samples"], aux_sum)
        self.assertEqual(frames[-1]["packet"]["total_bytes"], len(raw))

    def test_f8_t1_05_cross_preset_and_calibration_cal_crc32_parity(self) -> None:
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        preset = frames[0]["packet"]["rink_preset"]
        self.assertIn(preset, ("NHL_200X85FT", "IIHF_60X30M", "CUSTOM"))
        self.assertEqual(frames[0]["packet"]["cal_crc32"], frames[1]["packet"]["cal_crc32"])
        cal_payload = frames[1]["raw_frame"][helpers.FRAME_HEADER_BYTES : -helpers.FRAME_CRC_BYTES]
        self.assertEqual(helpers.crc32_ieee(cal_payload[:284]), frames[1]["packet"]["cal_crc32"])
        gate = frames[0]["packet"]["bench_gate_xy_m"]
        self.assertTrue(helpers.is_inside_rink(gate[0], gate[1], preset, tol=0.05))


# ---------------------------------------------------------------------------
# F9: Physics-Based Hockey Rink Simulator (M4) — 5 tests
# ---------------------------------------------------------------------------
class TestF9PhysicsRinkSimulator(unittest.TestCase):
    def test_f9_t1_01_multi_shift_nominal_session_nhl(self) -> None:
        helpers.require_m4()
        _, _, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        self.assertEqual(len(truth["shifts"]), 2)
        self.assertGreater(len(truth["trajectory"]), 100)
        for pt in truth["trajectory"]:
            self.assertTrue(helpers.is_unit_quaternion(pt["q_WB"]))
            self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], "NHL_200X85FT", torso_margin_m=0.25, tol=1e-4))
            self.assertAlmostEqual(pt["speed"], math.hypot(pt["vx"], pt["vy"]), places=3)

    def test_f9_t1_02_kinematic_frame_consistency_s_b_w_gravity(self) -> None:
        helpers.require_m4()
        with tempfile.TemporaryDirectory() as tmp:
            hks = Path(tmp) / "init.hks"
            truth_p = Path(tmp) / "truth.json"
            proc = helpers.run_sim_cli(
                [
                    "simulate",
                    "--preset",
                    "NHL_200X85FT",
                    "--duration-s",
                    "30",
                    "--shifts",
                    "1",
                    "--seed",
                    "101",
                    "--out-hks",
                    str(hks),
                    "--out-truth",
                    str(truth_p),
                ]
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            frames = helpers.unpack_frames(hks.read_bytes(), verify_crc=True, verify_footer_sha256=True)
            r_bs = frames[1]["packet"]["sensor_to_body_rotation_matrix"]
            ok, _, _ = helpers.is_so3(r_bs)
            self.assertTrue(ok)
            scale = frames[0]["packet"]["accel_scale_mps2_per_lsb"]
            imu_frames = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_IMU_BATCH]
            self.assertGreater(len(imu_frames), 0)
            s0 = imu_frames[0]["packet"]["samples"][0]
            f_s = [s0["ax_i16"] * scale, s0["ay_i16"] * scale, s0["az_i16"] * scale]
            f_b = helpers.mat_vec_mul(r_bs, f_s)
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in f_b)), helpers.GRAVITY_MPS2, delta=0.8)

    def test_f9_t1_03_dual_imu_saturation_and_adxl372_burst(self) -> None:
        helpers.require_m4()
        raw, _, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        hg_frames = [f for f in frames if f["header"]["msg_type"] == helpers.MSG_HIGH_G_BATCH]
        self.assertGreater(len(hg_frames), 0)
        peak_hg_mps2 = max(
            math.sqrt(s["ax_i16"] ** 2 + s["ay_i16"] ** 2 + s["az_i16"] ** 2) * helpers.ADXL372_SCALE_MPS2_PER_LSB
            for f in hg_frames
            for s in f["packet"]["samples"]
        )
        self.assertGreater(peak_hg_mps2, 16.0 * helpers.GRAVITY_MPS2)

    def test_f9_t1_04_baro_bench_stepover_and_battery_discharge(self) -> None:
        helpers.require_m4()
        raw, _, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        aux = [f["packet"] for f in frames if f["header"]["msg_type"] == helpers.MSG_AUX_ENV]
        self.assertGreater(len(aux), 10)
        vbatt_vals = [a["vbatt_mv"] for a in aux]
        self.assertGreaterEqual(min(vbatt_vals), 2800)
        self.assertLessEqual(max(vbatt_vals), 4350)
        self.assertGreaterEqual(vbatt_vals[0], vbatt_vals[-1])
        pressures = [a["pressure_pa_x64"] / 64.0 for a in aux]
        self.assertGreater(max(pressures) - min(pressures), 1.5)

    def test_f9_t1_05_deterministic_seed_reproducibility(self) -> None:
        helpers.require_m4()
        with tempfile.TemporaryDirectory() as tmp:
            hks1, t1 = Path(tmp) / "s1.hks", Path(tmp) / "t1.json"
            hks2, t2 = Path(tmp) / "s2.hks", Path(tmp) / "t2.json"
            for hks, tp in ((hks1, t1), (hks2, t2)):
                proc = helpers.run_sim_cli(
                    [
                        "simulate",
                        "--preset",
                        "NHL_200X85FT",
                        "--duration-s",
                        "40",
                        "--shifts",
                        "1",
                        "--seed",
                        "777",
                        "--out-hks",
                        str(hks),
                        "--out-truth",
                        str(tp),
                    ]
                )
                self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            self.assertEqual(hks1.read_bytes(), hks2.read_bytes())


# ---------------------------------------------------------------------------
# F10: Post-Session Downloader & Binary Codec (M4) — 5 tests
# ---------------------------------------------------------------------------
class TestF10PostSessionDownloaderAndCodec(unittest.TestCase):
    def test_f10_t1_01_crc32_ieee_standard_test_vector(self) -> None:
        m4 = helpers.require_m4()
        from sim import flash_codec

        self.assertTrue(hasattr(flash_codec, "crc32_ieee"))
        self.assertEqual(flash_codec.crc32_ieee(b"123456789"), 0xCBF43926)
        self.assertTrue(m4["flash_codec"].is_file())

    def test_f10_t1_02_encode_decode_all_7_frame_types(self) -> None:
        helpers.require_m4()
        from sim import flash_codec

        stream = helpers.build_synthetic_session_bytes(session_id=1010, include_high_g=True)
        decoded = flash_codec.decode_hks_session(
            stream, require_footer=True, verify_so3=True, validate_contract_schemas=True
        )
        self.assertEqual(len(decoded.frames), 7)
        self.assertEqual(decoded.header.session_id, 1010)
        self.assertEqual(len(decoded.imu_batches), 1)
        self.assertEqual(len(decoded.high_g_batches), 1)
        self.assertEqual(len(decoded.aux_env_packets), 1)
        self.assertEqual(len(decoded.event_markers), 1)
        self.assertIsNotNone(decoded.footer)

    def test_f10_t1_03_full_download_happy_path_workflow(self) -> None:
        helpers.require_m4()
        with tempfile.TemporaryDirectory() as tmp:
            src_hks = Path(tmp) / "src.hks"
            dl_hks = Path(tmp) / "dl.hks"
            tx_json = Path(tmp) / "tx.json"
            src_hks.write_bytes(helpers.build_synthetic_session_bytes(session_id=1011))
            proc = helpers.run_sim_cli(
                [
                    "download",
                    "--transport",
                    "sim",
                    "--hks-source",
                    str(src_hks),
                    "--out-hks",
                    str(dl_hks),
                    "--delete-after",
                    "--out-trace",
                    str(tx_json),
                ]
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            self.assertIn("DOWNLOAD_OK", proc.stdout)
            self.assertEqual(dl_hks.read_bytes(), src_hks.read_bytes())
            self.assertTrue(tx_json.is_file())
            tx = helpers.load_json_strict(tx_json)
            self.assertGreaterEqual(len(tx), 4)
            for pair in tx:
                helpers.validate_against_schema("download_command", pair["command"])
                helpers.validate_against_schema("download_response", pair["response"])

    def test_f10_t1_04_adxl372_sign_preservation_no_double_shift(self) -> None:
        helpers.require_m4()
        from sim import flash_codec

        hg_pkt = {
            "sample_count": 4,
            "odr_hz": 3200,
            "trigger_mode": "IMPACT_BURST",
            "samples": [
                {"ax_i16": -2048, "ay_i16": -1, "az_i16": 0},
                {"ax_i16": 2047, "ay_i16": 100, "az_i16": -100},
                {"ax_i16": 0, "ay_i16": 0, "az_i16": 0},
                {"ax_i16": 1, "ay_i16": -2, "az_i16": 3},
            ],
        }
        frame = helpers.pack_frame(helpers.MSG_HIGH_G_BATCH, hg_pkt, timestamp_us=1000)
        fhdr, payload, fcrc, next_off = flash_codec.unpack_frame(frame, 0)
        self.assertEqual(next_off, len(frame))
        decoded_hg = flash_codec.unpack_high_g_batch_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
        self.assertEqual(decoded_hg.samples[0].ax_i16, -2048)
        self.assertEqual(decoded_hg.samples[1].ax_i16, 2047)
        _, hg_mps2 = decoded_hg.to_numpy(flash_codec.DEFAULT_HIGH_G_SCALE)
        self.assertAlmostEqual(float(hg_mps2[0, 0]), -2048.0 * helpers.ADXL372_SCALE_MPS2_PER_LSB, places=4)
        self.assertAlmostEqual(float(hg_mps2[1, 0]), 2047.0 * helpers.ADXL372_SCALE_MPS2_PER_LSB, places=4)

    def test_f10_t1_05_pre_footer_sha256_and_footer_counts(self) -> None:
        helpers.require_m4()
        from sim import flash_codec

        stream = helpers.build_synthetic_session_bytes(session_id=1012)
        decoded = flash_codec.decode_hks_session(stream, require_footer=True, verify_so3=True)
        raw_frames = helpers.unpack_frames(stream, verify_crc=True, verify_footer_sha256=True)
        footer_off = raw_frames[-1]["offset"]
        expected_pre_sha = hashlib.sha256(stream[:footer_off]).hexdigest().lower()

        self.assertIsNotNone(decoded.footer)
        assert decoded.footer is not None
        self.assertEqual(decoded.footer.pre_footer_sha256.lower(), expected_pre_sha)
        self.assertEqual(decoded.footer.total_imu_samples, sum(len(b.samples) for b in decoded.imu_batches))
        self.assertEqual(decoded.footer.total_high_g_samples, sum(len(b.samples) for b in decoded.high_g_batches))
        self.assertEqual(decoded.footer.total_aux_samples, len(decoded.aux_env_packets))
        self.assertEqual(decoded.footer.total_bytes, len(stream))

        # Tamper 1 byte of pre_footer_sha256 (with valid footer frame CRC32) -> must reject
        bad_ftr = dict(raw_frames[-1]["packet"])
        orig_sha = bad_ftr["pre_footer_sha256"]
        flipped_ch = "0" if orig_sha[0].lower() != "0" else "1"
        bad_ftr["pre_footer_sha256"] = flipped_ch + orig_sha[1:]
        bad_ftr_frame = helpers.pack_frame(
            helpers.MSG_SESSION_FOOTER,
            bad_ftr,
            flags=raw_frames[-1]["header"]["flags"],
            seq=raw_frames[-1]["header"]["seq"],
            timestamp_us=raw_frames[-1]["header"]["timestamp_us"],
        )
        tampered = stream[:footer_off] + bad_ftr_frame
        with self.assertRaises(flash_codec.Sha256MismatchError):
            flash_codec.decode_hks_session(tampered, require_footer=True, verify_so3=True)


# ---------------------------------------------------------------------------
# F11: Anchor-Free ES-EKF + RTS Fusion & 5 Tracked Metrics (M4) — 5 tests
# ---------------------------------------------------------------------------
class TestF11FusionPipelineAndMetrics(unittest.TestCase):
    def test_f11_t1_01_analytics_report_schema_and_semantics(self) -> None:
        helpers.require_m4()
        from docs.validate_contract import validate_analytics_report_semantics

        # Verify actual subprocess CLI execution on a short session
        with tempfile.TemporaryDirectory() as tmp:
            hks = Path(tmp) / "sub.hks"
            rep_p = Path(tmp) / "sub_rep.json"
            p_sim = helpers.run_sim_cli(
                ["simulate", "--preset", "NHL_200X85FT", "--duration-s", "40", "--shifts", "1", "--seed", "1101", "--out-hks", str(hks)],
                subprocess_mode=True,
            )
            self.assertEqual(p_sim.returncode, 0, msg=p_sim.stderr)
            p_ana = helpers.run_sim_cli(
                ["analyze", "--in-hks", str(hks), "--out-report", str(rep_p)],
                subprocess_mode=True,
            )
            self.assertEqual(p_ana.returncode, 0, msg=p_ana.stderr)
            rep_sub = helpers.load_json_strict(rep_p)
            helpers.validate_against_schema("analytics_report", rep_sub)
            validate_analytics_report_semantics(rep_sub, presets=helpers.load_contract()["rink_geometry_presets"])
            self.assertFalse(rep_sub["session_summary"]["gps_used"])
            self.assertFalse(rep_sub["session_summary"]["external_anchors_used"])

    def test_f11_t1_02_speed_rmse_and_distance_accuracy(self) -> None:
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_spd = [p["speed"] for p in est_al]
        true_spd = [p["speed"] for p in truth_al]
        self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.8)
        dist_err = helpers.compute_distance_pct_error(
            rep["session_summary"]["total_distance_m"],
            truth["session_summary"]["total_distance_m"],
        )
        self.assertLessEqual(dist_err, 12.0)

    def test_f11_t1_03_on_ice_2d_position_cep50(self) -> None:
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_xy = [(p["x"], p["y"]) for p in est_al]
        true_xy = [(p["x"], p["y"]) for p in truth_al]
        self.assertLessEqual(helpers.compute_cep50(est_xy, true_xy), 6.0)

    def test_f11_t1_04_shift_detection_f1_and_boundary_error(self) -> None:
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        est_s = [(s["start_s"], s["end_s"]) for s in rep["shifts"]]
        true_s = [(s["start_s"], s["end_s"]) for s in truth["shifts"]]
        f1, bnd = helpers.compute_shift_f1_and_boundary_error(est_s, true_s)
        self.assertGreaterEqual(f1, 0.95)
        self.assertLessEqual(bnd, 3.0)

    def test_f11_t1_05_acceleration_accuracy_and_c_py_e2e_parity(self) -> None:
        helpers.require_m4()
        helpers.require_m3()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        with tempfile.TemporaryDirectory() as tmp:
            fw_hks = Path(tmp) / "fw.hks"
            rep_p = Path(tmp) / "fw_report.json"
            fw_hks.write_bytes(raw)
            p2 = helpers.run_sim_cli(["analyze", "--in-hks", str(fw_hks), "--out-report", str(rep_p)])
            self.assertEqual(p2.returncode, 0, msg=p2.stderr)
            rep = helpers.load_json_strict(rep_p)
            helpers.validate_against_schema("analytics_report", rep)
            self.assertGreater(len(rep["trajectory"]), 1)
            self.assertGreater(rep["session_summary"]["peak_accel_mps2"], 16.0 * helpers.GRAVITY_MPS2)
            for pt in rep["trajectory"]:
                self.assertLessEqual(abs(pt["speed"] - math.hypot(pt["vx"], pt["vy"])), 1e-3)
                self.assertLessEqual(abs(pt["accel_mag"] - math.sqrt(pt["ax"] ** 2 + pt["ay"] ** 2 + pt["az"] ** 2)), 1e-3)


if __name__ == "__main__":
    unittest.main()
