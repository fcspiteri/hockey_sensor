"""Tier 3 E2E Tests: Cross-Feature Pairwise Integration Matrix (P01-P14, 14 tests)."""

from __future__ import annotations

import hashlib
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


class TestTier3Pairwise(unittest.TestCase):
    def test_p01_c_fw_hks_to_py_downloader_to_fusion_contract(self) -> None:
        """P01: F8 x F10 x F11 x F2 — C firmware CLI -> Python downloader -> Fusion -> analytics_report schema."""
        helpers.require_m3()
        helpers.require_m4()
        from docs.validate_contract import validate_analytics_report_semantics

        raw_fw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        with tempfile.TemporaryDirectory() as tmp:
            fw_hks = Path(tmp) / "fw.hks"
            dl_hks = Path(tmp) / "dl.hks"
            tx_json = Path(tmp) / "tx.json"
            rep_json = Path(tmp) / "report.json"
            fw_hks.write_bytes(raw_fw)

            p_dl = helpers.run_sim_cli(
                [
                    "download",
                    "--transport",
                    "sim",
                    "--hks-source",
                    str(fw_hks),
                    "--out-hks",
                    str(dl_hks),
                    "--out-trace",
                    str(tx_json),
                ]
            )
            self.assertEqual(p_dl.returncode, 0, msg=p_dl.stderr)
            self.assertEqual(dl_hks.read_bytes(), raw_fw)
            self.assertTrue(tx_json.is_file())
            tx = helpers.load_json_strict(tx_json)
            for pair in tx:
                helpers.validate_against_schema("download_command", pair["command"])
                helpers.validate_against_schema("download_response", pair["response"])

            p_ana = helpers.run_sim_cli(["analyze", "--in-hks", str(dl_hks), "--out-report", str(rep_json)])
            self.assertEqual(p_ana.returncode, 0, msg=p_ana.stderr)
            rep = helpers.load_json_strict(rep_json)
            helpers.validate_against_schema("analytics_report", rep)
            validate_analytics_report_semantics(rep, presets=helpers.load_contract()["rink_geometry_presets"])

    def test_p02_hardware_bom_pinmux_dimensions_contract_parity(self) -> None:
        """P02: F4 x F6 x F2 — Hardware BOM, pinmux, and mechanical dimensions match INTERFACE_CONTRACT."""
        m2 = helpers.require_m2()
        contract = helpers.load_contract()
        hw_c = contract["hardware_abstraction"]
        pinmux = helpers.load_json_strict(m2["pinmux"])
        buses = pinmux.get("buses", pinmux)
        hits = helpers.scan_zero_gps_compliance(m2["ee_bom"], m2["pinmux"])
        self.assertEqual(hits, [])

        self.assertIn(hw_c["sensors"]["primary_imu"]["part_number"], str(buses["SPIM4"]))
        self.assertIn(hw_c["sensors"]["high_g_imu"]["part_number"], str(buses["SPIM1"]))
        self.assertLessEqual(int(buses["SPIM1"].get("clock_hz", 8_000_000)), 10_000_000)
        self.assertIn(hw_c["sensors"]["magnetometer"]["i2c_addr_hex"], str(buses["TWIM0"]))
        self.assertIn(hw_c["sensors"]["barometer"]["i2c_addr_hex"], str(buses["TWIM0"]))
        self.assertIn(hw_c["pmic"]["i2c_addr_hex"], str(buses["TWIM1"]))
        self.assertIn(hw_c["flash"]["part_number"], str(buses["QSPI"]))

        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        env_c = hw_c["enclosure_envelope_mm"]
        self.assertLessEqual(dims["length_mm"], env_c["length_mm"])
        self.assertLessEqual(dims["width_mm"], env_c["width_mm"])
        self.assertLessEqual(dims["thickness_mm"], env_c["thickness_mm"])
        self.assertLessEqual(dims["mass_g"], 28.0)
        self.assertEqual(dims["ip_rating"], env_c["ip_rating"])

    def test_p03_shift_detection_distance_and_high_g_body_checks(self) -> None:
        """P03: F9 x F11 x F2 — Body checks (>16 g) preserve shift continuity & distance accuracy."""
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        helpers.validate_against_schema("analytics_report", rep)
        self.assertGreater(rep["session_summary"]["peak_accel_mps2"], 16.0 * helpers.GRAVITY_MPS2)
        est_s = [(s["start_s"], s["end_s"]) for s in rep["shifts"]]
        true_s = [(s["start_s"], s["end_s"]) for s in truth["shifts"]]
        f1, _ = helpers.compute_shift_f1_and_boundary_error(est_s, true_s)
        self.assertGreaterEqual(f1, 0.95)
        self.assertLessEqual(
            helpers.compute_distance_pct_error(
                rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
            ),
            12.0,
        )

    def test_p04_cold_power_flash_budget_and_2h_plus_logging(self) -> None:
        """P04: F5 x F7 x F9 — Cold 0C power/flash budgets match >2h simulated .hks consumption."""
        helpers.require_m2()
        helpers.require_m3()
        helpers.require_m4()
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        self.assertGreaterEqual(budget["worst_case_runtime_h"], 2.0)
        raw, _, _ = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=7500.0, shifts=60, seed=4001)
        self.assertLessEqual(len(raw), budget["flash_usable_bytes"] or helpers.FLASH_USABLE_RING_BYTES)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertGreater(frames[-1]["header"]["timestamp_us"], 0xFFFFFFFF)

    def test_p05_rink_clamping_mag_gating_and_zupt_bench_resets(self) -> None:
        """P05: F9 x F11 x F2 — Corner clamping, mag anomaly gating, and bench ZUPT keep CEP50 <= 6.0 m."""
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        for pt in rep["trajectory"]:
            self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], "NHL_200X85FT", tol=1e-6))
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_xy = [(p["x"], p["y"]) for p in est_al]
        true_xy = [(p["x"], p["y"]) for p in truth_al]
        self.assertLessEqual(helpers.compute_cep50(est_xy, true_xy), 6.0)

    def test_p06_c_to_python_binary_frame_and_crc_sha256_parity(self) -> None:
        """P06: F7 x F8 x F10 — C binary frame generator <-> Python flash_codec exact parity."""
        helpers.require_m3()
        helpers.require_m4()
        from sim import flash_codec

        raw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        ref_frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        decoded = flash_codec.decode_hks_session(
            raw, require_footer=True, verify_so3=True, validate_contract_schemas=True
        )
        self.assertEqual(len(decoded.frames), len(ref_frames))
        self.assertEqual(decoded.header.session_id, info["session_id"])
        self.assertEqual(decoded.file_crc32, info["file_crc32"])
        self.assertEqual(decoded.file_sha256.hex(), info["file_sha256"])
        self.assertEqual(flash_codec.crc32_ieee(b"123456789"), 0xCBF43926)

    def test_p07_sensor_to_body_rbs_and_adxl372_shift_parity(self) -> None:
        """P07: F7 x F10 x F11 — Canonical R_BS and ADXL372 >>4 alignment across C, codec, and fusion."""
        helpers.require_m3()
        helpers.require_m4()
        raw, _, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        r_bs = frames[1]["packet"]["sensor_to_body_rotation_matrix"]
        ok, _, det = helpers.is_so3(r_bs)
        self.assertTrue(ok)
        self.assertAlmostEqual(det, 1.0, places=5)
        with tempfile.TemporaryDirectory() as tmp:
            hks = Path(tmp) / "p07.hks"
            rep_p = Path(tmp) / "r.json"
            hks.write_bytes(raw)
            p2 = helpers.run_sim_cli(["analyze", "--in-hks", str(hks), "--out-report", str(rep_p)])
            self.assertEqual(p2.returncode, 0, msg=p2.stderr)
            rep = helpers.load_json_strict(rep_p)
            helpers.validate_against_schema("analytics_report", rep)
            self.assertGreater(rep["session_summary"]["peak_accel_mps2"], 16.0 * helpers.GRAVITY_MPS2)

    def test_p08_download_protocol_ack_verify_gates_delete(self) -> None:
        """P08: F7 x F10 x F2 — ACK_VERIFY gating DELETE_SESSION across C/Python download protocol."""
        helpers.require_m3()
        helpers.require_m4()
        from sim.downloader import DeviceCommandError, DownloadClient, SimulatedDeviceTransport, StatusCode

        raw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        sid = int(info["session_id"])

        # 1. Verify DELETE_SESSION(force=0) BEFORE ACK_VERIFY is rejected with ERR_NOT_VERIFIED (0x05)
        transport = SimulatedDeviceTransport([raw])
        client = DownloadClient(transport, record_trace=True)
        with self.assertRaises(DeviceCommandError) as ctx:
            client.delete_session(sid, force=False)
        self.assertEqual(ctx.exception.status_code, StatusCode.ERR_NOT_VERIFIED)
        self.assertIn(sid, transport.session_ids)

        # 2. Complete download -> ACK_VERIFY -> DELETE_SESSION(force=0) succeeds
        dl = client.download_session(sid, chunk_size=4096, verify=True, delete_after=True, strict_decode=True)
        self.assertEqual(dl.raw_bytes, raw)
        self.assertEqual(dl.verified_status, 1)
        self.assertEqual(dl.hash_mode, 2)
        self.assertEqual(client.list_sessions(), [])

        # 3. Verify transcript via CLI trace file and validate every command/response against contract
        with tempfile.TemporaryDirectory() as tmp:
            tx_json = Path(tmp) / "tx.json"
            tx_json.write_text(json.dumps(client.trace, indent=2), encoding="utf-8")
            self.assertTrue(tx_json.is_file())
            tx = helpers.load_json_strict(tx_json)
            cmds = [pair["command"]["command"] for pair in tx]
            self.assertIn("DELETE_SESSION", cmds)
            self.assertIn("ACK_VERIFY", cmds)
            # First DELETE_SESSION (unverified) must record ERR_NOT_VERIFIED; final DELETE_SESSION must be OK
            del_statuses = [pair["response"]["status"] for pair in tx if pair["command"]["command"] == "DELETE_SESSION"]
            self.assertEqual(del_statuses[0], "ERR_NOT_VERIFIED")
            self.assertEqual(del_statuses[-1], "OK")
            for pair in tx:
                helpers.validate_against_schema("download_command", pair["command"])
                helpers.validate_against_schema("download_response", pair["response"])

    def test_p09_icm_saturation_baro_stepover_and_shift_fsm(self) -> None:
        """P09: F9 x F11 — Baro step-over + mid-shift >16g impact preserve shift detection."""
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        f1, bnd = helpers.compute_shift_f1_and_boundary_error(
            [(s["start_s"], s["end_s"]) for s in rep["shifts"]],
            [(s["start_s"], s["end_s"]) for s in truth["shifts"]],
        )
        self.assertGreaterEqual(f1, 0.95)
        self.assertLessEqual(bnd, 3.0)

    def test_p10_nhc_and_stride_speed_vs_passive_gliding(self) -> None:
        """P10: F9 x F11 — NHC + stride-cadence speed coupling vs passive gliding and sharp crossover turns."""
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=20261122)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_spd = [p["speed"] for p in est_al]
        true_spd = [p["speed"] for p in truth_al]
        self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.8)
        for sh in rep["shifts"]:
            self.assertLessEqual(sh["avg_speed_mps"], sh["peak_speed_mps"])

    def test_p11_packaging_thermal_and_cold_jeita_spec_consistency(self) -> None:
        """P11: F1 x F3 x F5 x F6 — Cold 0C JEITA NTC, ohmic sag, 502535 cell, and ePTFE vent consistency."""
        m2 = helpers.require_m2()
        helpers.validate_contract_fast()
        budget = helpers.extract_power_budget_metrics(helpers.run_power_budget_json())
        self.assertGreaterEqual(budget["worst_case_runtime_h"], 2.0)
        if budget["ohmic_sag_mv"] is not None:
            self.assertLess(budget["ohmic_sag_mv"], 5.0)
        schem = m2["schematic_spec"].read_text(encoding="utf-8")
        enc = m2["enclosure_spec"].read_text(encoding="utf-8")
        self.assertIn("NTC", schem)
        self.assertIn("ePTFE", enc)
        dims = helpers.extract_mechanical_dims(helpers.load_json_strict(m2["dimensions"]))
        self.assertLessEqual(dims["thickness_mm"], 10.5)

    def test_p12_multi_session_ring_and_corrupt_chunk_retry(self) -> None:
        """P12: F7 x F8 x F10 — Corrupt chunk injection triggers retry and yields bit-exact .hks."""
        helpers.require_m3()
        helpers.require_m4()
        from sim.downloader import DownloadClient, SimulatedDeviceTransport

        raw_fw, info, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        sid = int(info["session_id"])
        transport = SimulatedDeviceTransport([raw_fw])
        transport.inject_request_bad_crc(1)
        transport.corrupt_next_response_crc(1)
        transport.truncate_next_response(1)
        transport.corrupt_next_response_seq(1)

        client = DownloadClient(transport, max_retries=3, record_trace=True)
        dl = client.download_session(sid, chunk_size=2048, verify=True, delete_after=True, strict_decode=True)
        self.assertEqual(dl.raw_bytes, raw_fw)
        self.assertEqual(dl.verified_status, 1)
        self.assertEqual(client.list_sessions(), [])
        self.assertGreaterEqual(len(client.trace), 4)

    def test_p13_rink_preset_switching_and_bench_gate_prior(self) -> None:
        """P13: F2 x F9 x F11 — NHL_200X85FT, IIHF_60X30M, CUSTOM presets & bench-gate anchors."""
        helpers.require_m4()
        presets = helpers.load_contract()["rink_geometry_presets"]
        for preset in ("NHL_200X85FT", "IIHF_60X30M", "CUSTOM"):
            seed = 9203 if preset in ("NHL_200X85FT", "IIHF_60X30M") else 3013
            raw, rep, _ = helpers.sim_and_analyze_cached(preset=preset, duration_s=140.0, shifts=2, seed=seed)
            frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
            self.assertEqual(frames[0]["packet"]["rink_preset"], preset)
            self.assertEqual(frames[0]["packet"]["bench_gate_xy_m"], presets[preset]["default_bench_gate_xy_m"])
            self.assertGreater(len(rep["trajectory"]), 100)
            for pt in rep["trajectory"]:
                self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], preset, tol=1e-6))

    def test_p14_pipeline_determinism_and_report_semantic_integrity(self) -> None:
        """P14: F10 x F11 x F2 — Repeated download+analyze produces identical valid analytics_report."""
        helpers.require_m4()
        with tempfile.TemporaryDirectory() as tmp:
            hks = Path(tmp) / "p14.hks"
            truth_p = Path(tmp) / "t.json"
            p1 = helpers.run_sim_cli(
                [
                    "simulate",
                    "--preset",
                    "NHL_200X85FT",
                    "--duration-s",
                    "60",
                    "--shifts",
                    "1",
                    "--seed",
                    "3014",
                    "--out-hks",
                    str(hks),
                    "--out-truth",
                    str(truth_p),
                ]
            )
            self.assertEqual(p1.returncode, 0, msg=p1.stderr)
            r1_p, r2_p = Path(tmp) / "r1.json", Path(tmp) / "r2.json"
            for rp in (r1_p, r2_p):
                pa = helpers.run_sim_cli(["analyze", "--in-hks", str(hks), "--out-report", str(rp)])
                self.assertEqual(pa.returncode, 0, msg=pa.stderr)
            r1 = helpers.load_json_strict(r1_p)
            r2 = helpers.load_json_strict(r2_p)
            self.assertEqual(r1, r2)
            helpers.validate_against_schema("analytics_report", r1)


if __name__ == "__main__":
    unittest.main()
