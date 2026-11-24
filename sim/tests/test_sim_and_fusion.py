"""Comprehensive unittest suite for sim.flash_codec, sim.downloader, sim.physics_sim, sim.fusion_pipeline, and sim.cli."""

from __future__ import annotations

import ast
import json
import math
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from jsonschema import Draft202012Validator

from docs.validate_contract import (
    is_inside_rink,
    validate_analytics_report_semantics,
    validate_packet_semantics,
)
from sim import DEFAULT_CONTRACT_PATH, REPO_ROOT, load_contract
from sim.cli import main as cli_main
from sim.downloader import (
    CmdId,
    DeviceCommandError,
    DownloadClient,
    SimulatedDeviceTransport,
    StatusCode,
)
from sim.flash_codec import (
    CANONICAL_R_BS,
    DEFAULT_HIGH_G_SCALE,
    AuxEnvPacket,
    BadCRCError,
    BadSyncError,
    CalibrationBlock,
    CalibrationCRCError,
    CalibrationMismatchError,
    EventMarkerPacket,
    FooterError,
    FrameFlag,
    HighGBatchPacket,
    HighGSample,
    ImuBatchPacket,
    ImuSample,
    InvalidFieldError,
    SessionHeader,
    Sha256MismatchError,
    TruncatedFrameError,
    crc32_ieee,
    decode_hks_session,
    encode_hks_session,
)
from sim.fusion_pipeline import AblationConfig, run_fusion
from sim.physics_sim import SimConfig, simulate_session


class TestFlashCodec(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract(DEFAULT_CONTRACT_PATH)

    def test_crc32_ieee_known_vector(self) -> None:
        self.assertEqual(crc32_ieee(b"123456789"), 0xCBF43926)

    def test_all_7_frame_types_round_trip_with_64bit_ts_and_adxl372_limits(self) -> None:
        cal = CalibrationBlock()
        cal_crc = cal.compute_crc32()
        cal = CalibrationBlock(cal_crc32=cal_crc)

        hdr = SessionHeader(
            session_id=2025,
            start_utc_ms=1763852500000,
            device_id=0x0102030405060708,
            cal_crc32=cal_crc,
            rink_preset="NHL_200X85FT",
            mount_location="STERNUM_POCKET",
            player_height_cm=183,
            bench_gate_xy_m=(-6.0, 12.954),
            flags=int(FrameFlag.UTC_SYNCED),
        )

        ts_gt_71_5_min = 7_200_000_000  # > 0xFFFFFFFF (4,294,967,295 us)
        imu = ImuBatchPacket(
            timestamp_us=ts_gt_71_5_min,
            odr_hz=200,
            temp_c_x100=2150,
            samples=(
                ImuSample(2048, -12, 4, 10, -20, 30),
                ImuSample(2055, -10, 6, 12, -18, 28),
            ),
            flags=int(FrameFlag.UTC_SYNCED | FrameFlag.ACCEL_SAT),
        )
        hg = HighGBatchPacket(
            timestamp_us=ts_gt_71_5_min + 5000,
            odr_hz=3200,
            trigger_mode="IMPACT_BURST",
            samples=(
                HighGSample(-2048, 0, 2047),
                HighGSample(-500, 340, 120),
            ),
            flags=int(FrameFlag.UTC_SYNCED | FrameFlag.HIGH_G_ACTIVE),
        )
        aux = AuxEnvPacket(
            timestamp_us=ts_gt_71_5_min + 20_000,
            mx_i32=3200,
            my_i32=-1400,
            mz_i32=7300,
            pressure_pa_x64=6484800,
            baro_temp_c_x100=2140,
            vbatt_mv=3920,
            batt_temp_c_x100=1850,
            soc_pct_x10=845,
            pmic_status=0x0140,
            validity_flags=0x0F,
        )
        evt = EventMarkerPacket(
            timestamp_us=ts_gt_71_5_min + 30_000,
            event_code="IMPACT_SPIKE",
            severity=3,
            value_f32=420.5,
            detail_u64=ts_gt_71_5_min + 5000,
        )

        raw = encode_hks_session(
            hdr,
            cal,
            [imu, hg, aux, evt],
            end_utc_ms=1763859705000,
            end_reason="BUTTON_STOP",
            footer_timestamp_us=ts_gt_71_5_min + 50_000,
        )
        decoded = decode_hks_session(raw, require_footer=True, verify_so3=True, validate_contract_schemas=True)

        self.assertEqual(decoded.header.session_id, 2025)
        self.assertEqual(decoded.imu_batches[0].timestamp_us, ts_gt_71_5_min)
        self.assertEqual(decoded.high_g_batches[0].samples[0], HighGSample(-2048, 0, 2047))
        _, hg_mps2 = decoded.high_g_batches[0].to_numpy(DEFAULT_HIGH_G_SCALE)
        # Verify NO extra >>4 shift on ADXL372 decoding
        self.assertAlmostEqual(float(hg_mps2[0, 0]), -2048.0 * 0.980665, places=4)
        self.assertAlmostEqual(float(hg_mps2[0, 2]), 2047.0 * 0.980665, places=4)
        self.assertEqual(decoded.aux_env_packets[0].pmic_status_u8, 0x40)
        self.assertEqual(decoded.footer.total_bytes, len(raw))  # type: ignore[union-attr]

    def test_rejects_corruptions_and_tampering(self) -> None:
        cal = CalibrationBlock()
        cal = CalibrationBlock(cal_crc32=cal.compute_crc32())
        hdr = SessionHeader(session_id=99, cal_crc32=cal.cal_crc32)
        imu = ImuBatchPacket(timestamp_us=0, odr_hz=200, temp_c_x100=2100, samples=(ImuSample(0, 0, 2048, 0, 0, 0),))
        raw = encode_hks_session(hdr, cal, [imu], end_utc_ms=1000)

        # 1. Bad sync
        bad_sync = bytearray(raw)
        bad_sync[0] ^= 0xFF
        with self.assertRaises(BadSyncError):
            decode_hks_session(bytes(bad_sync))

        # 2. Truncated frame
        with self.assertRaises(TruncatedFrameError):
            decode_hks_session(raw[:-10])

        # 3. Payload bit-flip (frame CRC failure)
        bad_crc = bytearray(raw)
        bad_crc[30] ^= 0x01
        with self.assertRaises(BadCRCError):
            decode_hks_session(bytes(bad_crc))

        # 4. Trailing bytes after footer
        with self.assertRaises(FooterError):
            decode_hks_session(raw + b"\x00\x01\x02\x03")

        # 5. ADXL372 out-of-range sample (forgotten >> 4)
        bad_hg = HighGBatchPacket(
            timestamp_us=1000,
            odr_hz=3200,
            trigger_mode="IMPACT_BURST",
            samples=(HighGSample(32767, 0, 0),),
        )
        with self.assertRaises(InvalidFieldError):
            encode_hks_session(hdr, cal, [imu, bad_hg], end_utc_ms=1000)

        # 6. Tampered frame pre-footer SHA-256 mismatch (valid frame CRCs, modified pre-footer byte + recomputed frame CRC)
        # Re-encode footer with wrong pre_footer_sha256 by flipping a char in footer payload
        from sim.flash_codec import pack_frame, pack_session_footer_payload, SessionFooter

        pre_len = len(raw) - 96
        bad_ftr = SessionFooter(
            timestamp_us=0,
            end_utc_ms=1000,
            total_imu_samples=1,
            dropped_imu_samples=0,
            total_high_g_samples=0,
            dropped_high_g_samples=0,
            total_aux_samples=0,
            end_reason="BUTTON_STOP",
            total_bytes=len(raw),
            pre_footer_sha256="f" * 64,
        )
        tampered = raw[:pre_len] + pack_frame(7, pack_session_footer_payload(bad_ftr), seq=5, timestamp_us=0)
        with self.assertRaises(Sha256MismatchError):
            decode_hks_session(tampered)

        # 7. Non-SO(3) calibration matrix rejected
        bad_cal = CalibrationBlock(
            sensor_to_body_rotation_matrix=((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            cal_crc32=0,
        )
        with self.assertRaises(InvalidFieldError):
            encode_hks_session(hdr, bad_cal, [imu], end_utc_ms=1000)


class TestDownloaderProtocol(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract(DEFAULT_CONTRACT_PATH)
        cal = CalibrationBlock()
        cal = CalibrationBlock(cal_crc32=cal.compute_crc32())
        hdr = SessionHeader(session_id=5001, cal_crc32=cal.cal_crc32)
        batches = [
            ImuBatchPacket(
                timestamp_us=k * 100_000,
                odr_hz=200,
                temp_c_x100=2100,
                samples=tuple(ImuSample(0, 0, 2048, 0, 0, 0) for _ in range(20)),
            )
            for k in range(30)  # > 4096 B total to exercise multi-chunk READ_CHUNK
        ]
        cls.raw_hks = encode_hks_session(hdr, cal, batches, end_utc_ms=1763852503000)

    def test_full_download_flow_delete_guard_and_retry_recovery(self) -> None:
        transport = SimulatedDeviceTransport([self.raw_hks], pmic_status=0xABCD)
        client = DownloadClient(transport, max_retries=3, record_trace=True)

        # GET_DEVICE_STATUS 24 B + 8-bit pmic_status mask
        st = client.get_device_status()
        self.assertEqual(st.pmic_status, 0xCD)
        self.assertGreater(st.flash_free_kb, 100_000)

        # SET_UTC_TIME echo
        echoed = client.set_utc_time(1763859999000)
        self.assertEqual(echoed, 1763859999000)

        # DELETE_SESSION(force=0) BEFORE ACK_VERIFY must fail ERR_NOT_VERIFIED
        with self.assertRaises(DeviceCommandError) as ctx:
            client.delete_session(5001, force=False)
        self.assertEqual(ctx.exception.status_code, StatusCode.ERR_NOT_VERIFIED)

        # Inject faults across GET_HEADER & READ_CHUNK to verify retry recovery
        transport.inject_request_bad_crc(1)
        transport.corrupt_next_response_crc(1)
        transport.truncate_next_response(1)
        transport.corrupt_next_response_seq(1)

        dl = client.download_session(5001, chunk_size=2048, verify=True, delete_after=True, strict_decode=True)
        self.assertEqual(len(dl.header_416b), 416)
        self.assertEqual(dl.raw_bytes, self.raw_hks)
        self.assertEqual(dl.verified_status, 1)
        self.assertEqual(dl.hash_mode, 2)
        self.assertGreater(dl.deleted_freed_bytes or 0, 0)
        self.assertEqual(client.list_sessions(), [])

        # Validate recorded JSON download_command / download_response traces against contract schemas
        cmd_val = Draft202012Validator(self.contract["schemas"]["download_command"])
        rsp_val = Draft202012Validator(self.contract["schemas"]["download_response"])
        for pair in client.trace:
            cmd_val.validate(pair["command"])
            rsp_val.validate(pair["response"])
            validate_packet_semantics("download_response", pair["response"], self.contract)


class TestCInterop(unittest.TestCase):
    def test_c_firmware_cli_hks_decodes_and_fuses(self) -> None:
        fw_cli = REPO_ROOT / "firmware" / "build" / "hks_fw_sim_cli"
        if not fw_cli.exists():
            self.skipTest("firmware/build/hks_fw_sim_cli not built")

        with tempfile.TemporaryDirectory() as tmp:
            hks_path = Path(tmp) / "c_session.hks"
            info_path = Path(tmp) / "c_info.json"
            proc = subprocess.run(
                [str(fw_cli), "--out", str(hks_path), "--info-json", str(info_path), "--usb-selftest"],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            raw = hks_path.read_bytes()
            info = json.loads(info_path.read_text(encoding="utf-8"))

            decoded = decode_hks_session(raw, require_footer=True, verify_so3=True, validate_contract_schemas=True)
            self.assertEqual(decoded.header.session_id, info["session_id"])
            self.assertEqual(decoded.file_crc32, info["file_crc32"])
            self.assertEqual(decoded.file_sha256.hex(), info["file_sha256"])
            self.assertGreater(decoded.imu_batches[-1].timestamp_us, 0xFFFFFFFF)
            self.assertTrue(any(len(b.samples) > 0 for b in decoded.high_g_batches))

            report = run_fusion(decoded)
            contract = load_contract(DEFAULT_CONTRACT_PATH)
            Draft202012Validator(contract["schemas"]["analytics_report"]).validate(report)
            validate_analytics_report_semantics(report, presets=contract["rink_geometry_presets"])


class TestPhysicsSimAndFusion(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract(DEFAULT_CONTRACT_PATH)
        cls.report_validator = Draft202012Validator(cls.contract["schemas"]["analytics_report"])

    def _evaluate_e2e_preset(self, preset: str, seed: int) -> dict[str, float]:
        sim_res = simulate_session(SimConfig(rink_preset=preset, duration_s=140.0, num_shifts=2, seed=seed))
        gt = sim_res.ground_truth

        # Round-trip through SimulatedDeviceTransport -> DownloadClient
        transport = SimulatedDeviceTransport([sim_res.hks_bytes])
        client = DownloadClient(transport)
        sessions = client.list_sessions()
        self.assertEqual(len(sessions), 1)
        dl = client.download_session(sessions[0].session_id, chunk_size=4096, verify=True, delete_after=False)

        t0 = time.perf_counter()
        report = run_fusion(dl.raw_bytes)
        runtime_s = time.perf_counter() - t0

        # Contract & strict JSON validation (allow_nan=False)
        serialized = json.dumps(report, allow_nan=False)
        self.assertGreater(len(serialized), 100)
        self.report_validator.validate(report)
        validate_analytics_report_semantics(
            report,
            presets=self.contract["rink_geometry_presets"],
            q_tol=self.contract["coordinate_frames"]["quaternion_convention"]["unit_norm_tolerance"],
        )

        # Verify 100% of (x, y) inside rink polygon
        rink_spec = self.contract["rink_geometry_presets"][preset]
        for pt in report["trajectory"]:
            self.assertTrue(is_inside_rink(pt["x"], pt["y"], rink_spec))

        # Accuracy vs ground truth
        ss = report["session_summary"]
        self.assertFalse(ss["gps_used"])
        self.assertFalse(ss["external_anchors_used"])
        self.assertEqual(ss["shift_count"], gt["shift_count"])

        max_shift_err_s = 0.0
        for est_sh, gt_sh in zip(report["shifts"], gt["shifts"]):
            err_start = abs(est_sh["start_s"] - gt_sh["start_s"])
            err_end = abs(est_sh["end_s"] - gt_sh["end_s"])
            err_dur = abs(est_sh["duration_s"] - gt_sh["duration_s"])
            max_shift_err_s = max(max_shift_err_s, err_start, err_end, err_dur)

        dist_err_pct = 100.0 * abs(ss["total_distance_m"] - gt["total_distance_m"]) / max(gt["total_distance_m"], 1.0)

        # Trajectory alignment (both at 50 Hz)
        n_cmp = min(len(report["trajectory"]), len(gt["trajectory"]))
        est_xy = np.array([[pt["x"], pt["y"]] for pt in report["trajectory"][:n_cmp]], dtype=np.float64)
        gt_xy = np.array([[pt["x"], pt["y"]] for pt in gt["trajectory"][:n_cmp]], dtype=np.float64)
        est_spd = np.array([pt["speed"] for pt in report["trajectory"][:n_cmp]], dtype=np.float64)
        gt_spd = np.array([pt["speed"] for pt in gt["trajectory"][:n_cmp]], dtype=np.float64)
        gt_states = np.array([pt["state"] for pt in gt["trajectory"][:n_cmp]])

        pos_err = np.linalg.norm(est_xy - gt_xy, axis=1)
        on_ice_mask = np.isin(gt_states, ["STRIDING", "GLIDING", "STOPPING", "IMPACT"])
        cep50_m = float(np.median(pos_err[on_ice_mask]))
        speed_rmse = float(np.sqrt(np.mean((est_spd - gt_spd) ** 2)))

        # Verify ADXL372 unclipped peak accel (> 16 g) recovered accurately
        peak_acc_est = float(ss["peak_accel_mps2"])
        peak_acc_gt = float(gt["peak_accel_mps2"])
        self.assertGreater(peak_acc_est, 16.0 * 9.80665)
        acc_err_pct = 100.0 * abs(peak_acc_est - peak_acc_gt) / peak_acc_gt

        self.assertLessEqual(max_shift_err_s, 2.5, f"{preset}: shift error {max_shift_err_s:.2f} s > 2.5 s")
        self.assertLessEqual(dist_err_pct, 12.0, f"{preset}: dist error {dist_err_pct:.2f}% > 12%")
        self.assertLessEqual(speed_rmse, 0.8, f"{preset}: speed RMSE {speed_rmse:.3f} m/s > 0.8 m/s")
        self.assertLessEqual(cep50_m, 6.0, f"{preset}: CEP50 {cep50_m:.2f} m > 6.0 m")
        self.assertLessEqual(acc_err_pct, 8.0, f"{preset}: peak accel error {acc_err_pct:.2f}% > 8%")
        self.assertLessEqual(runtime_s, 3.0, f"{preset}: fusion runtime {runtime_s:.2f} s")

        return {
            "runtime_s": runtime_s,
            "shift_err_s": max_shift_err_s,
            "dist_err_pct": dist_err_pct,
            "speed_rmse": speed_rmse,
            "cep50_m": cep50_m,
            "peak_acc_est_g": peak_acc_est / 9.80665,
            "peak_acc_gt_g": peak_acc_gt / 9.80665,
        }

    def test_e2e_nhl_and_iihf_accuracy_and_contract(self) -> None:
        m_nhl = self._evaluate_e2e_preset("NHL_200X85FT", seed=42)
        m_iihf = self._evaluate_e2e_preset("IIHF_60X30M", seed=77)
        self.assertGreater(m_nhl["peak_acc_est_g"], 20.0)
        self.assertGreater(m_iihf["peak_acc_est_g"], 20.0)

    def test_ablation_divergence_and_zero_ground_truth_leakage(self) -> None:
        # 1. Static AST isolation check: fusion_pipeline.py must not import physics_sim or reference ground_truth
        fp_src = (REPO_ROOT / "sim" / "fusion_pipeline.py").read_text(encoding="utf-8")
        tree = ast.parse(fp_src)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn("physics_sim", alias.name)
            elif isinstance(node, ast.ImportFrom):
                self.assertNotIn("physics_sim", node.module or "")
        self.assertNotIn("ground_truth", fp_src)

        # 2. Ablation proof: disabling drift-bounding constraints causes > 20 m divergence
        sim_res = simulate_session(SimConfig(rink_preset="NHL_200X85FT", duration_s=140.0, num_shifts=2, seed=42))
        gt = sim_res.ground_truth
        full_rep = run_fusion(sim_res.hks_bytes)
        abl_rep = run_fusion(
            sim_res.hks_bytes,
            ablation=AblationConfig(
                use_zupt_zaru=False,
                use_nhc_vz=False,
                use_nhc_vy=False,
                use_stride_speed=False,
                use_rink_containment=False,
                use_bench_gate_reset=False,
                use_rts_smoother=False,
            ),
        )
        n_cmp = min(len(full_rep["trajectory"]), len(abl_rep["trajectory"]), len(gt["trajectory"]))
        gt_xy = np.array([[pt["x"], pt["y"]] for pt in gt["trajectory"][:n_cmp]], dtype=np.float64)
        full_xy = np.array([[pt["x"], pt["y"]] for pt in full_rep["trajectory"][:n_cmp]], dtype=np.float64)
        abl_xy = np.array([[pt["x"], pt["y"]] for pt in abl_rep["trajectory"][:n_cmp]], dtype=np.float64)

        full_cep50 = float(np.median(np.linalg.norm(full_xy - gt_xy, axis=1)))
        abl_cep50 = float(np.median(np.linalg.norm(abl_xy - gt_xy, axis=1)))
        abl_max_err = float(np.max(np.linalg.norm(abl_xy - gt_xy, axis=1)))

        self.assertLessEqual(full_cep50, 6.0)
        self.assertGreater(abl_max_err, 20.0, f"Ablated max drift {abl_max_err:.2f} m did not exceed 20 m")
        self.assertGreater(abl_cep50, full_cep50 * 3.0)


class TestCli(unittest.TestCase):
    def test_cli_simulate_download_analyze_and_e2e(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            hks_path = tmp / "sim.hks"
            truth_path = tmp / "truth.json"
            dl_hks = tmp / "dl.hks"
            trace_path = tmp / "trace.json"
            rep_path = tmp / "report.json"
            e2e_dir = tmp / "e2e"

            self.assertEqual(
                cli_main(["simulate", "--preset", "CUSTOM", "--duration-s", "60", "--shifts", "1", "--out-hks", str(hks_path), "--out-truth", str(truth_path)]),
                0,
            )
            self.assertTrue(hks_path.exists() and truth_path.exists())

            self.assertEqual(
                cli_main(["download", "--transport", "sim", "--hks-source", str(hks_path), "--out-hks", str(dl_hks), "--out-trace", str(trace_path)]),
                0,
            )
            self.assertEqual(dl_hks.read_bytes(), hks_path.read_bytes())

            self.assertEqual(
                cli_main(["analyze", "--in-hks", str(dl_hks), "--out-report", str(rep_path)]),
                0,
            )
            self.assertTrue(rep_path.exists())

            self.assertEqual(
                cli_main(["e2e", "--preset", "NHL_200X85FT", "--out-dir", str(e2e_dir), "--delete-after"]),
                0,
            )
            self.assertTrue((e2e_dir / "analytics_report.json").exists())
            self.assertTrue((e2e_dir / "e2e_summary.json").exists())


if __name__ == "__main__":
    unittest.main()
