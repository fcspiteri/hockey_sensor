"""Tier 4 E2E Tests: Real-World Hockey Application Scenarios (S01-S06, 6 scenarios)."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from e2e import helpers  # noqa: E402


class TestTier4Scenarios(unittest.TestCase):
    def test_s01_full_3_period_game_over_2_hours(self) -> None:
        """S01: Full 3-period game (7500 s > 2 h, > 2^32 us timestamp rollover, cold battery & flash)."""
        helpers.require_m2()
        helpers.require_m3()
        helpers.require_m4()
        from docs.validate_contract import validate_analytics_report_semantics

        raw, rep, truth = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=7500.0, shifts=60, seed=4001
        )
        helpers.validate_against_schema("analytics_report", rep)
        validate_analytics_report_semantics(rep, presets=helpers.load_contract()["rink_geometry_presets"])
        self.assertLessEqual(len(raw), helpers.FLASH_USABLE_RING_BYTES)

        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        self.assertGreater(frames[-1]["header"]["timestamp_us"], 0xFFFFFFFF)
        self.assertEqual(frames[-1]["packet"]["dropped_imu_samples"], 0)
        self.assertEqual(frames[-1]["packet"]["dropped_high_g_samples"], 0)

        self.assertGreaterEqual(rep["session_summary"]["total_duration_s"], 7200.0)
        self.assertFalse(rep["session_summary"]["gps_used"])
        self.assertFalse(rep["session_summary"]["external_anchors_used"])
        self.assertLessEqual(rep["session_summary"]["battery_used_pct"], 3.0)

        est_s = [(s["start_s"], s["end_s"]) for s in rep["shifts"]]
        true_s = [(s["start_s"], s["end_s"]) for s in truth["shifts"]]
        f1, bnd = helpers.compute_shift_f1_and_boundary_error(est_s, true_s)
        self.assertGreaterEqual(f1, 0.95)
        self.assertLessEqual(bnd, 3.0)

        dist_err = helpers.compute_distance_pct_error(
            rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
        )
        self.assertLessEqual(dist_err, 8.0)

        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_spd = [p["speed"] for p in est_al]
        true_spd = [p["speed"] for p in truth_al]
        self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.6)

        est_xy = [(p["x"], p["y"]) for p in est_al]
        true_xy = [(p["x"], p["y"]) for p in truth_al]
        self.assertLessEqual(helpers.compute_cep50(est_xy, true_xy), 6.0)

    def test_s02_intense_board_battle_and_body_checks_20_to_80g(self) -> None:
        """S02: Intense board battles & 20-80 g body checks without 16 g clipping."""
        helpers.require_m4()
        raw, rep, truth = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=4002
        )
        helpers.validate_against_schema("analytics_report", rep)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        mtypes = {f["header"]["msg_type"] for f in frames}
        self.assertIn(helpers.MSG_HIGH_G_BATCH, mtypes)
        self.assertIn(helpers.MSG_EVENT_MARKER, mtypes)

        self.assertGreater(rep["session_summary"]["peak_accel_mps2"], 16.0 * helpers.GRAVITY_MPS2)
        for pt in rep["trajectory"]:
            self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], "NHL_200X85FT", tol=1e-6))

        dist_err = helpers.compute_distance_pct_error(
            rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
        )
        self.assertLessEqual(dist_err, 8.0)

    def test_s03_nhl_vs_iihf_rink_preset_comparison(self) -> None:
        """S03: NHL_200X85FT (25.908 m wide) vs IIHF_60X30M (30.0 m wide) preset geometry & bench resets."""
        helpers.require_m4()
        presets = helpers.load_contract()["rink_geometry_presets"]
        raw_nhl, rep_nhl, truth_nhl = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=9203
        )
        raw_iihf, rep_iihf, truth_iihf = helpers.sim_and_analyze_cached(
            preset="IIHF_60X30M", duration_s=140.0, shifts=2, seed=9203
        )
        f_nhl = helpers.unpack_frames(raw_nhl, verify_crc=True, verify_footer_sha256=True)
        f_iihf = helpers.unpack_frames(raw_iihf, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual(f_nhl[0]["packet"]["bench_gate_xy_m"], presets["NHL_200X85FT"]["default_bench_gate_xy_m"])
        self.assertEqual(f_iihf[0]["packet"]["bench_gate_xy_m"], presets["IIHF_60X30M"]["default_bench_gate_xy_m"])

        for pt in rep_nhl["trajectory"]:
            self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], "NHL_200X85FT", tol=1e-6))
            self.assertLessEqual(abs(pt["y"]), 12.954 + 1e-6)
        for pt in rep_iihf["trajectory"]:
            self.assertTrue(helpers.is_inside_rink(pt["x"], pt["y"], "IIHF_60X30M", tol=1e-6))

        for rep, truth in ((rep_nhl, truth_nhl), (rep_iihf, truth_iihf)):
            est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
            est_spd = [p["speed"] for p in est_al]
            true_spd = [p["speed"] for p in truth_al]
            self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.6)
            self.assertLessEqual(
                helpers.compute_distance_pct_error(
                    rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
                ),
                8.0,
            )

    def test_s04_power_play_sprint_glide_hockey_stop_and_faceoff_zupt(self) -> None:
        """S04: Breakaway sprint + passive glide + sharp hockey stop + <45s faceoff ZUPT."""
        helpers.require_m4()
        _, rep, truth = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=4004
        )
        helpers.validate_against_schema("analytics_report", rep)
        states = {p["state"] for p in rep["trajectory"]}
        self.assertIn("STRIDING", states)
        self.assertIn("BENCH", states)
        self.assertIn("GLIDING", states)
        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_spd = [p["speed"] for p in est_al]
        true_spd = [p["speed"] for p in truth_al]
        self.assertLessEqual(helpers.compute_speed_rmse(est_spd, true_spd), 0.6)
        self.assertLessEqual(
            helpers.compute_distance_pct_error(
                rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
            ),
            8.0,
        )

    def test_s05_post_session_wired_usb_cdc_multi_session_dump_and_ack_verify(self) -> None:
        """S05: Multi-session wired USB CDC-ACM download with corrupt chunk retry & ACK_VERIFY delete gate."""
        helpers.require_m3()
        helpers.require_m4()
        from sim.downloader import DeviceCommandError, DownloadClient, SimulatedDeviceTransport, StatusCode
        from sim.physics_sim import SimConfig, simulate_session

        raw_c, info_c, _ = helpers.run_fw_sim_cached(usb_selftest=True)
        sid_c = int(info_c["session_id"])
        raw_p2 = simulate_session(SimConfig(session_id=5002, duration_s=45.0, num_shifts=1, seed=5002)).hks_bytes
        raw_p3 = simulate_session(SimConfig(session_id=5003, duration_s=60.0, num_shifts=1, seed=5003)).hks_bytes
        expected_blobs = {sid_c: raw_c, 5002: raw_p2, 5003: raw_p3}

        transport = SimulatedDeviceTransport([raw_c, raw_p2, raw_p3])
        client = DownloadClient(transport, max_retries=3, record_trace=True)

        listed = client.list_sessions()
        self.assertEqual({s.session_id for s in listed}, set(expected_blobs.keys()))

        # Unverified DELETE_SESSION must fail ERR_NOT_VERIFIED
        with self.assertRaises(DeviceCommandError) as ctx:
            client.delete_session(5002, force=False)
        self.assertEqual(ctx.exception.status_code, StatusCode.ERR_NOT_VERIFIED)

        # Inject corruptions on session 2 download to verify retry recovery
        transport.inject_request_bad_crc(1)
        transport.corrupt_next_response_crc(1)
        transport.truncate_next_response(1)

        with tempfile.TemporaryDirectory() as tmp:
            reports = []
            for sid, expected_raw in expected_blobs.items():
                dl = client.download_session(sid, chunk_size=2048, verify=True, delete_after=True, strict_decode=True)
                self.assertEqual(dl.raw_bytes, expected_raw)
                self.assertEqual(dl.verified_status, 1)
                self.assertEqual(dl.hash_mode, 2)

                dl_hks = Path(tmp) / f"s_{sid}_dl.hks"
                rep_p = Path(tmp) / f"s_{sid}_rep.json"
                dl_hks.write_bytes(dl.raw_bytes)
                p_ana = helpers.run_sim_cli(["analyze", "--in-hks", str(dl_hks), "--out-report", str(rep_p)])
                self.assertEqual(p_ana.returncode, 0, msg=p_ana.stderr)
                rep = helpers.load_json_strict(rep_p)
                helpers.validate_against_schema("analytics_report", rep)
                reports.append(rep)

            self.assertEqual(len(reports), 3)
            self.assertEqual(client.list_sessions(), [])
            tx_json = Path(tmp) / "multi_tx.json"
            tx_json.write_text(json.dumps(client.trace, indent=2), encoding="utf-8")
            tx = helpers.load_json_strict(tx_json)
            for pair in tx:
                helpers.validate_against_schema("download_command", pair["command"])
                helpers.validate_against_schema("download_response", pair["response"])

    def test_s06_magnetic_interference_rink_zamboni_goal_dasher(self) -> None:
        """S06: Magnetic interference near goal/dasher/Zamboni steel bounded by anomaly gate."""
        helpers.require_m4()
        raw, rep, truth = helpers.sim_and_analyze_cached(
            preset="NHL_200X85FT", duration_s=140.0, shifts=2, seed=4006
        )
        helpers.validate_against_schema("analytics_report", rep)
        frames = helpers.unpack_frames(raw, verify_crc=True, verify_footer_sha256=True)
        mag_anom_frames = sum(
            1 for f in frames if f["header"]["msg_type"] == helpers.MSG_AUX_ENV and (f["header"]["flags"] & 0x04) != 0
        )
        self.assertGreater(mag_anom_frames, 0)

        est_al, truth_al = helpers.align_trajectory_by_time(rep["trajectory"], truth["trajectory"])
        est_xy = [(p["x"], p["y"]) for p in est_al]
        true_xy = [(p["x"], p["y"]) for p in truth_al]
        self.assertLessEqual(helpers.compute_cep50(est_xy, true_xy), 6.0)
        self.assertLessEqual(
            helpers.compute_distance_pct_error(
                rep["session_summary"]["total_distance_m"], truth["session_summary"]["total_distance_m"]
            ),
            8.0,
        )


if __name__ == "__main__":
    unittest.main()
