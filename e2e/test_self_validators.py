"""Self-tests for e2e/helpers.py codecs, math/geometry oracles, protocol state machine, and rubric."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from e2e import helpers


class TestBinaryHksCodec(unittest.TestCase):
    def test_crc32_ieee_standard_test_vector(self) -> None:
        self.assertEqual(helpers.crc32_ieee(b"123456789"), 0xCBF43926)

    def test_all_7_frame_types_round_trip_and_schema_validation(self) -> None:
        contract = helpers.load_contract()
        examples = contract["examples"]

        stream = helpers.build_synthetic_session_bytes(
            session_id=4242,
            rink_preset="NHL_200X85FT",
            start_timestamp_us=0,
            end_timestamp_us=7_200_050_000,
            seq_start=65534,
            include_high_g=True,
        )
        frames = helpers.unpack_frames(stream, verify_crc=True, verify_footer_sha256=True)
        self.assertEqual(len(frames), 7)

        expected_sizes = [104, 312, 272 - 18 * 12, 224 - 30 * 6, 52, 40, 96]
        # Verify frame sizes & msg_types (0x01, 0x06, 0x02, 0x03, 0x04, 0x05, 0x07)
        self.assertEqual([f["header"]["msg_type"] for f in frames], [1, 6, 2, 3, 4, 5, 7])
        self.assertEqual(frames[0]["frame_bytes"], 104)
        self.assertEqual(frames[1]["frame_bytes"], 312)
        self.assertEqual(frames[4]["frame_bytes"], 52)
        self.assertEqual(frames[5]["frame_bytes"], 40)
        self.assertEqual(frames[6]["frame_bytes"], 96)
        self.assertEqual(len(expected_sizes), 7)

        # Seq rollover 65534 -> 65535 -> 0 -> 1 -> 2 -> 3 -> 4
        self.assertEqual([f["header"]["seq"] for f in frames], [65534, 65535, 0, 1, 2, 3, 4])

        # Validate every unpacked packet against its contract JSON Schema
        schema_map = {
            1: "flash_session_header",
            2: "imu_batch_packet",
            3: "high_g_batch_packet",
            4: "aux_env_packet",
            5: "event_marker_packet",
            6: "calibration_block",
            7: "flash_session_footer",
        }
        for f in frames:
            mtype = f["header"]["msg_type"]
            helpers.validate_against_schema(schema_map[mtype], f["packet"])

        # Nominal N=20 IMU_BATCH (272 B) & N=32 HIGH_G_BATCH (224 B) exact sizes
        imu20 = json.loads(json.dumps(examples["imu_batch_packet"]))
        imu20["samples"] = [imu20["samples"][0]] * 20
        imu20["sample_count"] = 20
        self.assertEqual(len(helpers.pack_frame(helpers.MSG_IMU_BATCH, imu20)), 272)

        hg32 = json.loads(json.dumps(examples["high_g_batch_packet"]))
        hg32["samples"] = [hg32["samples"][0]] * 32
        hg32["sample_count"] = 32
        self.assertEqual(len(helpers.pack_frame(helpers.MSG_HIGH_G_BATCH, hg32)), 224)

    def test_one_bit_flip_rejected_by_crc32_and_bad_sha256_rejected(self) -> None:
        stream = bytearray(helpers.build_synthetic_session_bytes())
        # Flip 1 bit inside first frame payload
        corrupted_payload = bytearray(stream)
        corrupted_payload[30] ^= 0x01
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bytes(corrupted_payload), verify_crc=True)

        # Flip 1 bit inside CRC32 trailer of first frame
        corrupted_crc = bytearray(stream)
        corrupted_crc[103] ^= 0x80
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bytes(corrupted_crc), verify_crc=True)

        # Flip 1 bit in sync marker
        bad_sync = bytearray(stream)
        bad_sync[0] ^= 0x01
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bytes(bad_sync), verify_crc=True)

        # Bad pre_footer_sha256 (even with valid frame CRC32)
        frames = helpers.unpack_frames(bytes(stream), verify_crc=True, verify_footer_sha256=True)
        footer_pkt = dict(frames[-1]["packet"])
        footer_pkt["pre_footer_sha256"] = "0" * 64
        bad_footer_frame = helpers.pack_frame(
            helpers.MSG_SESSION_FOOTER,
            footer_pkt,
            flags=frames[-1]["header"]["flags"],
            seq=frames[-1]["header"]["seq"],
            timestamp_us=frames[-1]["header"]["timestamp_us"],
        )
        bad_sha_stream = bytes(stream[:-96]) + bad_footer_frame
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bad_sha_stream, verify_crc=True, verify_footer_sha256=True)

        # Bad total_bytes in footer (even with valid CRC32 & valid pre_footer_sha256)
        bad_total_pkt = dict(frames[-1]["packet"])
        bad_total_pkt["total_bytes"] = len(stream) - 10
        bad_total_frame = helpers.pack_frame(
            helpers.MSG_SESSION_FOOTER,
            bad_total_pkt,
            flags=frames[-1]["header"]["flags"],
            seq=frames[-1]["header"]["seq"],
            timestamp_us=frames[-1]["header"]["timestamp_us"],
        )
        bad_total_stream = bytes(stream[:-96]) + bad_total_frame
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bad_total_stream, verify_crc=True, verify_footer_sha256=True)

        # Truncated mid-frame
        with self.assertRaises(ValueError):
            helpers.unpack_frames(bytes(stream[:-10]), verify_crc=True)

        # Empty stream
        with self.assertRaises(ValueError):
            helpers.unpack_frames(b"", verify_crc=True)

    def test_64bit_timestamps_exceed_32bit_rollover(self) -> None:
        ts_values = [4_294_967_295, 4_294_967_296, 7_200_000_000, 14_400_000_000]
        stream = b"".join(
            helpers.pack_frame(
                helpers.MSG_EVENT_MARKER,
                {"event_code": "BUTTON_MARK", "severity": 0, "value_f32": float(i), "detail_u64": ts},
                seq=i,
                timestamp_us=ts,
            )
            for i, ts in enumerate(ts_values)
        )
        frames = helpers.unpack_frames(stream, verify_crc=True, verify_footer_sha256=False)
        decoded_ts = [f["header"]["timestamp_us"] for f in frames]
        self.assertEqual(decoded_ts, ts_values)
        self.assertTrue(all(decoded_ts[i] < decoded_ts[i + 1] for i in range(len(decoded_ts) - 1)))

    def test_adxl372_right_shift_bounds_no_double_shift(self) -> None:
        self.assertEqual(helpers.adxl372_fifo_word_to_i16(0x8000), -2048)
        self.assertEqual(helpers.adxl372_fifo_word_to_i16(0x7FF0), 2047)
        self.assertEqual(helpers.adxl372_fifo_word_to_i16(0x0C80), 200)
        self.assertAlmostEqual(helpers.adxl372_i16_to_mps2(-2048), -2008.40192, places=5)
        self.assertAlmostEqual(helpers.adxl372_i16_to_mps2(2047), 2007.421255, places=5)
        self.assertAlmostEqual(helpers.adxl372_i16_to_mps2(-1), -0.980665, places=6)
        self.assertAlmostEqual(helpers.adxl372_i16_to_mps2(0), 0.0, places=6)
        with self.assertRaises(ValueError):
            helpers.adxl372_i16_to_mps2(-2049)
        with self.assertRaises(ValueError):
            helpers.adxl372_i16_to_mps2(2048)


class TestDownloadProtocolStateMachine(unittest.TestCase):
    def test_full_usb_cdc_workflow_and_error_guards(self) -> None:
        sess_blob = helpers.build_synthetic_session_bytes(session_id=1001)
        dev = helpers.ReferenceDownloadDevice(sessions={1001: sess_blob})

        # 1. SET_UTC_TIME
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_SET_UTC_TIME, struct.pack("<Q", 1763852600000), seq=1)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        self.assertEqual(struct.unpack("<Q", rsp["payload"])[0], 1763852600000)

        # 2. GET_DEVICE_STATUS (24 B)
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(helpers.pack_download_command(helpers.CMD_GET_DEVICE_STATUS, seq=2))
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        self.assertEqual(len(rsp["payload"]), 24)

        # 3. LIST_SESSIONS
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(helpers.pack_download_command(helpers.CMD_LIST_SESSIONS, seq=3))
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        count, _ = struct.unpack("<HH", rsp["payload"][:4])
        self.assertEqual(count, 1)

        # 4. GET_HEADER (416 B)
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_GET_HEADER, struct.pack("<Q", 1001), seq=4)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        self.assertEqual(len(rsp["payload"]), 416)
        self.assertEqual(rsp["payload"], sess_blob[:416])

        # 5. READ_CHUNK loop
        downloaded = bytearray()
        offset = 0
        while offset < len(sess_blob):
            rsp = helpers.unpack_download_response(
                dev.handle_command_frame(
                    helpers.pack_download_command(
                        helpers.CMD_READ_CHUNK, struct.pack("<QIH", 1001, offset, 256), seq=5
                    )
                )
            )
            self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
            r_off, actual_len = struct.unpack("<IH", rsp["payload"][:6])
            self.assertEqual(r_off, offset)
            chunk = rsp["payload"][6:]
            self.assertEqual(len(chunk), actual_len)
            downloaded.extend(chunk)
            offset += actual_len
        self.assertEqual(bytes(downloaded), sess_blob)

        # 6. DELETE_SESSION before ACK_VERIFY -> ERR_NOT_VERIFIED (0x05)
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_DELETE_SESSION, struct.pack("<QB", 1001, 0), seq=6)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_NOT_VERIFIED)
        self.assertIn(1001, dev.sessions)

        # 7. ACK_VERIFY with wrong SHA256 -> ERR_VERIFY_MISMATCH (0x08)
        bad_ack = struct.pack("<QI", 1001, helpers.crc32_ieee(sess_blob)) + (b"\x00" * 32)
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(helpers.pack_download_command(helpers.CMD_ACK_VERIFY, bad_ack, seq=7))
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_VERIFY_MISMATCH)

        # 8. ACK_VERIFY valid -> OK, then DELETE_SESSION -> OK
        good_ack = struct.pack("<QI", 1001, helpers.crc32_ieee(sess_blob)) + hashlib.sha256(sess_blob).digest()
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(helpers.pack_download_command(helpers.CMD_ACK_VERIFY, good_ack, seq=8))
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        self.assertEqual(struct.unpack("<BB", rsp["payload"]), (1, 2))

        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_DELETE_SESSION, struct.pack("<QB", 1001, 0), seq=9)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_OK)
        self.assertEqual(struct.unpack("<I", rsp["payload"])[0], len(sess_blob))
        self.assertEqual(len(dev.sessions), 0)

    def test_protocol_bad_crc_unknown_cmd_busy_and_offset_errors(self) -> None:
        sess_blob = helpers.build_synthetic_session_bytes(session_id=2002)
        dev = helpers.ReferenceDownloadDevice(sessions={2002: sess_blob})

        # Bad CRC
        bad_cmd = bytearray(helpers.pack_download_command(helpers.CMD_LIST_SESSIONS, seq=1))
        bad_cmd[-1] ^= 0x01
        rsp = helpers.unpack_download_response(dev.handle_command_frame(bytes(bad_cmd)))
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_BAD_CRC)

        # Unknown command
        rsp = helpers.unpack_download_response(dev.handle_command_frame(helpers.pack_download_command(0xFE, seq=2)))
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_UNKNOWN_CMD)

        # Offset out of range
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(
                    helpers.CMD_READ_CHUNK, struct.pack("<QIH", 2002, len(sess_blob) + 1, 64), seq=3
                )
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_OFFSET_OUT_OF_RANGE)

        # Invalid length_bytes (4097)
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_READ_CHUNK, struct.pack("<QIH", 2002, 0, 4097), seq=4)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_INVALID_PARAM)

        # Busy recording
        dev.recording = True
        rsp = helpers.unpack_download_response(
            dev.handle_command_frame(
                helpers.pack_download_command(helpers.CMD_DELETE_SESSION, struct.pack("<QB", 2002, 1), seq=5)
            )
        )
        self.assertEqual(rsp["status_code"], helpers.STATUS_ERR_BUSY_RECORDING)


class TestMathRinkMetricsAndRubric(unittest.TestCase):
    def test_so3_and_hamilton_quaternion_rotation(self) -> None:
        canonical_r_bs = [[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]]
        ok, ortho_err, det = helpers.is_so3(canonical_r_bs)
        self.assertTrue(ok)
        self.assertLessEqual(ortho_err, 1e-5)
        self.assertAlmostEqual(det, 1.0, places=6)

        # Reflection det = -1 rejected
        reflection = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        ok_ref, _, det_ref = helpers.is_so3(reflection)
        self.assertFalse(ok_ref)
        self.assertAlmostEqual(det_ref, -1.0, places=6)

        # Stationary sensor reading [1g, 0, 0] in S transforms via R_BS to [0, 0, +1g] in B
        g_b = helpers.mat_vec_mul(canonical_r_bs, [helpers.GRAVITY_MPS2, 0.0, 0.0])
        self.assertAlmostEqual(g_b[0], 0.0, places=6)
        self.assertAlmostEqual(g_b[1], 0.0, places=6)
        self.assertAlmostEqual(g_b[2], helpers.GRAVITY_MPS2, places=6)

        # 90 deg yaw about +Z_W: q = [cos(pi/4), 0, 0, sin(pi/4)] rotates +X_B -> +Y_W
        q_yaw90 = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
        self.assertTrue(helpers.is_unit_quaternion(q_yaw90))
        v_w = helpers.rotate_vec_by_quat(q_yaw90, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(v_w[0], 0.0, places=6)
        self.assertAlmostEqual(v_w[1], 1.0, places=6)
        self.assertAlmostEqual(v_w[2], 0.0, places=6)

    def test_rink_rounded_rectangle_boundaries_all_presets(self) -> None:
        # Center and straight board limits for NHL_200X85FT (60.96 x 25.908, Rc=8.5344)
        self.assertTrue(helpers.is_inside_rink(0.0, 0.0, "NHL_200X85FT"))
        self.assertTrue(helpers.is_inside_rink(0.0, 12.954, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(0.0, 12.955, "NHL_200X85FT"))
        # Corner rectangle point inside [-L/2,L/2]x[-W/2,W/2] yet outside rounded fillet
        self.assertFalse(helpers.is_inside_rink(29.5, 12.2, "NHL_200X85FT"))
        # Exact corner arc point
        core_x = 60.96 / 2.0 - 8.5344
        core_y = 25.908 / 2.0 - 8.5344
        arc_x = core_x + 8.5344 * math.sqrt(0.5)
        arc_y = core_y + 8.5344 * math.sqrt(0.5)
        self.assertTrue(helpers.is_inside_rink(arc_x, arc_y, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(arc_x, arc_y, "NHL_200X85FT", torso_margin_m=0.25))
        self.assertGreater(helpers.rink_wall_signed_distance(29.5, 12.2, "NHL_200X85FT"), 0.0)

        # IIHF allows y = 14.0 m where NHL does not
        self.assertTrue(helpers.is_inside_rink(0.0, 14.0, "IIHF_60X30M"))
        self.assertFalse(helpers.is_inside_rink(0.0, 14.0, "NHL_200X85FT"))
        self.assertTrue(helpers.is_inside_rink(-5.0, 13.0, "CUSTOM"))

    def test_metric_calculators(self) -> None:
        self.assertAlmostEqual(helpers.compute_speed_rmse([5.0, 6.0], [5.3, 5.7]), 0.3, places=6)
        self.assertAlmostEqual(helpers.compute_cep50([(0.0, 1.0), (0.0, 3.0), (0.0, 5.0)], [(0.0, 0.0)] * 3), 3.0)
        f1, bnd = helpers.compute_shift_f1_and_boundary_error([(5.5, 48.0), (100.0, 145.5)], [(5.0, 48.5), (100.0, 145.0)])
        self.assertAlmostEqual(f1, 1.0)
        self.assertAlmostEqual(bnd, 0.5)
        self.assertAlmostEqual(helpers.compute_distance_pct_error(105.0, 100.0), 5.0)

    def test_rubric_formatter_pass_and_fail_rendering(self) -> None:
        report_pass = helpers.evaluate_rubric(contract_only=True)
        self.assertTrue(report_pass.overall_pass)
        txt_pass = helpers.format_rubric_table(report_pass)
        self.assertIn("MODE: CONTRACT-ONLY", txt_pass)
        self.assertIn("OVERALL RUBRIC VERDICT: PASS", txt_pass)

        # Injected failure check
        report_fail = helpers.RubricReport(
            mode="FULL",
            overall_pass=False,
            passed_checks=1,
            total_checks=2,
            pillars=[
                helpers.RubricPillar(1, "Metric Check", True, [helpers.RubricCheck("c1", True, "ok")]),
                helpers.RubricPillar(2, "Zero GPS", False, [helpers.RubricCheck("c2", False, "injected fail")]),
            ],
        )
        txt_fail = helpers.format_rubric_table(report_fail)
        self.assertIn("MODE: FULL", txt_fail)
        self.assertIn("FAIL", txt_fail)
        self.assertIn("OVERALL RUBRIC VERDICT: FAIL", txt_fail)

    def test_strict_artifact_resolvers_raise_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            empty_dir = Path(tmp) / "empty"
            empty_dir.mkdir()
            with patch.object(helpers, "HARDWARE_DIR", empty_dir):
                with self.assertRaises(AssertionError):
                    helpers.require_m2()
            with patch.object(helpers, "FIRMWARE_DIR", empty_dir):
                with self.assertRaises(AssertionError):
                    helpers.require_m3(build_if_needed=False)
            with patch.object(helpers, "SIM_DIR", empty_dir):
                with self.assertRaises(AssertionError):
                    helpers.require_m4()

    def test_non_finite_guards_rink_cep50_and_rmse(self) -> None:
        self.assertFalse(helpers.is_inside_rink(float("nan"), 0.0, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(0.0, float("nan"), "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(float("inf"), 0.0, "NHL_200X85FT"))
        self.assertFalse(helpers.is_inside_rink(0.0, 0.0, "NHL_200X85FT", torso_margin_m=float("nan")))
        with self.assertRaises(ValueError):
            helpers.rink_wall_signed_distance(float("nan"), 0.0, "NHL_200X85FT")

        with self.assertRaises(ValueError):
            helpers.compute_cep50([(float("nan"), 0.0), (0.0, 1.0), (0.0, 2.0)], [(0.0, 0.0)] * 3)
        with self.assertRaises(ValueError):
            helpers.compute_cep50([(0.0, 0.0), (float("inf"), 1.0)], [(0.0, 0.0)] * 2)
        with self.assertRaises(ValueError):
            helpers.compute_speed_rmse([float("nan"), 1.0], [0.0, 1.0])
        with self.assertRaises(ValueError):
            helpers.compute_distance_pct_error(float("nan"), 100.0)

    def test_align_trajectory_by_time_disparate_odr_and_truncation_rejection(self) -> None:
        # Truth at 50 Hz (0.00 .. 10.00 s), linear motion x=2*t, y=-t, speed=hypot(2,-1)
        spd = math.hypot(2.0, -1.0)
        truth = [{"t": k * 0.02, "x": 2.0 * k * 0.02, "y": -k * 0.02, "speed": spd} for k in range(501)]
        # Est at 25 Hz (0.00 .. 10.00 s) with slight offset
        est = [{"t": k * 0.04, "x": 2.0 * k * 0.04 + 0.1, "y": -k * 0.04, "speed": spd} for k in range(251)]

        a_est, a_truth = helpers.align_trajectory_by_time(est, truth)
        self.assertEqual(len(a_est), len(est))
        self.assertEqual(len(a_truth), len(est))
        self.assertAlmostEqual(
            helpers.compute_cep50([(p["x"], p["y"]) for p in a_est], [(p["x"], p["y"]) for p in a_truth]),
            0.1,
            places=6,
        )
        self.assertAlmostEqual(
            helpers.compute_speed_rmse([p["speed"] for p in a_est], [p["speed"] for p in a_truth]),
            0.0,
            places=6,
        )

        # Reject 1-point degenerate trajectory
        with self.assertRaises(ValueError):
            helpers.align_trajectory_by_time(est[:1], truth)

        # Reject truncated trajectory stopping early (e.g., 4.0 s vs 10.0 s)
        with self.assertRaises(ValueError):
            helpers.align_trajectory_by_time(est[:101], truth)

    def test_rubric_handles_missing_metric_invalid_report_and_sneaky_substring(self) -> None:
        base_contract = copy.deepcopy(helpers.load_contract(use_cache=False))
        with tempfile.TemporaryDirectory() as tmp:
            mut_path = Path(tmp) / "mut_contract.json"

            # 1. Deleted metric in units_and_tolerances -> FAIL P1 cleanly without KeyError
            c_missing = copy.deepcopy(base_contract)
            del c_missing["units_and_tolerances"]["distance"]
            mut_path.write_text(json.dumps(c_missing), encoding="utf-8")
            try:
                helpers.clear_caches()
                with patch.object(helpers, "CONTRACT_PATH", mut_path):
                    rep = helpers.evaluate_rubric(contract_only=True)
                self.assertFalse(rep.overall_pass)
                self.assertEqual([p.pillar_id for p in rep.pillars if not p.passed], [1])
            finally:
                helpers.clear_caches()

            # 2. "+/- 15% garbage" must NOT pass P1.distance_skated_contract
            c_sub = copy.deepcopy(base_contract)
            c_sub["units_and_tolerances"]["distance"]["tolerance"] = "+/- 15% garbage"
            mut_path.write_text(json.dumps(c_sub), encoding="utf-8")
            try:
                helpers.clear_caches()
                with patch.object(helpers, "CONTRACT_PATH", mut_path):
                    rep = helpers.evaluate_rubric(contract_only=True)
                self.assertFalse(rep.overall_pass)
                p1 = next(p for p in rep.pillars if p.pillar_id == 1)
                dist_chk = next(c for c in p1.checks if c.name == "distance_skated_contract")
                self.assertFalse(dist_chk.passed)
            finally:
                helpers.clear_caches()

            # 3. Invalid analytics_report example -> FAIL P1 cleanly without uncaught ValidationError
            c_bad_rep = copy.deepcopy(base_contract)
            c_bad_rep["examples"]["analytics_report"]["session_summary"]["gps_used"] = True
            mut_path.write_text(json.dumps(c_bad_rep), encoding="utf-8")
            try:
                helpers.clear_caches()
                with patch.object(helpers, "CONTRACT_PATH", mut_path):
                    rep = helpers.evaluate_rubric(contract_only=True)
                self.assertFalse(rep.overall_pass)
                self.assertIn(1, [p.pillar_id for p in rep.pillars if not p.passed])
            finally:
                helpers.clear_caches()

    def test_scan_zero_gps_catches_dwm3000(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bom = Path(tmp) / "bom.csv"
            pinmux = Path(tmp) / "pinmux.json"
            bom.write_text("refdes,subsystem_key,category,part_number,manufacturer,bus_or_function,notes\nU1,mcu,MCU,nRF5340,Nordic,Core,ok\n", encoding="utf-8")
            pinmux.write_text(
                json.dumps(
                    {
                        "invariants": {"uses_gps": False, "uses_external_anchors": False},
                        "buses": {"SPIM2": {"devices": [{"part_number": "DWM3000"}]}},
                    }
                ),
                encoding="utf-8",
            )
            hits = helpers.scan_zero_gps_compliance(bom, pinmux)
            self.assertTrue(any("dwm3000" in h for h in hits), msg=f"Expected dwm3000 hit, got {hits}")


if __name__ == "__main__":
    unittest.main()
