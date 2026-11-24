#!/usr/bin/env python3
"""Automated JSON Schema Draft 2020-12 & Semantic Invariant Validator for INTERFACE_CONTRACT.json."""

from __future__ import annotations

import argparse
import base64
import binascii
import copy
import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator, ValidationError

REQUIRED_SCHEMA_NAMES: tuple[str, ...] = (
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
)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONTRACT_PATH = REPO_ROOT / "INTERFACE_CONTRACT.json"
DEFAULT_META_SCHEMA_PATH = Path(__file__).resolve().parent / "interface_contract.meta.schema.json"

# Active file paths can be overridden via CLI for testing
_ACTIVE_CONTRACT_PATH = DEFAULT_CONTRACT_PATH
_ACTIVE_META_SCHEMA_PATH = DEFAULT_META_SCHEMA_PATH


class ContractInvariantError(ValueError):
    """Raised when a semantic, geometric, hardware, or binary-layout invariant fails."""


def _reject_non_standard_json_constant(constant: str) -> Any:
    raise ContractInvariantError(f"Non-standard JSON constant '{constant}' is forbidden (RFC 8259 strict JSON required)")


def loads_json_strict(text: str) -> Any:
    return json.loads(text, parse_constant=_reject_non_standard_json_constant)


def load_json_strict(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f, parse_constant=_reject_non_standard_json_constant)


def _mat3_det(m: list[list[float]]) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def _mat3_orthogonality_error(m: list[list[float]]) -> float:
    max_err = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(m[k][i] * m[k][j] for k in range(3))
            target = 1.0 if i == j else 0.0
            max_err = max(max_err, abs(dot - target))
    return max_err


def assert_proper_rotation_matrix(m: Any, name: str, tol: float = 1e-5) -> None:
    if not isinstance(m, list) or len(m) != 3 or any(not isinstance(r, list) or len(r) != 3 for r in m):
        raise ContractInvariantError(f"{name} must be a 3x3 matrix")
    ortho_err = _mat3_orthogonality_error(m)
    if ortho_err > tol:
        raise ContractInvariantError(f"{name} is not orthogonal (max |R^T R - I| = {ortho_err:.6g} > {tol})")
    det = _mat3_det(m)
    if abs(det - 1.0) > tol:
        raise ContractInvariantError(f"{name} must be a proper rotation with det=+1 (got det={det:.6g})")


def assert_unit_quaternion(q: Any, name: str, tol: float = 1e-6) -> None:
    if not isinstance(q, list) or len(q) != 4:
        raise ContractInvariantError(f"{name} must be a 4-element Hamilton quaternion [qw, qx, qy, qz]")
    norm = math.sqrt(sum(float(c) ** 2 for c in q))
    if abs(norm - 1.0) > tol:
        raise ContractInvariantError(f"{name} is not unit-norm (||q|| = {norm:.8f}, tol = {tol})")


def is_inside_rink(x: float, y: float, preset: dict[str, Any], tol: float = 1e-6) -> bool:
    half_l = preset["length_m"] / 2.0
    half_w = preset["width_m"] / 2.0
    rc = preset["corner_radius_m"]
    ax = abs(x)
    ay = abs(y)
    if ax > half_l + tol or ay > half_w + tol:
        return False
    core_x = half_l - rc
    core_y = half_w - rc
    if ax > core_x and ay > core_y:
        dist = math.hypot(ax - core_x, ay - core_y)
        if dist > rc + tol:
            return False
    return True


def _collect_external_refs(node: Any, refs: list[str]) -> None:
    if isinstance(node, dict):
        ref = node.get("$ref")
        if isinstance(ref, str) and not ref.startswith("#/"):
            refs.append(ref)
        for v in node.values():
            _collect_external_refs(v, refs)
    elif isinstance(node, list):
        for item in node:
            _collect_external_refs(item, refs)


def validate_schemas_and_examples(contract: dict[str, Any]) -> None:
    schemas = contract["schemas"]
    examples = contract["examples"]

    if set(schemas.keys()) != set(REQUIRED_SCHEMA_NAMES):
        raise ContractInvariantError(
            f"schemas keys mismatch: expected {sorted(REQUIRED_SCHEMA_NAMES)}, got {sorted(schemas.keys())}"
        )
    if set(examples.keys()) != set(REQUIRED_SCHEMA_NAMES):
        raise ContractInvariantError(
            f"examples keys mismatch: expected {sorted(REQUIRED_SCHEMA_NAMES)}, got {sorted(examples.keys())}"
        )

    for name in REQUIRED_SCHEMA_NAMES:
        schema = schemas[name]
        Draft202012Validator.check_schema(schema)
        ext_refs: list[str] = []
        _collect_external_refs(schema, ext_refs)
        if ext_refs:
            raise ContractInvariantError(
                f"Schema '{name}' contains non-local $refs (breaks standalone validation): {ext_refs}"
            )
        validator = Draft202012Validator(schema)
        validator.validate(examples[name])

    # Verify analytics_report.$defs stay structurally identical to the standalone schemas
    report_defs = schemas["analytics_report"].get("$defs", {})
    for sub_name in ("analytics_trajectory_point", "analytics_shift_summary", "analytics_session_summary"):
        if sub_name not in report_defs:
            raise ContractInvariantError(f"analytics_report.$defs missing '{sub_name}'")
        standalone_cmp = {k: v for k, v in schemas[sub_name].items() if k not in ("$schema", "$id")}
        def_cmp = {k: v for k, v in report_defs[sub_name].items() if k not in ("$schema", "$id")}
        if standalone_cmp != def_cmp:
            raise ContractInvariantError(
                f"analytics_report.$defs['{sub_name}'] drifted from $.schemas['{sub_name}']"
            )


def _verify_struct_sizes(bff: dict[str, Any]) -> None:
    hdr_size = struct.calcsize("<HBBHHQI")
    if hdr_size != bff["frame_header_bytes"]:
        raise ContractInvariantError(f"Frame header struct size {hdr_size} != {bff['frame_header_bytes']}")

    for mname, minfo in bff["message_types"].items():
        fmt = minfo["struct_format"]
        pbytes = minfo["payload_bytes"]
        if "+ N*" in fmt:
            hdr_fmt, smp_fmt = [part.strip() for part in fmt.split("+ N*", 1)]
            if not hdr_fmt.startswith("<") or not smp_fmt.startswith("<"):
                raise ContractInvariantError(
                    f"{mname} struct_format '{fmt}' must use little-endian '<' prefix on both header and sample formats"
                )
            hdr_len = struct.calcsize(hdr_fmt)
            smp_len = struct.calcsize(smp_fmt)
            expected_expr = f"{hdr_len} + {smp_len} * sample_count"
            if pbytes != expected_expr:
                raise ContractInvariantError(
                    f"{mname} payload_bytes '{pbytes}' != '{expected_expr}' from struct_format '{fmt}'"
                )
        else:
            if not fmt.startswith("<"):
                raise ContractInvariantError(f"{mname} struct_format '{fmt}' must use little-endian '<' prefix")
            calc_len = struct.calcsize(fmt)
            if pbytes != calc_len:
                raise ContractInvariantError(
                    f"{mname} payload_bytes {pbytes} != struct.calcsize('{fmt}')={calc_len}"
                )


def validate_rink_presets_semantics(presets: dict[str, Any]) -> None:
    for pname, preset in presets.items():
        half_l = preset["length_m"] / 2.0
        half_w = preset["width_m"] / 2.0
        rc = preset["corner_radius_m"]
        if rc < 0.0 or rc > min(half_l, half_w):
            raise ContractInvariantError(
                f"rink_geometry_presets.{pname}.corner_radius_m={rc} must satisfy 0 <= rc <= min(L/2, W/2)={min(half_l, half_w)}"
            )
        goal = preset["goal_line_from_boards_m"]
        blue = preset["blue_line_from_boards_m"]
        if not (0.0 < goal < blue < half_l):
            raise ContractInvariantError(
                f"rink_geometry_presets.{pname} must satisfy 0 < goal_line ({goal}) < blue_line ({blue}) < L/2 ({half_l})"
            )
        gate = preset["default_bench_gate_xy_m"]
        if not is_inside_rink(gate[0], gate[1], preset, tol=1e-6):
            raise ContractInvariantError(
                f"rink_geometry_presets.{pname}.default_bench_gate_xy_m={gate} lies outside rink bounds"
            )


def validate_packet_semantics(
    packet_name: str,
    packet: dict[str, Any],
    contract: dict[str, Any] | None = None,
) -> None:
    if packet_name == "calibration_block":
        assert_proper_rotation_matrix(
            packet["sensor_to_body_rotation_matrix"], f"{packet_name}.sensor_to_body_rotation_matrix"
        )
        assert_proper_rotation_matrix(
            packet["high_g_to_body_rotation_matrix"], f"{packet_name}.high_g_to_body_rotation_matrix"
        )
    elif packet_name in ("imu_batch_packet", "high_g_batch_packet"):
        if packet["sample_count"] != len(packet["samples"]):
            raise ContractInvariantError(
                f"{packet_name} sample_count ({packet['sample_count']}) != len(samples) ({len(packet['samples'])})"
            )
    elif packet_name == "flash_session_header":
        if contract is not None:
            preset = contract["rink_geometry_presets"][packet["rink_preset"]]
            if not is_inside_rink(packet["bench_gate_xy_m"][0], packet["bench_gate_xy_m"][1], preset, tol=0.05):
                raise ContractInvariantError(f"{packet_name}.bench_gate_xy_m outside rink bounds")
    elif packet_name == "download_response":
        payload = packet.get("payload", {})
        if "data_b64" in payload and "actual_len" in payload:
            try:
                raw = base64.b64decode(payload["data_b64"], validate=True)
            except (binascii.Error, ValueError) as exc:
                raise ContractInvariantError(f"{packet_name}.payload.data_b64 invalid base64: {exc}") from exc
            if len(raw) != payload["actual_len"]:
                raise ContractInvariantError(
                    f"{packet_name}.payload.data_b64 decoded length {len(raw)} != actual_len {payload['actual_len']}"
                )
        if "header_b64" in payload:
            try:
                base64.b64decode(payload["header_b64"], validate=True)
            except (binascii.Error, ValueError) as exc:
                raise ContractInvariantError(f"{packet_name}.payload.header_b64 invalid base64: {exc}") from exc


def validate_analytics_trajectory_point_semantics(
    pt: dict[str, Any],
    preset: dict[str, Any] | None = None,
    q_tol: float = 1e-6,
    name: str = "analytics_trajectory_point",
) -> None:
    assert_unit_quaternion(pt["q_WB"], f"{name}.q_WB", q_tol)
    expected_speed = math.hypot(pt["vx"], pt["vy"])
    if abs(pt["speed"] - expected_speed) > 1e-3:
        raise ContractInvariantError(f"{name}.speed {pt['speed']} != hypot(vx, vy)={expected_speed}")
    expected_amag = math.sqrt(pt["ax"] ** 2 + pt["ay"] ** 2 + pt["az"] ** 2)
    if abs(pt["accel_mag"] - expected_amag) > 1e-3:
        raise ContractInvariantError(f"{name}.accel_mag {pt['accel_mag']} != norm(a)={expected_amag}")
    if preset is not None and not is_inside_rink(pt["x"], pt["y"], preset):
        raise ContractInvariantError(f"{name} ({pt['x']}, {pt['y']}) outside rink bounds")


def validate_analytics_shift_semantics(
    shift: dict[str, Any],
    name: str = "analytics_shift_summary",
) -> None:
    if shift["end_s"] <= shift["start_s"]:
        raise ContractInvariantError(f"{name}: end_s ({shift['end_s']}) must be > start_s ({shift['start_s']})")
    expected_dur = shift["end_s"] - shift["start_s"]
    if abs(expected_dur - shift["duration_s"]) > 1e-6:
        raise ContractInvariantError(
            f"{name}: duration_s ({shift['duration_s']}) != end_s - start_s ({expected_dur})"
        )
    if shift["avg_speed_mps"] > shift["peak_speed_mps"]:
        raise ContractInvariantError(
            f"{name}: avg_speed_mps ({shift['avg_speed_mps']}) > peak_speed_mps ({shift['peak_speed_mps']})"
        )


def validate_analytics_session_summary_semantics(
    summary: dict[str, Any],
    name: str = "analytics_session_summary",
) -> None:
    if summary["total_ice_time_s"] > summary["total_duration_s"]:
        raise ContractInvariantError(
            f"{name}: total_ice_time_s ({summary['total_ice_time_s']}) > total_duration_s ({summary['total_duration_s']})"
        )


def validate_analytics_report_semantics(
    report: dict[str, Any],
    presets: dict[str, Any] | None = None,
    q_tol: float = 1e-6,
) -> None:
    summary = report["session_summary"]
    validate_analytics_session_summary_semantics(summary, "analytics_report.session_summary")
    if summary["shift_count"] != len(report["shifts"]):
        raise ContractInvariantError(
            f"analytics_report.session_summary.shift_count ({summary['shift_count']}) != len(shifts) ({len(report['shifts'])})"
        )
    for idx, shift in enumerate(report["shifts"]):
        validate_analytics_shift_semantics(shift, f"analytics_report.shifts[{idx}]")

    rep_preset = presets[summary["rink_preset"]] if presets is not None else None
    for idx, rpt in enumerate(report["trajectory"]):
        validate_analytics_trajectory_point_semantics(
            rpt,
            preset=rep_preset,
            q_tol=q_tol,
            name=f"analytics_report.trajectory[{idx}]",
        )


def verify_semantic_invariants(contract: dict[str, Any]) -> None:
    constants = contract["constants"]
    frames = contract["coordinate_frames"]
    hw = contract["hardware_abstraction"]
    bff = contract["binary_flash_format"]
    dp = contract["download_protocol"]
    presets = contract["rink_geometry_presets"]
    examples = contract["examples"]

    # 1. Non-GPS, non-anchor, >= 2h battery, hardware bus & address invariants
    if constants["gps_enabled"] is not False or constants["external_anchors_enabled"] is not False:
        raise ContractInvariantError("GPS and external anchors must be disabled in constants")
    if hw["uses_gps"] is not False or hw["uses_external_anchors"] is not False:
        raise ContractInvariantError("Hardware must not rely on GPS or external anchors")
    if contract["units_and_tolerances"]["gps_or_anchors_used"] is not False:
        raise ContractInvariantError("units_and_tolerances.gps_or_anchors_used must be false")
    if hw["min_battery_life_hours"] < 2.0:
        raise ContractInvariantError("min_battery_life_hours must be >= 2.0")

    sensors = hw["sensors"]
    if sensors["primary_imu"]["bus"] == sensors["high_g_imu"]["bus"]:
        raise ContractInvariantError("ADXL372 (10 MHz max SPI) must be isolated from ICM-42688-P SPI bus")
    if sensors["barometer"]["part_number"] == "BMP581" and sensors["barometer"]["i2c_addr_hex"] not in ("0x47", "0x46"):
        raise ContractInvariantError("BMP581 7-bit I2C address must be 0x47 (SDO=1V8) or 0x46 (SDO=GND)")
    if hw["flash"]["usable_ring_blocks"] * hw["flash"]["block_bytes"] < 130_000_000:
        raise ContractInvariantError("Flash TELEMETRY_RING usable capacity unexpectedly low")

    # 2. Timestamp >= 64-bit checks & binary frame header packing checks
    if constants["timestamp_width_bits"] < 64 or constants["timestamp_type"] != "uint64_t":
        raise ContractInvariantError("constants must enforce >= 64-bit uint64_t timestamp")
    if bff["timestamp_bits"] < 64 or bff["timestamp_type"] != "uint64_t":
        raise ContractInvariantError("binary_flash_format must enforce >= 64-bit uint64_t timestamp")

    expected_offset = 0
    found_ts = False
    for field in sorted(bff["header_fields"], key=lambda f: f["offset"]):
        if field["offset"] != expected_offset:
            raise ContractInvariantError(
                f"Header field '{field['name']}' offset {field['offset']} != expected {expected_offset}"
            )
        expected_offset += field["size"]
        if field["name"] == bff["timestamp_field"]:
            found_ts = True
            if field["size"] < 8 or field["c_type"] != "uint64_t":
                raise ContractInvariantError("Binary header timestamp_us must be 8-byte uint64_t")
    if not found_ts:
        raise ContractInvariantError("Binary header missing timestamp_us field")
    if expected_offset != bff["frame_header_bytes"]:
        raise ContractInvariantError(
            f"Header field sizes sum to {expected_offset}, expected {bff['frame_header_bytes']}"
        )

    _verify_struct_sizes(bff)

    # 3. CRC & Sync consistency across sections
    if bff["crc_polynomial"] != constants["crc32_polynomial_reflected"]:
        raise ContractInvariantError("CRC32 polynomial mismatch between constants and binary_flash_format")
    if not (constants["sync_marker_u16"] == bff["sync_u16"] == dp["request_sync_u16"]):
        raise ContractInvariantError("Request/frame sync marker 0xAA55 mismatch")
    if constants["response_sync_marker_u16"] != dp["response_sync_u16"]:
        raise ContractInvariantError("Response sync marker 0x55AA mismatch")

    msg_ids = [v["msg_type_id"] for v in bff["message_types"].values()]
    if len(msg_ids) != len(set(msg_ids)):
        raise ContractInvariantError("Duplicate msg_type_id in binary_flash_format.message_types")
    for mname, minfo in bff["message_types"].items():
        ref = minfo["schema_ref"]
        if not ref.startswith("#/schemas/") or ref.split("/")[-1] not in contract["schemas"]:
            raise ContractInvariantError(f"Invalid schema_ref '{ref}' on message_type '{mname}'")

    if len(set(dp["commands"].values())) != len(dp["commands"]):
        raise ContractInvariantError("Duplicate command hex codes in download_protocol")
    if len(set(dp["status_codes"].values())) != len(dp["status_codes"]):
        raise ContractInvariantError("Duplicate status hex codes in download_protocol")

    # 4. Canonical R_BS and calibration rotation matrices SO(3): orthogonal & det = +1
    canonical_r_bs = frames["frames"]["S"]["canonical_R_BS"]
    assert_proper_rotation_matrix(canonical_r_bs, "coordinate_frames.frames.S.canonical_R_BS")
    cal = examples["calibration_block"]
    validate_packet_semantics("calibration_block", cal, contract)
    if cal["sensor_to_body_rotation_matrix"] != canonical_r_bs:
        raise ContractInvariantError("examples.calibration_block.sensor_to_body_rotation_matrix != canonical_R_BS")

    # 5. Quaternion unit-norm checks
    q_tol = frames["quaternion_convention"]["unit_norm_tolerance"]
    assert_unit_quaternion(frames["frames"]["B"]["identity_quaternion_q_WB"], "identity_quaternion_q_WB", q_tol)

    # 6. Example packet consistency & >71.5 min 64-bit timestamp exercise
    validate_packet_semantics("imu_batch_packet", examples["imu_batch_packet"], contract)
    validate_packet_semantics("high_g_batch_packet", examples["high_g_batch_packet"], contract)
    validate_packet_semantics("flash_session_header", examples["flash_session_header"], contract)
    validate_packet_semantics("download_response", examples["download_response"], contract)
    if examples["imu_batch_packet"]["timestamp_us"] <= 0xFFFFFFFF:
        raise ContractInvariantError(
            "Example imu_batch_packet should demonstrate timestamp_us > 32-bit max (4294967295)"
        )

    # 7. Rink presets, trajectory kinematics, shifts, session summary & analytics_report consistency
    validate_rink_presets_semantics(presets)
    validate_analytics_trajectory_point_semantics(
        examples["analytics_trajectory_point"],
        preset=presets["NHL_200X85FT"],
        q_tol=q_tol,
        name="examples.analytics_trajectory_point",
    )
    validate_analytics_shift_semantics(examples["analytics_shift_summary"], "examples.analytics_shift_summary")
    validate_analytics_session_summary_semantics(
        examples["analytics_session_summary"], "examples.analytics_session_summary"
    )
    validate_analytics_report_semantics(examples["analytics_report"], presets=presets, q_tol=q_tol)


def validate_contract_files(
    contract_path: Path = DEFAULT_CONTRACT_PATH,
    meta_schema_path: Path = DEFAULT_META_SCHEMA_PATH,
) -> tuple[dict[str, Any], dict[str, Any]]:
    contract = load_json_strict(contract_path)
    meta_schema = load_json_strict(meta_schema_path)

    Draft202012Validator.check_schema(meta_schema)
    Draft202012Validator(meta_schema).validate(contract)
    validate_schemas_and_examples(contract)
    verify_semantic_invariants(contract)
    return contract, meta_schema


class TestInterfaceContractPositive(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract, cls.meta_schema = validate_contract_files(
            _ACTIVE_CONTRACT_PATH, _ACTIVE_META_SCHEMA_PATH
        )

    def test_meta_schema_and_all_13_schemas_and_examples_valid(self) -> None:
        Draft202012Validator.check_schema(self.meta_schema)
        Draft202012Validator(self.meta_schema).validate(self.contract)
        validate_schemas_and_examples(self.contract)
        verify_semantic_invariants(self.contract)

    def test_all_13_schemas_validate_standalone(self) -> None:
        for name in REQUIRED_SCHEMA_NAMES:
            with self.subTest(schema=name):
                schema = self.contract["schemas"][name]
                Draft202012Validator.check_schema(schema)
                Draft202012Validator(schema).validate(self.contract["examples"][name])

    def test_binary_payload_struct_sizes_match_contract(self) -> None:
        _verify_struct_sizes(self.contract["binary_flash_format"])
        msg_types = self.contract["binary_flash_format"]["message_types"]
        self.assertEqual(msg_types["SESSION_HEADER"]["payload_bytes"], 80)
        self.assertEqual(msg_types["IMU_BATCH"]["payload_bytes"], "8 + 12 * sample_count")
        self.assertEqual(msg_types["HIGH_G_BATCH"]["payload_bytes"], "8 + 6 * sample_count")
        self.assertEqual(msg_types["AUX_ENV"]["payload_bytes"], 28)
        self.assertEqual(msg_types["EVENT_MARKER"]["payload_bytes"], 16)
        self.assertEqual(msg_types["CALIBRATION_BLOCK"]["payload_bytes"], 288)
        self.assertEqual(msg_types["SESSION_FOOTER"]["payload_bytes"], 72)

    def test_accepts_all_valid_download_commands_and_responses(self) -> None:
        cmd_validator = Draft202012Validator(self.contract["schemas"]["download_command"])
        rsp_validator = Draft202012Validator(self.contract["schemas"]["download_response"])

        valid_cmds = [
            ("LIST_SESSIONS", 1, {}),
            ("GET_HEADER", 2, {"session_id": 1001}),
            ("READ_CHUNK", 3, {"session_id": 1001, "offset_bytes": 0, "length_bytes": 4096}),
            ("ACK_VERIFY", 4, {"session_id": 1001, "computed_file_crc32": 123456}),
            ("ACK_VERIFY", 4, {"session_id": 1001, "computed_file_crc32": 123456, "computed_sha256": "a" * 64}),
            ("DELETE_SESSION", 5, {"session_id": 1001, "force_flag": 0}),
            ("GET_DEVICE_STATUS", 6, {}),
            ("SET_UTC_TIME", 7, {"utc_ms": 1763852500000}),
        ]
        for command, cmd_id, payload in valid_cmds:
            with self.subTest(command=command, payload=payload):
                cmd_validator.validate(
                    {"sync": 43605, "command": command, "cmd_id": cmd_id, "seq": 1, "payload": payload, "crc32": 0}
                )

        statuses = [
            ("OK", 0),
            ("ERR_BAD_CRC", 1),
            ("ERR_UNKNOWN_CMD", 2),
            ("ERR_SESSION_NOT_FOUND", 3),
            ("ERR_OFFSET_OUT_OF_RANGE", 4),
            ("ERR_NOT_VERIFIED", 5),
            ("ERR_FLASH_IO", 6),
            ("ERR_BUSY_RECORDING", 7),
            ("ERR_VERIFY_MISMATCH", 8),
            ("ERR_INVALID_PARAM", 9),
        ]
        for command, cmd_id, _ in [c for c in valid_cmds if not (c[0] == "ACK_VERIFY" and "computed_sha256" in c[2])]:
            for status, status_code in statuses:
                with self.subTest(command=command, status=status):
                    rsp_validator.validate(
                        {
                            "sync": 21930,
                            "command": command,
                            "cmd_id": cmd_id,
                            "status": status,
                            "status_code": status_code,
                            "seq": 1,
                            "payload": {},
                            "crc32": 0,
                        }
                    )

    def test_accepts_unclipped_adxl372_raw_minus_2048_in_trajectory_and_packet(self) -> None:
        raw_min_mps2 = -2048 * 0.980665  # -2008.40192 m/s^2
        pt = copy.deepcopy(self.contract["examples"]["analytics_trajectory_point"])
        pt["ax"] = raw_min_mps2
        pt["ay"] = 0.0
        pt["az"] = 0.0
        pt["accel_mag"] = abs(raw_min_mps2)
        Draft202012Validator(self.contract["schemas"]["analytics_trajectory_point"]).validate(pt)

        rep = copy.deepcopy(self.contract["examples"]["analytics_report"])
        rep["trajectory"][0] = pt
        Draft202012Validator(self.contract["schemas"]["analytics_report"]).validate(rep)
        validate_analytics_report_semantics(rep, presets=self.contract["rink_geometry_presets"])

        hg = copy.deepcopy(self.contract["examples"]["high_g_batch_packet"])
        hg["samples"][0]["ax_i16"] = -2048
        Draft202012Validator(self.contract["schemas"]["high_g_batch_packet"]).validate(hg)


class TestInterfaceContractNegative(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base_contract = load_json_strict(_ACTIVE_CONTRACT_PATH)
        cls.meta_schema = load_json_strict(_ACTIVE_META_SCHEMA_PATH)

    def test_rejects_unknown_top_level_property(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["unexpected_top_field"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(self.meta_schema).validate(bad)

    def test_rejects_missing_required_schema_entry(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        del bad["schemas"]["high_g_batch_packet"]
        with self.assertRaises(ValidationError):
            Draft202012Validator(self.meta_schema).validate(bad)

    def test_rejects_gps_enabled(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["constants"]["gps_enabled"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(self.meta_schema).validate(bad)

    def test_rejects_32bit_timestamp_violation(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["constants"]["timestamp_width_bits"] = 32
        bad["constants"]["timestamp_type"] = "uint32_t"
        with self.assertRaises(ValidationError):
            Draft202012Validator(self.meta_schema).validate(bad)

        bad_sem = copy.deepcopy(self.base_contract)
        for fld in bad_sem["binary_flash_format"]["header_fields"]:
            if fld["name"] == "timestamp_us":
                fld["c_type"] = "uint32_t"
                fld["size"] = 4
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_sem)

    def test_rejects_reflection_rotation_matrix_det_minus_one(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["examples"]["calibration_block"]["sensor_to_body_rotation_matrix"] = [
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad)

    def test_rejects_non_orthogonal_rotation_matrix(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["examples"]["calibration_block"]["sensor_to_body_rotation_matrix"] = [
            [2.0, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 1.0],
        ]
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad)

    def test_rejects_non_unit_quaternion(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["examples"]["analytics_trajectory_point"]["q_WB"] = [1.0, 1.0, 0.0, 0.0]
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad)

    def test_rejects_out_of_rink_corner_trajectory_point(self) -> None:
        bad = copy.deepcopy(self.base_contract)
        bad["examples"]["analytics_trajectory_point"]["x"] = 29.5
        bad["examples"]["analytics_trajectory_point"]["y"] = 12.2
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad)

    def test_rejects_bad_types_and_unknown_packet_field(self) -> None:
        schema = self.base_contract["schemas"]["imu_batch_packet"]
        validator = Draft202012Validator(schema)

        bad_type = copy.deepcopy(self.base_contract["examples"]["imu_batch_packet"])
        bad_type["sample_count"] = "twenty"
        with self.assertRaises(ValidationError):
            validator.validate(bad_type)

        bad_extra = copy.deepcopy(self.base_contract["examples"]["imu_batch_packet"])
        bad_extra["extra_field"] = 99
        with self.assertRaises(ValidationError):
            validator.validate(bad_extra)

    def test_rejects_missing_required_field_in_analytics_report(self) -> None:
        schema = self.base_contract["schemas"]["analytics_report"]
        validator = Draft202012Validator(schema)

        bad_report = copy.deepcopy(self.base_contract["examples"]["analytics_report"])
        del bad_report["trajectory"][0]["q_WB"]
        with self.assertRaises(ValidationError):
            validator.validate(bad_report)

    def test_rejects_invalid_nested_shift_in_analytics_report(self) -> None:
        schema = self.base_contract["schemas"]["analytics_report"]
        validator = Draft202012Validator(schema)

        bad_report = copy.deepcopy(self.base_contract["examples"]["analytics_report"])
        bad_report["shifts"][0]["duration_s"] = 0.0
        with self.assertRaises(ValidationError):
            validator.validate(bad_report)

    def test_rejects_bad_crc_and_frame_offsets(self) -> None:
        bad_crc = copy.deepcopy(self.base_contract)
        bad_crc["constants"]["crc32_polynomial_reflected"] = "0x04C11DB7"
        with self.assertRaises(ValidationError):
            Draft202012Validator(self.meta_schema).validate(bad_crc)

        bad_offset = copy.deepcopy(self.base_contract)
        bad_offset["binary_flash_format"]["header_fields"][-1]["offset"] = 18
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_offset)

    def test_rejects_shared_spi_bus_and_wrong_bmp581_addr(self) -> None:
        bad_bus = copy.deepcopy(self.base_contract)
        bad_bus["hardware_abstraction"]["sensors"]["high_g_imu"]["bus"] = "SPIM4"
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_bus)

        bad_addr = copy.deepcopy(self.base_contract)
        bad_addr["hardware_abstraction"]["sensors"]["barometer"]["i2c_addr_hex"] = "0x77"
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_addr)

    def test_rejects_download_response_command_and_status_mismatches(self) -> None:
        validator = Draft202012Validator(self.base_contract["schemas"]["download_response"])

        bad_cmd = copy.deepcopy(self.base_contract["examples"]["download_response"])
        bad_cmd["command"] = "READ_CHUNK"
        bad_cmd["cmd_id"] = 7
        with self.assertRaises(ValidationError):
            validator.validate(bad_cmd)

        bad_status = copy.deepcopy(self.base_contract["examples"]["download_response"])
        bad_status["status"] = "OK"
        bad_status["status_code"] = 9
        with self.assertRaises(ValidationError):
            validator.validate(bad_status)

    def test_rejects_download_command_sibling_payload_keys(self) -> None:
        validator = Draft202012Validator(self.base_contract["schemas"]["download_command"])
        sibling_cases = [
            ("LIST_SESSIONS", 1, {"utc_ms": 12345}),
            ("GET_DEVICE_STATUS", 6, {"session_id": 1001}),
            ("GET_HEADER", 2, {"session_id": 1001, "force_flag": 1}),
            ("READ_CHUNK", 3, {"session_id": 1001, "offset_bytes": 0, "length_bytes": 16, "utc_ms": 1}),
            ("ACK_VERIFY", 4, {"session_id": 1001, "computed_file_crc32": 0, "force_flag": 0}),
            ("DELETE_SESSION", 5, {"session_id": 1001, "force_flag": 0, "utc_ms": 1}),
            ("SET_UTC_TIME", 7, {"utc_ms": 1763852500000, "session_id": 1001}),
        ]
        for command, cmd_id, payload in sibling_cases:
            with self.subTest(command=command, payload=payload):
                with self.assertRaises(ValidationError):
                    validator.validate(
                        {
                            "sync": 43605,
                            "command": command,
                            "cmd_id": cmd_id,
                            "seq": 1,
                            "payload": payload,
                            "crc32": 0,
                        }
                    )

    def test_rejects_shift_time_and_speed_invariants(self) -> None:
        # end_s < start_s and end_s == start_s
        for bad_end in (4.0, 5.0):
            bad_shift = copy.deepcopy(self.base_contract)
            bad_shift["examples"]["analytics_shift_summary"]["end_s"] = bad_end
            with self.assertRaises(ContractInvariantError):
                verify_semantic_invariants(bad_shift)

            bad_rep = copy.deepcopy(self.base_contract)
            bad_rep["examples"]["analytics_report"]["shifts"][0]["end_s"] = bad_end
            with self.assertRaises(ContractInvariantError):
                verify_semantic_invariants(bad_rep)

        # duration_s != end_s - start_s
        bad_dur = copy.deepcopy(self.base_contract)
        bad_dur["examples"]["analytics_shift_summary"]["duration_s"] = 10.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_dur)

        bad_rep_dur = copy.deepcopy(self.base_contract)
        bad_rep_dur["examples"]["analytics_report"]["shifts"][0]["duration_s"] = 10.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_rep_dur)

        # avg_speed_mps > peak_speed_mps
        bad_spd = copy.deepcopy(self.base_contract)
        bad_spd["examples"]["analytics_shift_summary"]["avg_speed_mps"] = 9.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_spd)

        bad_rep_spd = copy.deepcopy(self.base_contract)
        bad_rep_spd["examples"]["analytics_report"]["shifts"][0]["avg_speed_mps"] = 9.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_rep_spd)

    def test_rejects_session_ice_time_and_shift_count_mismatches(self) -> None:
        bad_sess = copy.deepcopy(self.base_contract)
        bad_sess["examples"]["analytics_session_summary"]["total_ice_time_s"] = 99999.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_sess)

        bad_rep_ice = copy.deepcopy(self.base_contract)
        bad_rep_ice["examples"]["analytics_report"]["session_summary"]["total_ice_time_s"] = 99999.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_rep_ice)

        bad_cnt = copy.deepcopy(self.base_contract)
        bad_cnt["examples"]["analytics_report"]["session_summary"]["shift_count"] = 2
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_cnt)

    def test_rejects_report_trajectory_speed_and_accel_mag_mismatches(self) -> None:
        bad_spd = copy.deepcopy(self.base_contract)
        bad_spd["examples"]["analytics_report"]["trajectory"][0]["speed"] = 1.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_spd)

        bad_acc = copy.deepcopy(self.base_contract)
        bad_acc["examples"]["analytics_report"]["trajectory"][0]["accel_mag"] = 10.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_acc)

    def test_rejects_download_response_data_b64_length_mismatch(self) -> None:
        bad_len = copy.deepcopy(self.base_contract)
        bad_len["examples"]["download_response"]["payload"]["actual_len"] = 17
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_len)

        bad_b64 = copy.deepcopy(self.base_contract)
        bad_b64["examples"]["download_response"]["payload"]["data_b64"] = "not-valid-base64!!!"
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_b64)

    def test_rejects_invalid_rink_geometry_presets(self) -> None:
        bad_rc = copy.deepcopy(self.base_contract)
        bad_rc["rink_geometry_presets"]["CUSTOM"]["corner_radius_m"] = 20.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_rc)

        bad_lines = copy.deepcopy(self.base_contract)
        bad_lines["rink_geometry_presets"]["CUSTOM"]["goal_line_from_boards_m"] = 22.0
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_lines)

        bad_gate = copy.deepcopy(self.base_contract)
        bad_gate["rink_geometry_presets"]["IIHF_60X30M"]["default_bench_gate_xy_m"] = [-6.0, 20.0]
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_gate)

    def test_rejects_big_endian_struct_format_and_header_sum_mismatch(self) -> None:
        bad_endian_fixed = copy.deepcopy(self.base_contract)
        bad_endian_fixed["binary_flash_format"]["message_types"]["EVENT_MARKER"]["struct_format"] = ">HHfQ"
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_endian_fixed)

        bad_endian_batch = copy.deepcopy(self.base_contract)
        bad_endian_batch["binary_flash_format"]["message_types"]["IMU_BATCH"]["struct_format"] = "<HHhH + N*>hhhhhh"
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_endian_batch)

        bad_hdr_start = copy.deepcopy(self.base_contract)
        bad_hdr_start["binary_flash_format"]["header_fields"][0]["offset"] = 2
        with self.assertRaises(ContractInvariantError):
            verify_semantic_invariants(bad_hdr_start)

    def test_rejects_nan_and_infinity_json_constants(self) -> None:
        for bad_json in ('{"a": NaN}', '{"a": Infinity}', '{"a": -Infinity}'):
            with self.subTest(bad_json=bad_json):
                with self.assertRaises(ContractInvariantError):
                    loads_json_strict(bad_json)

        with tempfile.TemporaryDirectory() as tmpdir:
            bad_path = Path(tmpdir) / "bad_nan.json"
            bad_path.write_text('{"x": NaN}', encoding="utf-8")
            with self.assertRaises(ContractInvariantError):
                validate_contract_files(bad_path, _ACTIVE_META_SCHEMA_PATH)


def main(argv: list[str] | None = None) -> int:
    global _ACTIVE_CONTRACT_PATH, _ACTIVE_META_SCHEMA_PATH

    parser = argparse.ArgumentParser(description="Validate INTERFACE_CONTRACT.json and run schema tests.")
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT_PATH, help="Path to INTERFACE_CONTRACT.json")
    parser.add_argument(
        "--meta-schema",
        type=Path,
        default=DEFAULT_META_SCHEMA_PATH,
        help="Path to interface_contract.meta.schema.json",
    )
    parser.add_argument("--skip-tests", action="store_true", help="Only validate contract files without running unittest")
    args, remaining = parser.parse_known_args(argv)

    _ACTIVE_CONTRACT_PATH = args.contract.resolve()
    _ACTIVE_META_SCHEMA_PATH = args.meta_schema.resolve()

    contract, _ = validate_contract_files(_ACTIVE_CONTRACT_PATH, _ACTIVE_META_SCHEMA_PATH)
    print(
        f"CONTRACT_VALID: {_ACTIVE_CONTRACT_PATH.name} v{contract['contract_version']} "
        f"({len(contract['schemas'])} schemas, {len(contract['examples'])} examples, semantic invariants OK)"
    )

    if args.skip_tests:
        return 0

    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
