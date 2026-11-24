"""Opaque-box contract loaders, binary codecs, math/geometry oracles, strict resolvers, and rubric helpers."""

from __future__ import annotations

import base64
from bisect import bisect_right
import contextlib
import copy
from dataclasses import asdict, dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import zlib
from typing import Any, Sequence

from jsonschema import Draft202012Validator, ValidationError

REPO_ROOT = Path(__file__).resolve().parent.parent
SPEC_PATH = REPO_ROOT / "PROJECT_SPEC.md"
CONTRACT_PATH = REPO_ROOT / "INTERFACE_CONTRACT.json"
META_SCHEMA_PATH = REPO_ROOT / "docs" / "interface_contract.meta.schema.json"
ARCH_PATH = REPO_ROOT / "docs" / "architecture.md"
VALIDATE_CONTRACT_PY = REPO_ROOT / "docs" / "validate_contract.py"

HARDWARE_DIR = REPO_ROOT / "hardware"
FIRMWARE_DIR = REPO_ROOT / "firmware"
SIM_DIR = REPO_ROOT / "sim"

# Protocol & Binary Constants (locked in INTERFACE_CONTRACT.json v1.0.0)
GRAVITY_MPS2 = 9.80665
SYNC_U16 = 0xAA55
RSP_SYNC_U16 = 0x55AA
SESSION_MAGIC = b"HKS1"
MAX_PAYLOAD_BYTES = 4072
FRAME_HEADER_BYTES = 20
FRAME_CRC_BYTES = 4
MAX_CHUNK_BYTES = 4096

ICM_ACCEL_SCALE_MPS2_PER_LSB = 9.80665 / 2048.0  # 0.0047884033203125
ICM_GYRO_SCALE_RADS_PER_LSB = (math.pi / 180.0) / 16.4  # 0.001064225153655079
ADXL372_SCALE_MPS2_PER_LSB = 0.980665  # 100 mg/LSB right-aligned >> 4
MMC5983MA_SCALE_UT_PER_LSB = 100.0 / 16384.0  # 0.006103515625
BMP581_SCALE_PA_PER_LSB = 1.0 / 64.0  # 0.015625

TORSO_RADIUS_M = 0.25
FLASH_USABLE_RING_BLOCKS = 998
FLASH_BLOCK_BYTES = 131072
FLASH_USABLE_RING_BYTES = FLASH_USABLE_RING_BLOCKS * FLASH_BLOCK_BYTES  # 130,809,856 B

# Binary .hks Message Type IDs
MSG_SESSION_HEADER = 0x01
MSG_IMU_BATCH = 0x02
MSG_HIGH_G_BATCH = 0x03
MSG_AUX_ENV = 0x04
MSG_EVENT_MARKER = 0x05
MSG_CALIBRATION_BLOCK = 0x06
MSG_SESSION_FOOTER = 0x07

# Struct formats
FRAME_HDR_FMT = "<HBBHHQI"
SESSION_HEADER_FMT = "<4sHHIIQQQHHHHffffBBHffI"
IMU_BATCH_HDR_FMT = "<HHhH"
IMU_SAMPLE_FMT = "<hhhhhh"
HIGH_G_BATCH_HDR_FMT = "<HHBBH"
HIGH_G_SAMPLE_FMT = "<hhh"
AUX_ENV_FMT = "<iiiIhHhHHH"
EVENT_MARKER_FMT = "<HHfQ"
CALIBRATION_BLOCK_FMT = "<HHQ9f9f3f9f3f9f9f3f3f9fffI"
SESSION_FOOTER_FMT = "<QIIIIIIQ32s"

# Enum mappings
RINK_PRESET_TO_ID = {"NHL_200X85FT": 0, "IIHF_60X30M": 1, "CUSTOM": 2}
ID_TO_RINK_PRESET = {v: k for k, v in RINK_PRESET_TO_ID.items()}

MOUNT_TO_ID = {"STERNUM_POCKET": 0, "CHEST_STRAP": 1}
ID_TO_MOUNT = {v: k for k, v in MOUNT_TO_ID.items()}

TRIGGER_MODE_TO_ID = {"CONTINUOUS": 0, "IMPACT_BURST": 1}
ID_TO_TRIGGER_MODE = {v: k for k, v in TRIGGER_MODE_TO_ID.items()}

EVENT_CODE_TO_ID = {
    "SESSION_START": 1,
    "BUTTON_MARK": 2,
    "IMPACT_SPIKE": 3,
    "SHIFT_HINT_ON": 4,
    "SHIFT_HINT_OFF": 5,
    "LOW_BATT": 6,
    "BOARD_COLLISION": 7,
    "SESSION_END": 8,
}
ID_TO_EVENT_CODE = {v: k for k, v in EVENT_CODE_TO_ID.items()}

END_REASON_TO_ID = {
    "BUTTON_STOP": 0,
    "USB_DOCK": 1,
    "LOW_BATT": 2,
    "FLASH_FULL": 3,
    "WATCHDOG_RECOVER": 4,
}
ID_TO_END_REASON = {v: k for k, v in END_REASON_TO_ID.items()}

# Download Protocol Commands & Status Codes
CMD_LIST_SESSIONS = 0x01
CMD_GET_HEADER = 0x02
CMD_READ_CHUNK = 0x03
CMD_ACK_VERIFY = 0x04
CMD_DELETE_SESSION = 0x05
CMD_GET_DEVICE_STATUS = 0x06
CMD_SET_UTC_TIME = 0x07

CMD_NAME_TO_ID = {
    "LIST_SESSIONS": CMD_LIST_SESSIONS,
    "GET_HEADER": CMD_GET_HEADER,
    "READ_CHUNK": CMD_READ_CHUNK,
    "ACK_VERIFY": CMD_ACK_VERIFY,
    "DELETE_SESSION": CMD_DELETE_SESSION,
    "GET_DEVICE_STATUS": CMD_GET_DEVICE_STATUS,
    "SET_UTC_TIME": CMD_SET_UTC_TIME,
}
ID_TO_CMD_NAME = {v: k for k, v in CMD_NAME_TO_ID.items()}

STATUS_OK = 0x00
STATUS_ERR_BAD_CRC = 0x01
STATUS_ERR_UNKNOWN_CMD = 0x02
STATUS_ERR_SESSION_NOT_FOUND = 0x03
STATUS_ERR_OFFSET_OUT_OF_RANGE = 0x04
STATUS_ERR_NOT_VERIFIED = 0x05
STATUS_ERR_FLASH_IO = 0x06
STATUS_ERR_BUSY_RECORDING = 0x07
STATUS_ERR_VERIFY_MISMATCH = 0x08
STATUS_ERR_INVALID_PARAM = 0x09

STATUS_NAME_TO_CODE = {
    "OK": STATUS_OK,
    "ERR_BAD_CRC": STATUS_ERR_BAD_CRC,
    "ERR_UNKNOWN_CMD": STATUS_ERR_UNKNOWN_CMD,
    "ERR_SESSION_NOT_FOUND": STATUS_ERR_SESSION_NOT_FOUND,
    "ERR_OFFSET_OUT_OF_RANGE": STATUS_ERR_OFFSET_OUT_OF_RANGE,
    "ERR_NOT_VERIFIED": STATUS_ERR_NOT_VERIFIED,
    "ERR_FLASH_IO": STATUS_ERR_FLASH_IO,
    "ERR_BUSY_RECORDING": STATUS_ERR_BUSY_RECORDING,
    "ERR_VERIFY_MISMATCH": STATUS_ERR_VERIFY_MISMATCH,
    "ERR_INVALID_PARAM": STATUS_ERR_INVALID_PARAM,
}
CODE_TO_STATUS_NAME = {v: k for k, v in STATUS_NAME_TO_CODE.items()}

FORBIDDEN_BOM_KEYWORDS = (
    "gps",
    "gnss",
    "ublox",
    "u-blox",
    "uwb",
    "dwm1000",
    "dwm3000",
    "dw3000",
    "anchor",
    "beacon",
    "base_station",
    "basestation",
)

# Caches
_CONTRACT_CACHE: dict[str, Any] | None = None
_META_SCHEMA_CACHE: dict[str, Any] | None = None
_VALIDATOR_CACHE: dict[str, Draft202012Validator] = {}
_META_VALIDATOR_CACHE: Draft202012Validator | None = None
_VALIDATE_CONTRACT_CLI_CACHE: dict[tuple[str, ...], subprocess.CompletedProcess[str]] = {}
_POWER_BUDGET_CACHE: dict[str, Any] | None = None
_HW_TESTS_CACHE: subprocess.CompletedProcess[str] | None = None
_FW_MAKE_TEST_CACHE: subprocess.CompletedProcess[str] | None = None
_SIM_TESTS_CACHE: subprocess.CompletedProcess[str] | None = None
_SIM_ANALYZE_CACHE: dict[tuple[Any, ...], tuple[bytes, dict[str, Any], dict[str, Any]]] = {}
_FW_SIM_CACHE: dict[bool, tuple[bytes, dict[str, Any], subprocess.CompletedProcess[str]]] = {}


def clear_caches() -> None:
    """Reset module-level caches (used in mutation/self-validator tests)."""
    global _CONTRACT_CACHE, _META_SCHEMA_CACHE, _META_VALIDATOR_CACHE
    global _POWER_BUDGET_CACHE, _HW_TESTS_CACHE, _FW_MAKE_TEST_CACHE, _SIM_TESTS_CACHE
    _CONTRACT_CACHE = None
    _META_SCHEMA_CACHE = None
    _VALIDATOR_CACHE.clear()
    _META_VALIDATOR_CACHE = None
    _VALIDATE_CONTRACT_CLI_CACHE.clear()
    _POWER_BUDGET_CACHE = None
    _HW_TESTS_CACHE = None
    _FW_MAKE_TEST_CACHE = None
    _SIM_TESTS_CACHE = None
    _SIM_ANALYZE_CACHE.clear()
    _FW_SIM_CACHE.clear()


def _reject_non_standard_json_constant(constant: str) -> Any:
    raise ValueError(f"Non-standard JSON constant '{constant}' forbidden (RFC 8259 strict JSON)")


def loads_json_strict(text: str) -> Any:
    return json.loads(text, parse_constant=_reject_non_standard_json_constant)


def load_json_strict(path: Path) -> Any:
    return loads_json_strict(path.read_text(encoding="utf-8"))


def load_spec_text() -> str:
    if not SPEC_PATH.is_file():
        raise AssertionError(f"Missing M1 specification file: {SPEC_PATH}")
    return SPEC_PATH.read_text(encoding="utf-8")


def load_arch_text() -> str:
    if not ARCH_PATH.is_file():
        raise AssertionError(f"Missing M1 architecture file: {ARCH_PATH}")
    return ARCH_PATH.read_text(encoding="utf-8")


def load_contract(use_cache: bool = True) -> dict[str, Any]:
    global _CONTRACT_CACHE
    if use_cache and _CONTRACT_CACHE is not None:
        return _CONTRACT_CACHE
    if not CONTRACT_PATH.is_file():
        raise AssertionError(f"Missing M1 contract file: {CONTRACT_PATH}")
    data = load_json_strict(CONTRACT_PATH)
    if use_cache:
        _CONTRACT_CACHE = data
    return data


def load_meta_schema(use_cache: bool = True) -> dict[str, Any]:
    global _META_SCHEMA_CACHE
    if use_cache and _META_SCHEMA_CACHE is not None:
        return _META_SCHEMA_CACHE
    if not META_SCHEMA_PATH.is_file():
        raise AssertionError(f"Missing M1 meta-schema file: {META_SCHEMA_PATH}")
    data = load_json_strict(META_SCHEMA_PATH)
    if use_cache:
        _META_SCHEMA_CACHE = data
    return data


def get_meta_validator() -> Draft202012Validator:
    global _META_VALIDATOR_CACHE
    if _META_VALIDATOR_CACHE is None:
        meta = load_meta_schema()
        Draft202012Validator.check_schema(meta)
        _META_VALIDATOR_CACHE = Draft202012Validator(meta)
    return _META_VALIDATOR_CACHE


def get_validator(schema_name: str) -> Draft202012Validator:
    if schema_name not in _VALIDATOR_CACHE:
        contract = load_contract()
        schemas = contract.get("schemas", {})
        if schema_name not in schemas:
            raise KeyError(f"Unknown schema '{schema_name}' in INTERFACE_CONTRACT.json")
        schema = schemas[schema_name]
        Draft202012Validator.check_schema(schema)
        _VALIDATOR_CACHE[schema_name] = Draft202012Validator(schema)
    return _VALIDATOR_CACHE[schema_name]


def validate_against_schema(schema_name: str, instance: Any) -> None:
    get_validator(schema_name).validate(instance)


def is_contract_only_mode() -> bool:
    return os.environ.get("HKS_E2E_CONTRACT_ONLY", "0").strip() == "1"


def validate_contract_fast() -> tuple[dict[str, Any], dict[str, Any]]:
    """Validate INTERFACE_CONTRACT.json using docs/validate_contract.py Python API."""
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from docs.validate_contract import validate_contract_files

    return validate_contract_files(CONTRACT_PATH, META_SCHEMA_PATH)


def run_validate_contract_cli(
    extra_args: Sequence[str] = ("--skip-tests",),
    *,
    use_cache: bool = True,
) -> subprocess.CompletedProcess[str]:
    if not VALIDATE_CONTRACT_PY.is_file():
        raise AssertionError(f"Missing M1 validator script: {VALIDATE_CONTRACT_PY}")
    key = tuple(extra_args)
    cacheable = use_cache and key in ((), ("--skip-tests",))
    if cacheable and key in _VALIDATE_CONTRACT_CLI_CACHE:
        return _VALIDATE_CONTRACT_CLI_CACHE[key]
    cmd = [sys.executable, str(VALIDATE_CONTRACT_PY), *extra_args]
    proc = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True, check=False)
    if cacheable:
        _VALIDATE_CONTRACT_CLI_CACHE[key] = proc
    return proc


# ---------------------------------------------------------------------------
# Strict M2 / M3 / M4 Artifact & CLI Resolvers (NEVER mock/skip in full mode)
# ---------------------------------------------------------------------------

def _rel_display(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def require_m2() -> dict[str, Path]:
    """Strictly verify all M2 hardware deliverables exist; raise AssertionError if any missing."""
    required = {
        "ee_bom": HARDWARE_DIR / "ee" / "bom.csv",
        "pinmux": HARDWARE_DIR / "ee" / "pinmux_and_buses.json",
        "schematic_spec": HARDWARE_DIR / "ee" / "schematic_design.md",
        "dimensions": HARDWARE_DIR / "mechanical" / "dimensions.json",
        "mech_bom": HARDWARE_DIR / "mechanical" / "bom.csv",
        "enclosure_spec": HARDWARE_DIR / "mechanical" / "enclosure_spec.md",
        "power_budget": HARDWARE_DIR / "power_budget.py",
        "tests_dir": HARDWARE_DIR / "tests",
    }
    missing = [f"{k}: {_rel_display(p)}" for k, p in required.items() if not p.exists()]
    if required["tests_dir"].is_dir():
        test_files = list(required["tests_dir"].glob("test_*.py"))
        if not test_files:
            missing.append("tests: hardware/tests/test_*.py (no unit test files found)")
    if missing:
        raise AssertionError(
            "M2 hardware deliverables missing (strict full mode forbids mocks/skips): " + ", ".join(missing)
        )
    return required


def require_m3(*, build_if_needed: bool = True) -> dict[str, Path]:
    """Strictly verify M3 C11 firmware deliverables & compiled CLI exist; raise AssertionError if missing."""
    required_dirs = {
        "makefile": FIRMWARE_DIR / "Makefile",
        "include": FIRMWARE_DIR / "include",
        "src": FIRMWARE_DIR / "src",
        "hal": FIRMWARE_DIR / "hal",
        "tools": FIRMWARE_DIR / "tools",
        "tests": FIRMWARE_DIR / "tests",
    }
    missing = [f"{k}: {_rel_display(p)}" for k, p in required_dirs.items() if not p.exists()]
    for sub in ("include", "src", "tests"):
        d = required_dirs[sub]
        if d.is_dir() and not any(d.iterdir()):
            missing.append(f"{sub}: {_rel_display(d)} is empty")
    if missing:
        raise AssertionError(
            "M3 firmware deliverables missing (strict full mode forbids mocks/skips): " + ", ".join(missing)
        )

    cli_candidates = [
        FIRMWARE_DIR / "build" / "hks_fw_sim_cli",
        FIRMWARE_DIR / "hks_fw_sim_cli",
        FIRMWARE_DIR / "tools" / "hks_fw_sim_cli",
    ]
    cli_path = next((p for p in cli_candidates if p.is_file() and os.access(p, os.X_OK)), None)
    if cli_path is None and build_if_needed:
        proc = subprocess.run(
            ["make", "-C", str(FIRMWARE_DIR), "all"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
        cli_path = next((p for p in cli_candidates if p.is_file() and os.access(p, os.X_OK)), None)
        if cli_path is None:
            proc_test = subprocess.run(
                ["make", "-C", str(FIRMWARE_DIR), "test"],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                check=False,
            )
            cli_path = next((p for p in cli_candidates if p.is_file() and os.access(p, os.X_OK)), None)
            if cli_path is None:
                raise AssertionError(
                    "M3 firmware CLI binary hks_fw_sim_cli missing after `make -C firmware`. "
                    f"make all rc={proc.returncode} stderr={proc.stderr.strip()}; "
                    f"make test rc={proc_test.returncode} stderr={proc_test.stderr.strip()}"
                )
    if cli_path is None:
        raise AssertionError("M3 firmware CLI binary hks_fw_sim_cli missing under firmware/build/")
    required_dirs["cli"] = cli_path
    return required_dirs


def require_m4() -> dict[str, Path]:
    """Strictly verify all M4 host/simulator deliverables exist; raise AssertionError if any missing."""
    required = {
        "init": SIM_DIR / "__init__.py",
        "physics_sim": SIM_DIR / "physics_sim.py",
        "flash_codec": SIM_DIR / "flash_codec.py",
        "downloader": SIM_DIR / "downloader.py",
        "fusion_pipeline": SIM_DIR / "fusion_pipeline.py",
        "cli": SIM_DIR / "cli.py",
        "tests_dir": SIM_DIR / "tests",
    }
    missing = [f"{k}: {_rel_display(p)}" for k, p in required.items() if not p.exists()]
    if required["tests_dir"].is_dir():
        test_files = list(required["tests_dir"].glob("test_*.py"))
        if not test_files:
            missing.append("tests: sim/tests/test_*.py (no unit test files found)")
    if missing:
        raise AssertionError(
            "M4 sim deliverables missing (strict full mode forbids mocks/skips): " + ", ".join(missing)
        )
    return required


def run_power_budget_json(*, use_cache: bool = True) -> dict[str, Any]:
    global _POWER_BUDGET_CACHE
    if use_cache and _POWER_BUDGET_CACHE is not None:
        return copy.deepcopy(_POWER_BUDGET_CACHE)
    paths = require_m2()
    proc = subprocess.run(
        [sys.executable, str(paths["power_budget"]), "--json"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"hardware/power_budget.py --json failed (rc={proc.returncode}): {proc.stderr.strip()}"
        )
    data = loads_json_strict(proc.stdout)
    if use_cache:
        _POWER_BUDGET_CACHE = data
    return copy.deepcopy(data)


def run_hardware_unit_tests_cached(*, use_cache: bool = True) -> subprocess.CompletedProcess[str]:
    global _HW_TESTS_CACHE
    if use_cache and _HW_TESTS_CACHE is not None:
        return _HW_TESTS_CACHE
    m2 = require_m2()
    proc = subprocess.run(
        [sys.executable, "-m", "unittest", "discover", "-s", str(m2["tests_dir"]), "-v"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    if use_cache:
        _HW_TESTS_CACHE = proc
    return proc


def run_firmware_make_test_cached(*, use_cache: bool = True) -> subprocess.CompletedProcess[str]:
    global _FW_MAKE_TEST_CACHE
    if use_cache and _FW_MAKE_TEST_CACHE is not None:
        return _FW_MAKE_TEST_CACHE
    require_m3()
    proc = subprocess.run(
        ["make", "-C", str(FIRMWARE_DIR), "test"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    if use_cache:
        _FW_MAKE_TEST_CACHE = proc
    return proc


def run_sim_unit_tests_cached(*, use_cache: bool = True) -> subprocess.CompletedProcess[str]:
    global _SIM_TESTS_CACHE
    if use_cache and _SIM_TESTS_CACHE is not None:
        return _SIM_TESTS_CACHE
    require_m4()
    proc = subprocess.run(
        [sys.executable, "-m", "unittest", "discover", "-s", str(SIM_DIR / "tests"), "-v"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    if use_cache:
        _SIM_TESTS_CACHE = proc
    return proc


def run_fw_cli(args: Sequence[str]) -> subprocess.CompletedProcess[str]:
    paths = require_m3()
    return subprocess.run(
        [str(paths["cli"]), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )


def run_fw_sim_cached(
    *,
    usb_selftest: bool = True,
    use_cache: bool = True,
) -> tuple[bytes, dict[str, Any], subprocess.CompletedProcess[str]]:
    """Invoke hks_fw_sim_cli (--out, --info-json, [--usb-selftest]) and cache output."""
    if use_cache and usb_selftest in _FW_SIM_CACHE:
        raw, info, proc = _FW_SIM_CACHE[usb_selftest]
        return raw, copy.deepcopy(info), proc
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        out_hks = Path(tmp) / "fw_sim.hks"
        info_json = Path(tmp) / "fw_info.json"
        cli_args = ["--out", str(out_hks), "--info-json", str(info_json)]
        if usb_selftest:
            cli_args.append("--usb-selftest")
        proc = run_fw_cli(cli_args)
        if proc.returncode != 0:
            raise AssertionError(
                f"hks_fw_sim_cli failed (rc={proc.returncode}): stdout={proc.stdout} stderr={proc.stderr}"
            )
        raw = out_hks.read_bytes()
        info = load_json_strict(info_json)
    if use_cache:
        _FW_SIM_CACHE[usb_selftest] = (raw, info, proc)
    return raw, copy.deepcopy(info), proc


def run_sim_cli(
    args: Sequence[str],
    *,
    subprocess_mode: bool = False,
) -> subprocess.CompletedProcess[str]:
    """Invoke sim.cli with opaque-box CLI args (in-process sim.cli.main by default, or subprocess)."""
    require_m4()
    if subprocess_mode:
        return subprocess.run(
            [sys.executable, "-m", "sim.cli", *args],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from sim.cli import main as sim_cli_main

    out_buf = io.StringIO()
    err_buf = io.StringIO()
    with contextlib.redirect_stdout(out_buf), contextlib.redirect_stderr(err_buf):
        rc = int(sim_cli_main(list(args)))
    return subprocess.CompletedProcess(
        args=[sys.executable, "-m", "sim.cli", *args],
        returncode=rc,
        stdout=out_buf.getvalue(),
        stderr=err_buf.getvalue(),
    )


def sim_and_analyze_cached(
    *,
    preset: str = "NHL_200X85FT",
    duration_s: float = 140.0,
    shifts: int = 2,
    seed: int = 20261122,
    mount: str = "STERNUM_POCKET",
    player_height_cm: int = 183,
    use_cache: bool = True,
) -> tuple[bytes, dict[str, Any], dict[str, Any]]:
    """Run sim.cli simulate + analyze and memoize identical sessions per test process."""
    key = (preset, float(duration_s), int(shifts), int(seed), mount, int(player_height_cm))
    if use_cache and key in _SIM_ANALYZE_CACHE:
        raw, rep, truth = _SIM_ANALYZE_CACHE[key]
        return raw, copy.deepcopy(rep), copy.deepcopy(truth)

    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        hks_p = Path(tmp) / "session.hks"
        truth_p = Path(tmp) / "truth.json"
        rep_p = Path(tmp) / "report.json"
        p_sim = run_sim_cli(
            [
                "simulate",
                "--preset",
                preset,
                "--duration-s",
                str(duration_s),
                "--shifts",
                str(shifts),
                "--seed",
                str(seed),
                "--mount",
                mount,
                "--player-height-cm",
                str(player_height_cm),
                "--out-hks",
                str(hks_p),
                "--out-truth",
                str(truth_p),
            ]
        )
        if p_sim.returncode != 0:
            raise AssertionError(f"sim.cli simulate failed: rc={p_sim.returncode} stderr={p_sim.stderr}")
        p_ana = run_sim_cli(["analyze", "--in-hks", str(hks_p), "--out-report", str(rep_p)])
        if p_ana.returncode != 0:
            raise AssertionError(f"sim.cli analyze failed: rc={p_ana.returncode} stderr={p_ana.stderr}")
        raw = hks_p.read_bytes()
        rep = load_json_strict(rep_p)
        truth = load_json_strict(truth_p)
    if use_cache:
        _SIM_ANALYZE_CACHE[key] = (raw, rep, truth)
    return raw, copy.deepcopy(rep), copy.deepcopy(truth)


# ---------------------------------------------------------------------------
# Synthetic Opaque-Box Binary .hks Codec (0x01 - 0x07) & CRC32-IEEE
# ---------------------------------------------------------------------------

def crc32_ieee(data: bytes) -> int:
    """IEEE 802.3 CRC32 (reflected poly 0xEDB88320, init 0xFFFFFFFF, xorout 0xFFFFFFFF)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def adxl372_fifo_word_to_i16(raw_u16: int) -> int:
    """Simulate ADXL372 hardware 12-bit left-justified word -> firmware >> 4 signed int16 [-2048, +2047]."""
    val_u16 = raw_u16 & 0xFFFF
    signed_16 = val_u16 - 0x10000 if (val_u16 & 0x8000) else val_u16
    return signed_16 >> 4


def adxl372_i16_to_mps2(raw_i16: int) -> float:
    """Convert right-aligned firmware int16 [-2048, +2047] to m/s^2 WITHOUT double >> 4 shift."""
    if raw_i16 < -2048 or raw_i16 > 2047:
        raise ValueError(f"ADXL372 int16 sample {raw_i16} outside 12-bit range [-2048, +2047]")
    return raw_i16 * ADXL372_SCALE_MPS2_PER_LSB


def _pack_calibration_payload(pkt: dict[str, Any], *, compute_crc: bool = True) -> bytes:
    r_bs = [float(x) for row in pkt["sensor_to_body_rotation_matrix"] for x in row]
    a_m = [float(x) for row in pkt["accel_misalign_scale"] for x in row]
    a_b = [float(x) for x in pkt["accel_bias_mps2"]]
    g_m = [float(x) for row in pkt["gyro_misalign_scale"] for x in row]
    g_b = [float(x) for x in pkt["gyro_bias_rads"]]
    hg_r = [float(x) for row in pkt["high_g_to_body_rotation_matrix"] for x in row]
    hg_m = [float(x) for row in pkt["high_g_misalign_scale"] for x in row]
    hg_b = [float(x) for x in pkt["high_g_bias_mps2"]]
    mag_h = [float(x) for x in pkt["mag_hard_iron_ut"]]
    mag_s = [float(x) for row in pkt["mag_soft_iron_matrix"] for x in row]

    body_284 = struct.pack(
        "<HHQ9f9f3f9f3f9f9f3f3f9fff",
        int(pkt["cal_version"]),
        int(pkt["flags"]),
        int(pkt["cal_utc_ms"]),
        *r_bs,
        *a_m,
        *a_b,
        *g_m,
        *g_b,
        *hg_r,
        *hg_m,
        *hg_b,
        *mag_h,
        *mag_s,
        float(pkt["baseline_mag_norm_ut"]),
        float(pkt["baseline_mag_dip_deg"]),
    )
    cal_crc = crc32_ieee(body_284) if compute_crc else int(pkt["cal_crc32"])
    return body_284 + struct.pack("<I", cal_crc)


def pack_payload(msg_type: int, pkt: dict[str, Any], *, compute_cal_crc: bool = True) -> bytes:
    if msg_type == MSG_SESSION_HEADER:
        rink_id = RINK_PRESET_TO_ID[pkt["rink_preset"]] if isinstance(pkt["rink_preset"], str) else int(pkt["rink_preset"])
        mount_id = MOUNT_TO_ID[pkt["mount_location"]] if isinstance(pkt["mount_location"], str) else int(pkt["mount_location"])
        gate = pkt["bench_gate_xy_m"]
        magic = pkt["magic"].encode("ascii") if isinstance(pkt["magic"], str) else bytes(pkt["magic"])
        return struct.pack(
            SESSION_HEADER_FMT,
            magic,
            int(pkt["format_version"]),
            int(pkt["hw_rev"]),
            int(pkt["fw_version"]),
            int(pkt["cal_crc32"]),
            int(pkt["session_id"]),
            int(pkt["start_utc_ms"]),
            int(pkt["device_id"]),
            int(pkt["imu_odr_hz"]),
            int(pkt["high_g_odr_hz"]),
            int(pkt["mag_odr_hz"]),
            int(pkt["baro_odr_hz"]),
            float(pkt["accel_scale_mps2_per_lsb"]),
            float(pkt["gyro_scale_rads_per_lsb"]),
            float(pkt["high_g_scale_mps2_per_lsb"]),
            float(pkt["mag_scale_ut_per_lsb"]),
            rink_id,
            mount_id,
            int(pkt["player_height_cm"]),
            float(gate[0]),
            float(gate[1]),
            int(pkt.get("reserved", 0)),
        )
    if msg_type == MSG_IMU_BATCH:
        samples = pkt["samples"]
        hdr = struct.pack(
            IMU_BATCH_HDR_FMT,
            len(samples),
            int(pkt["odr_hz"]),
            int(pkt["temp_c_x100"]),
            int(pkt.get("reserved", 0)),
        )
        body = b"".join(
            struct.pack(
                IMU_SAMPLE_FMT,
                int(s["ax_i16"]),
                int(s["ay_i16"]),
                int(s["az_i16"]),
                int(s["gx_i16"]),
                int(s["gy_i16"]),
                int(s["gz_i16"]),
            )
            for s in samples
        )
        return hdr + body
    if msg_type == MSG_HIGH_G_BATCH:
        samples = pkt["samples"]
        trig = (
            TRIGGER_MODE_TO_ID[pkt["trigger_mode"]]
            if isinstance(pkt["trigger_mode"], str)
            else int(pkt["trigger_mode"])
        )
        hdr = struct.pack(
            HIGH_G_BATCH_HDR_FMT,
            len(samples),
            int(pkt["odr_hz"]),
            trig,
            int(pkt.get("reserved0", 0)),
            int(pkt.get("reserved1", 0)),
        )
        body = b"".join(
            struct.pack(
                HIGH_G_SAMPLE_FMT,
                int(s["ax_i16"]),
                int(s["ay_i16"]),
                int(s["az_i16"]),
            )
            for s in samples
        )
        return hdr + body
    if msg_type == MSG_AUX_ENV:
        return struct.pack(
            AUX_ENV_FMT,
            int(pkt["mx_i32"]),
            int(pkt["my_i32"]),
            int(pkt["mz_i32"]),
            int(pkt["pressure_pa_x64"]),
            int(pkt["baro_temp_c_x100"]),
            int(pkt["vbatt_mv"]),
            int(pkt["batt_temp_c_x100"]),
            int(pkt["soc_pct_x10"]),
            int(pkt["pmic_status"]),
            int(pkt["validity_flags"]),
        )
    if msg_type == MSG_EVENT_MARKER:
        ev_id = EVENT_CODE_TO_ID[pkt["event_code"]] if isinstance(pkt["event_code"], str) else int(pkt["event_code"])
        return struct.pack(
            EVENT_MARKER_FMT,
            ev_id,
            int(pkt["severity"]),
            float(pkt["value_f32"]),
            int(pkt["detail_u64"]),
        )
    if msg_type == MSG_CALIBRATION_BLOCK:
        return _pack_calibration_payload(pkt, compute_crc=compute_cal_crc)
    if msg_type == MSG_SESSION_FOOTER:
        reason_id = END_REASON_TO_ID[pkt["end_reason"]] if isinstance(pkt["end_reason"], str) else int(pkt["end_reason"])
        sha_raw = (
            bytes.fromhex(pkt["pre_footer_sha256"])
            if isinstance(pkt["pre_footer_sha256"], str)
            else bytes(pkt["pre_footer_sha256"])
        )
        if len(sha_raw) != 32:
            raise ValueError(f"pre_footer_sha256 must be 32 bytes, got {len(sha_raw)}")
        return struct.pack(
            SESSION_FOOTER_FMT,
            int(pkt["end_utc_ms"]),
            int(pkt["total_imu_samples"]),
            int(pkt["dropped_imu_samples"]),
            int(pkt["total_high_g_samples"]),
            int(pkt["dropped_high_g_samples"]),
            int(pkt["total_aux_samples"]),
            reason_id,
            int(pkt["total_bytes"]),
            sha_raw,
        )
    raise ValueError(f"Unknown msg_type: {msg_type:#x}")


def pack_frame(
    msg_type: int,
    payload: bytes | dict[str, Any],
    *,
    flags: int = 0,
    seq: int = 0,
    timestamp_us: int = 0,
    reserved: int = 0,
) -> bytes:
    payload_bytes = pack_payload(msg_type, payload) if isinstance(payload, dict) else bytes(payload)
    if len(payload_bytes) > MAX_PAYLOAD_BYTES:
        raise ValueError(f"Payload length {len(payload_bytes)} exceeds max {MAX_PAYLOAD_BYTES}")
    hdr = struct.pack(
        FRAME_HDR_FMT,
        SYNC_U16,
        msg_type & 0xFF,
        flags & 0xFF,
        len(payload_bytes),
        seq & 0xFFFF,
        int(timestamp_us),
        reserved & 0xFFFFFFFF,
    )
    frame_no_crc = hdr + payload_bytes
    crc = crc32_ieee(frame_no_crc)
    return frame_no_crc + struct.pack("<I", crc)


def _reshape3x3(vals: Sequence[float]) -> list[list[float]]:
    return [
        [float(vals[0]), float(vals[1]), float(vals[2])],
        [float(vals[3]), float(vals[4]), float(vals[5])],
        [float(vals[6]), float(vals[7]), float(vals[8])],
    ]


def unpack_payload(
    msg_type: int,
    payload: bytes,
    header_meta: dict[str, Any] | None = None,
) -> dict[str, Any]:
    base = dict(header_meta) if header_meta else {}
    if msg_type == MSG_SESSION_HEADER:
        if len(payload) != 80:
            raise ValueError(f"SESSION_HEADER payload must be 80 B, got {len(payload)}")
        u = struct.unpack(SESSION_HEADER_FMT, payload)
        return {
            **base,
            "magic": u[0].decode("ascii", errors="strict"),
            "format_version": u[1],
            "hw_rev": u[2],
            "fw_version": u[3],
            "cal_crc32": u[4],
            "session_id": u[5],
            "start_utc_ms": u[6],
            "device_id": u[7],
            "imu_odr_hz": u[8],
            "high_g_odr_hz": u[9],
            "mag_odr_hz": u[10],
            "baro_odr_hz": u[11],
            "accel_scale_mps2_per_lsb": u[12],
            "gyro_scale_rads_per_lsb": u[13],
            "high_g_scale_mps2_per_lsb": u[14],
            "mag_scale_ut_per_lsb": u[15],
            "rink_preset": ID_TO_RINK_PRESET[u[16]],
            "mount_location": ID_TO_MOUNT[u[17]],
            "player_height_cm": u[18],
            "bench_gate_xy_m": [round(float(u[19]), 6), round(float(u[20]), 6)],
        }
    if msg_type == MSG_IMU_BATCH:
        if len(payload) < 8:
            raise ValueError(f"IMU_BATCH payload too short: {len(payload)}")
        sample_count, odr_hz, temp_c_x100, _rsv = struct.unpack(IMU_BATCH_HDR_FMT, payload[:8])
        expected_len = 8 + 12 * sample_count
        if len(payload) != expected_len:
            raise ValueError(f"IMU_BATCH length {len(payload)} != {expected_len}")
        samples = []
        for idx in range(sample_count):
            off = 8 + 12 * idx
            ax, ay, az, gx, gy, gz = struct.unpack(IMU_SAMPLE_FMT, payload[off : off + 12])
            samples.append(
                {
                    "ax_i16": ax,
                    "ay_i16": ay,
                    "az_i16": az,
                    "gx_i16": gx,
                    "gy_i16": gy,
                    "gz_i16": gz,
                }
            )
        return {
            **base,
            "sample_count": sample_count,
            "odr_hz": odr_hz,
            "temp_c_x100": temp_c_x100,
            "samples": samples,
        }
    if msg_type == MSG_HIGH_G_BATCH:
        if len(payload) < 8:
            raise ValueError(f"HIGH_G_BATCH payload too short: {len(payload)}")
        sample_count, odr_hz, trig_id, _r0, _r1 = struct.unpack(HIGH_G_BATCH_HDR_FMT, payload[:8])
        expected_len = 8 + 6 * sample_count
        if len(payload) != expected_len:
            raise ValueError(f"HIGH_G_BATCH length {len(payload)} != {expected_len}")
        samples = []
        for idx in range(sample_count):
            off = 8 + 6 * idx
            ax, ay, az = struct.unpack(HIGH_G_SAMPLE_FMT, payload[off : off + 6])
            samples.append({"ax_i16": ax, "ay_i16": ay, "az_i16": az})
        return {
            **base,
            "sample_count": sample_count,
            "odr_hz": odr_hz,
            "trigger_mode": ID_TO_TRIGGER_MODE[trig_id],
            "samples": samples,
        }
    if msg_type == MSG_AUX_ENV:
        if len(payload) != 28:
            raise ValueError(f"AUX_ENV payload must be 28 B, got {len(payload)}")
        u = struct.unpack(AUX_ENV_FMT, payload)
        return {
            **base,
            "mx_i32": u[0],
            "my_i32": u[1],
            "mz_i32": u[2],
            "pressure_pa_x64": u[3],
            "baro_temp_c_x100": u[4],
            "vbatt_mv": u[5],
            "batt_temp_c_x100": u[6],
            "soc_pct_x10": u[7],
            "pmic_status": u[8],
            "validity_flags": u[9],
        }
    if msg_type == MSG_EVENT_MARKER:
        if len(payload) != 16:
            raise ValueError(f"EVENT_MARKER payload must be 16 B, got {len(payload)}")
        ev_id, sev, val_f32, detail_u64 = struct.unpack(EVENT_MARKER_FMT, payload)
        return {
            **base,
            "event_code": ID_TO_EVENT_CODE[ev_id],
            "severity": sev,
            "value_f32": val_f32,
            "detail_u64": detail_u64,
        }
    if msg_type == MSG_CALIBRATION_BLOCK:
        if len(payload) != 288:
            raise ValueError(f"CALIBRATION_BLOCK payload must be 288 B, got {len(payload)}")
        u = struct.unpack(CALIBRATION_BLOCK_FMT, payload)
        expected_cal_crc = crc32_ieee(payload[:284])
        cal_crc = u[71]
        if cal_crc != expected_cal_crc:
            raise ValueError(f"CALIBRATION_BLOCK cal_crc32 mismatch: {cal_crc:#010x} != {expected_cal_crc:#010x}")
        return {
            "cal_version": u[0],
            "flags": u[1],
            "cal_utc_ms": u[2],
            "sensor_to_body_rotation_matrix": _reshape3x3(u[3:12]),
            "accel_misalign_scale": _reshape3x3(u[12:21]),
            "accel_bias_mps2": list(u[21:24]),
            "gyro_misalign_scale": _reshape3x3(u[24:33]),
            "gyro_bias_rads": list(u[33:36]),
            "high_g_to_body_rotation_matrix": _reshape3x3(u[36:45]),
            "high_g_misalign_scale": _reshape3x3(u[45:54]),
            "high_g_bias_mps2": list(u[54:57]),
            "mag_hard_iron_ut": list(u[57:60]),
            "mag_soft_iron_matrix": _reshape3x3(u[60:69]),
            "baseline_mag_norm_ut": u[69],
            "baseline_mag_dip_deg": u[70],
            "cal_crc32": u[71],
        }
    if msg_type == MSG_SESSION_FOOTER:
        if len(payload) != 72:
            raise ValueError(f"SESSION_FOOTER payload must be 72 B, got {len(payload)}")
        u = struct.unpack(SESSION_FOOTER_FMT, payload)
        return {
            **base,
            "end_utc_ms": u[0],
            "total_imu_samples": u[1],
            "dropped_imu_samples": u[2],
            "total_high_g_samples": u[3],
            "dropped_high_g_samples": u[4],
            "total_aux_samples": u[5],
            "end_reason": ID_TO_END_REASON[u[6]],
            "total_bytes": u[7],
            "pre_footer_sha256": u[8].hex(),
        }
    raise ValueError(f"Unknown msg_type: {msg_type:#x}")


def unpack_frames(
    stream: bytes,
    *,
    verify_crc: bool = True,
    verify_footer_sha256: bool = True,
) -> list[dict[str, Any]]:
    """Decode a little-endian .hks binary frame stream with strict CRC32 & SHA-256 verification."""
    if len(stream) == 0:
        raise ValueError("Empty .hks stream (0 bytes)")

    frames: list[dict[str, Any]] = []
    pos = 0
    total_len = len(stream)

    while pos < total_len:
        if pos + FRAME_HEADER_BYTES + FRAME_CRC_BYTES > total_len:
            raise ValueError(f"Truncated .hks frame header at byte offset {pos}/{total_len}")

        sync, msg_type, flags, payload_len, seq, timestamp_us, reserved = struct.unpack(
            FRAME_HDR_FMT, stream[pos : pos + FRAME_HEADER_BYTES]
        )
        if sync != SYNC_U16:
            raise ValueError(f"Invalid .hks frame sync {sync:#06x} at offset {pos} (expected 0xAA55)")
        if payload_len > MAX_PAYLOAD_BYTES:
            raise ValueError(f"Frame payload_len {payload_len} > max {MAX_PAYLOAD_BYTES} at offset {pos}")

        frame_end = pos + FRAME_HEADER_BYTES + payload_len + FRAME_CRC_BYTES
        if frame_end > total_len:
            raise ValueError(
                f"Truncated .hks frame body at offset {pos}: need {frame_end} bytes, have {total_len}"
            )

        payload_start = pos + FRAME_HEADER_BYTES
        payload_end = payload_start + payload_len
        payload = stream[payload_start:payload_end]
        (crc_actual,) = struct.unpack("<I", stream[payload_end:frame_end])

        if verify_crc:
            crc_expected = crc32_ieee(stream[pos:payload_end])
            if crc_actual != crc_expected:
                raise ValueError(
                    f"Frame CRC32 mismatch at offset {pos} (msg_type={msg_type:#04x}): "
                    f"got {crc_actual:#010x}, expected {crc_expected:#010x}"
                )

        hdr_meta = {
            "sync": sync,
            "msg_type": msg_type,
            "flags": flags,
            "seq": seq,
            "timestamp_us": timestamp_us,
            "crc32": crc_actual,
        }
        decoded = unpack_payload(
            msg_type,
            payload,
            header_meta=None if msg_type == MSG_CALIBRATION_BLOCK else hdr_meta,
        )

        if msg_type == MSG_SESSION_FOOTER and verify_footer_sha256:
            expected_sha = hashlib.sha256(stream[:pos]).hexdigest()
            if decoded["pre_footer_sha256"].lower() != expected_sha.lower():
                raise ValueError(
                    f"SESSION_FOOTER pre_footer_sha256 mismatch: "
                    f"got {decoded['pre_footer_sha256']}, expected {expected_sha}"
                )
            if decoded["total_bytes"] != total_len:
                raise ValueError(
                    f"SESSION_FOOTER total_bytes mismatch: "
                    f"got {decoded['total_bytes']}, expected {total_len}"
                )

        frames.append(
            {
                "offset": pos,
                "frame_bytes": frame_end - pos,
                "header": {
                    "sync": sync,
                    "msg_type": msg_type,
                    "flags": flags,
                    "payload_len": payload_len,
                    "seq": seq,
                    "timestamp_us": timestamp_us,
                    "reserved": reserved,
                    "crc32": crc_actual,
                },
                "packet": decoded,
                "raw_frame": stream[pos:frame_end],
            }
        )
        pos = frame_end

    return frames


def build_synthetic_session_bytes(
    *,
    session_id: int = 1001,
    rink_preset: str = "NHL_200X85FT",
    start_timestamp_us: int = 0,
    end_timestamp_us: int = 7_200_050_000,
    seq_start: int = 0,
    include_high_g: bool = True,
) -> bytes:
    """Build a contract-compliant synthetic .hks binary stream for self-validator tests ONLY."""
    contract = load_contract()
    examples = contract["examples"]

    cal_pkt = json.loads(json.dumps(examples["calibration_block"]))
    cal_payload = pack_payload(MSG_CALIBRATION_BLOCK, cal_pkt, compute_cal_crc=True)
    cal_crc32 = struct.unpack("<I", cal_payload[-4:])[0]

    hdr_pkt = json.loads(json.dumps(examples["flash_session_header"]))
    hdr_pkt["session_id"] = session_id
    hdr_pkt["cal_crc32"] = cal_crc32
    hdr_pkt["rink_preset"] = rink_preset
    hdr_pkt["bench_gate_xy_m"] = contract["rink_geometry_presets"][rink_preset]["default_bench_gate_xy_m"]

    imu_pkt = json.loads(json.dumps(examples["imu_batch_packet"]))
    hg_pkt = json.loads(json.dumps(examples["high_g_batch_packet"]))
    aux_pkt = json.loads(json.dumps(examples["aux_env_packet"]))
    ev_pkt = json.loads(json.dumps(examples["event_marker_packet"]))

    seq = seq_start & 0xFFFF
    chunks: list[bytes] = []

    chunks.append(pack_frame(MSG_SESSION_HEADER, hdr_pkt, flags=0x20, seq=seq, timestamp_us=start_timestamp_us))
    seq = (seq + 1) & 0xFFFF
    chunks.append(pack_frame(MSG_CALIBRATION_BLOCK, cal_payload, flags=0x20, seq=seq, timestamp_us=start_timestamp_us))
    seq = (seq + 1) & 0xFFFF
    chunks.append(pack_frame(MSG_IMU_BATCH, imu_pkt, flags=0x00, seq=seq, timestamp_us=end_timestamp_us - 40_000))
    seq = (seq + 1) & 0xFFFF

    hg_count = 0
    if include_high_g:
        chunks.append(pack_frame(MSG_HIGH_G_BATCH, hg_pkt, flags=0x10, seq=seq, timestamp_us=end_timestamp_us - 30_000))
        seq = (seq + 1) & 0xFFFF
        hg_count = len(hg_pkt["samples"])

    chunks.append(pack_frame(MSG_AUX_ENV, aux_pkt, flags=0x00, seq=seq, timestamp_us=end_timestamp_us - 20_000))
    seq = (seq + 1) & 0xFFFF
    chunks.append(pack_frame(MSG_EVENT_MARKER, ev_pkt, flags=0x10, seq=seq, timestamp_us=end_timestamp_us - 10_000))
    seq = (seq + 1) & 0xFFFF

    pre_footer = b"".join(chunks)
    pre_footer_sha = hashlib.sha256(pre_footer).hexdigest()

    footer_pkt = {
        "end_utc_ms": hdr_pkt["start_utc_ms"] + (end_timestamp_us - start_timestamp_us) // 1000,
        "total_imu_samples": len(imu_pkt["samples"]),
        "dropped_imu_samples": 0,
        "total_high_g_samples": hg_count,
        "dropped_high_g_samples": 0,
        "total_aux_samples": 1,
        "end_reason": "BUTTON_STOP",
        "total_bytes": len(pre_footer) + 96,
        "pre_footer_sha256": pre_footer_sha,
    }
    footer_frame = pack_frame(MSG_SESSION_FOOTER, footer_pkt, flags=0x00, seq=seq, timestamp_us=end_timestamp_us)
    return pre_footer + footer_frame


# ---------------------------------------------------------------------------
# Synthetic USB CDC-ACM Binary Protocol Codec & Reference Device (Self-Tests Only)
# ---------------------------------------------------------------------------

DL_CMD_HDR_FMT = "<HBBH"  # 6 B: sync(0xAA55), cmd_id, seq, payload_len
DL_RSP_HDR_FMT = "<HBBBBH"  # 8 B: sync(0x55AA), cmd_id, status, seq, reserved, payload_len


def pack_download_command(cmd_id: int, payload: bytes = b"", *, seq: int = 0) -> bytes:
    hdr = struct.pack(DL_CMD_HDR_FMT, SYNC_U16, cmd_id & 0xFF, seq & 0xFF, len(payload))
    body = hdr + payload
    return body + struct.pack("<I", crc32_ieee(body))


def unpack_download_command(raw: bytes) -> dict[str, Any]:
    if len(raw) < 10:
        raise ValueError(f"Download command too short: {len(raw)} B")
    sync, cmd_id, seq, plen = struct.unpack(DL_CMD_HDR_FMT, raw[:6])
    if sync != SYNC_U16:
        raise ValueError(f"Invalid request sync {sync:#06x} != 0xAA55")
    if len(raw) != 10 + plen:
        raise ValueError(f"Command length {len(raw)} != {10 + plen}")
    payload = raw[6 : 6 + plen]
    (crc_actual,) = struct.unpack("<I", raw[6 + plen :])
    crc_expected = crc32_ieee(raw[: 6 + plen])
    return {
        "sync": sync,
        "cmd_id": cmd_id,
        "seq": seq,
        "payload": payload,
        "crc32": crc_actual,
        "crc_valid": crc_actual == crc_expected,
    }


def pack_download_response(cmd_id: int, status: int, payload: bytes = b"", *, seq: int = 0) -> bytes:
    hdr = struct.pack(DL_RSP_HDR_FMT, RSP_SYNC_U16, cmd_id & 0xFF, status & 0xFF, seq & 0xFF, 0, len(payload))
    body = hdr + payload
    return body + struct.pack("<I", crc32_ieee(body))


def unpack_download_response(raw: bytes) -> dict[str, Any]:
    if len(raw) < 12:
        raise ValueError(f"Download response too short: {len(raw)} B")
    sync, cmd_id, status, seq, reserved, plen = struct.unpack(DL_RSP_HDR_FMT, raw[:8])
    if sync != RSP_SYNC_U16:
        raise ValueError(f"Invalid response sync {sync:#06x} != 0x55AA")
    if len(raw) != 12 + plen:
        raise ValueError(f"Response length {len(raw)} != {12 + plen}")
    payload = raw[8 : 8 + plen]
    (crc_actual,) = struct.unpack("<I", raw[8 + plen :])
    crc_expected = crc32_ieee(raw[: 8 + plen])
    if crc_actual != crc_expected:
        raise ValueError(f"Response CRC32 mismatch: got {crc_actual:#010x}, expected {crc_expected:#010x}")
    return {
        "sync": sync,
        "cmd_id": cmd_id,
        "command": ID_TO_CMD_NAME.get(cmd_id, f"UNKNOWN_{cmd_id:#04x}"),
        "status_code": status,
        "status": CODE_TO_STATUS_NAME.get(status, f"UNKNOWN_{status:#04x}"),
        "seq": seq,
        "reserved": reserved,
        "payload": payload,
        "crc32": crc_actual,
    }


class ReferenceDownloadDevice:
    """In-memory USB CDC-ACM protocol oracle for test_self_validators.py ONLY."""

    def __init__(self, sessions: dict[int, bytes] | None = None, *, recording: bool = False) -> None:
        self.sessions: dict[int, bytes] = dict(sessions) if sessions else {}
        self.verified_sessions: set[int] = set()
        self.recording = recording
        self.utc_ms = 1763852500000
        self.fw_version = 0x01000000
        self.hw_rev = 0x0100
        self.vbatt_mv = 3920
        self.soc_pct_x10 = 850
        self.pmic_status = 0

    def handle_command_frame(self, raw_cmd: bytes) -> bytes:
        try:
            cmd = unpack_download_command(raw_cmd)
        except ValueError:
            return pack_download_response(0, STATUS_ERR_INVALID_PARAM, b"", seq=0)

        cmd_id = cmd["cmd_id"]
        seq = cmd["seq"]
        payload = cmd["payload"]

        if not cmd["crc_valid"]:
            return pack_download_response(cmd_id, STATUS_ERR_BAD_CRC, b"", seq=seq)

        if self.recording and cmd_id in (CMD_GET_HEADER, CMD_READ_CHUNK, CMD_DELETE_SESSION):
            return pack_download_response(cmd_id, STATUS_ERR_BUSY_RECORDING, b"", seq=seq)

        if cmd_id == CMD_LIST_SESSIONS:
            if len(payload) != 0:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            entries: list[bytes] = []
            for sid in sorted(self.sessions):
                blob = self.sessions[sid]
                frames = unpack_frames(blob, verify_crc=True, verify_footer_sha256=False)
                start_ms = frames[0]["packet"]["start_utc_ms"] if frames else 0
                dur_ms = int((frames[-1]["header"]["timestamp_us"] - frames[0]["header"]["timestamp_us"]) // 1000) if frames else 0
                sample_cnt = sum(
                    f["packet"].get("sample_count", 0)
                    for f in frames
                    if f["header"]["msg_type"] in (MSG_IMU_BATCH, MSG_HIGH_G_BATCH)
                )
                entries.append(struct.pack("<QQIIII", sid, start_ms, dur_ms, len(blob), sample_cnt, 0))
            rsp_payload = struct.pack("<HH", len(entries), 0) + b"".join(entries)
            return pack_download_response(cmd_id, STATUS_OK, rsp_payload, seq=seq)

        if cmd_id == CMD_GET_HEADER:
            if len(payload) != 8:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            (sid,) = struct.unpack("<Q", payload)
            if sid not in self.sessions:
                return pack_download_response(cmd_id, STATUS_ERR_SESSION_NOT_FOUND, b"", seq=seq)
            blob = self.sessions[sid]
            if len(blob) < 416:
                return pack_download_response(cmd_id, STATUS_ERR_FLASH_IO, b"", seq=seq)
            return pack_download_response(cmd_id, STATUS_OK, blob[:416], seq=seq)

        if cmd_id == CMD_READ_CHUNK:
            if len(payload) != 14:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            sid, offset, length = struct.unpack("<QIH", payload)
            if length == 0 or length > MAX_CHUNK_BYTES:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            if sid not in self.sessions:
                return pack_download_response(cmd_id, STATUS_ERR_SESSION_NOT_FOUND, b"", seq=seq)
            blob = self.sessions[sid]
            if offset > len(blob):
                return pack_download_response(cmd_id, STATUS_ERR_OFFSET_OUT_OF_RANGE, b"", seq=seq)
            chunk = blob[offset : offset + length]
            rsp_payload = struct.pack("<IH", offset, len(chunk)) + chunk
            return pack_download_response(cmd_id, STATUS_OK, rsp_payload, seq=seq)

        if cmd_id == CMD_ACK_VERIFY:
            if len(payload) not in (12, 44):
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            sid, comp_crc = struct.unpack("<QI", payload[:12])
            if sid not in self.sessions:
                return pack_download_response(cmd_id, STATUS_ERR_SESSION_NOT_FOUND, b"", seq=seq)
            blob = self.sessions[sid]
            actual_crc = crc32_ieee(blob)
            hash_mode = 2 if len(payload) == 44 else 1
            ok = comp_crc == actual_crc
            if ok and hash_mode == 2:
                comp_sha = payload[12:44]
                ok = comp_sha == hashlib.sha256(blob).digest()
            if not ok:
                self.verified_sessions.discard(sid)
                return pack_download_response(
                    cmd_id, STATUS_ERR_VERIFY_MISMATCH, struct.pack("<BB", 0, hash_mode), seq=seq
                )
            self.verified_sessions.add(sid)
            return pack_download_response(cmd_id, STATUS_OK, struct.pack("<BB", 1, hash_mode), seq=seq)

        if cmd_id == CMD_DELETE_SESSION:
            if len(payload) != 9:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            sid, force_flag = struct.unpack("<QB", payload)
            if force_flag not in (0, 1):
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            if sid not in self.sessions:
                return pack_download_response(cmd_id, STATUS_ERR_SESSION_NOT_FOUND, b"", seq=seq)
            if force_flag == 0 and sid not in self.verified_sessions:
                return pack_download_response(cmd_id, STATUS_ERR_NOT_VERIFIED, b"", seq=seq)
            freed = len(self.sessions.pop(sid))
            self.verified_sessions.discard(sid)
            return pack_download_response(cmd_id, STATUS_OK, struct.pack("<I", freed), seq=seq)

        if cmd_id == CMD_GET_DEVICE_STATUS:
            if len(payload) != 0:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            used_kb = sum(len(b) for b in self.sessions.values()) // 1024
            free_kb = max(0, (FLASH_USABLE_RING_BYTES // 1024) - used_kb)
            rsp_payload = struct.pack(
                "<IHHHBBIQ",
                self.fw_version,
                self.hw_rev,
                self.vbatt_mv,
                self.soc_pct_x10,
                1 if self.recording else 0,
                self.pmic_status & 0xFF,
                free_kb,
                self.utc_ms,
            )
            return pack_download_response(cmd_id, STATUS_OK, rsp_payload, seq=seq)

        if cmd_id == CMD_SET_UTC_TIME:
            if len(payload) != 8:
                return pack_download_response(cmd_id, STATUS_ERR_INVALID_PARAM, b"", seq=seq)
            (new_utc,) = struct.unpack("<Q", payload)
            self.utc_ms = new_utc
            return pack_download_response(cmd_id, STATUS_OK, struct.pack("<Q", self.utc_ms), seq=seq)

        return pack_download_response(cmd_id, STATUS_ERR_UNKNOWN_CMD, b"", seq=seq)


# ---------------------------------------------------------------------------
# Math, Coordinate Frame, Rink Geometry & Metric Evaluators
# ---------------------------------------------------------------------------

def mat3_det(m: Sequence[Sequence[float]]) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def mat3_ortho_error(m: Sequence[Sequence[float]]) -> float:
    max_err = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(m[k][i] * m[k][j] for k in range(3))
            target = 1.0 if i == j else 0.0
            max_err = max(max_err, abs(dot - target))
    return max_err


def is_so3(m: Any, tol: float = 1e-5) -> tuple[bool, float, float]:
    if not isinstance(m, (list, tuple)) or len(m) != 3:
        return False, float("inf"), float("nan")
    if any(not isinstance(r, (list, tuple)) or len(r) != 3 for r in m):
        return False, float("inf"), float("nan")
    ortho_err = mat3_ortho_error(m)
    det = mat3_det(m)
    ok = ortho_err <= tol and abs(det - 1.0) <= tol
    return ok, ortho_err, det


def quat_norm(q: Sequence[float]) -> float:
    if len(q) != 4:
        raise ValueError(f"Quaternion must have 4 components, got {len(q)}")
    return math.sqrt(sum(float(c) ** 2 for c in q))


def is_unit_quaternion(q: Sequence[float], tol: float = 1e-6) -> bool:
    if len(q) != 4:
        return False
    return abs(quat_norm(q) - 1.0) <= tol


def quat_to_rotmat(q: Sequence[float]) -> list[list[float]]:
    """Convert Hamilton scalar-first unit quaternion [qw, qx, qy, qz] to active 3x3 rotation matrix."""
    qw, qx, qy, qz = (float(c) for c in q)
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n == 0.0:
        raise ValueError("Zero-norm quaternion")
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return [
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ]


def mat_vec_mul(m: Sequence[Sequence[float]], v: Sequence[float]) -> list[float]:
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def rotate_vec_by_quat(q: Sequence[float], v: Sequence[float]) -> list[float]:
    return mat_vec_mul(quat_to_rotmat(q), v)


def _get_preset_dict(preset: str | dict[str, Any]) -> dict[str, Any]:
    if isinstance(preset, str):
        contract = load_contract()
        return contract["rink_geometry_presets"][preset]
    return preset


def rink_wall_signed_distance(x: float, y: float, preset: str | dict[str, Any]) -> float:
    """Signed distance to rounded-rectangle rink boundary (<= 0 inside, > 0 outside)."""
    xf = float(x)
    yf = float(y)
    if not (math.isfinite(xf) and math.isfinite(yf)):
        raise ValueError(f"Non-finite rink coordinates: ({x!r}, {y!r})")
    p = _get_preset_dict(preset)
    half_l = p["length_m"] / 2.0
    half_w = p["width_m"] / 2.0
    rc = p["corner_radius_m"]
    core_x = half_l - rc
    core_y = half_w - rc
    qx = abs(xf) - core_x
    qy = abs(yf) - core_y
    outside_dist = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside_dist = min(max(qx, qy), 0.0)
    return outside_dist + inside_dist - rc


def is_inside_rink(
    x: float,
    y: float,
    preset: str | dict[str, Any],
    *,
    tol: float = 1e-6,
    torso_margin_m: float = 0.0,
) -> bool:
    xf = float(x)
    yf = float(y)
    tolf = float(tol)
    marginf = float(torso_margin_m)
    if not (math.isfinite(xf) and math.isfinite(yf) and math.isfinite(tolf) and math.isfinite(marginf)):
        return False
    p = _get_preset_dict(preset)
    half_l = p["length_m"] / 2.0 - marginf
    half_w = p["width_m"] / 2.0 - marginf
    rc = p["corner_radius_m"] - marginf
    if rc < 0.0 or half_l < 0.0 or half_w < 0.0:
        return False
    ax = abs(xf)
    ay = abs(yf)
    if ax > half_l + tolf or ay > half_w + tolf:
        return False
    core_x = half_l - rc
    core_y = half_w - rc
    if ax > core_x and ay > core_y:
        if math.hypot(ax - core_x, ay - core_y) > rc + tolf:
            return False
    return True


def compute_speed_rmse(est_speeds: Sequence[float], true_speeds: Sequence[float]) -> float:
    if len(est_speeds) != len(true_speeds) or len(est_speeds) == 0:
        raise ValueError("est_speeds and true_speeds must be non-empty and equal length")
    sq_err = 0.0
    for e, t in zip(est_speeds, true_speeds):
        ef = float(e)
        tf = float(t)
        if not (math.isfinite(ef) and math.isfinite(tf)):
            raise ValueError(f"Non-finite speed value in compute_speed_rmse: est={e!r}, true={t!r}")
        sq_err += (ef - tf) ** 2
    return math.sqrt(sq_err / len(est_speeds))


def compute_cep50(
    est_xy: Sequence[tuple[float, float]],
    true_xy: Sequence[tuple[float, float]],
) -> float:
    if len(est_xy) != len(true_xy) or len(est_xy) == 0:
        raise ValueError("est_xy and true_xy must be non-empty and equal length")
    dists: list[float] = []
    for (ex, ey), (tx, ty) in zip(est_xy, true_xy):
        exf, eyf, txf, tyf = float(ex), float(ey), float(tx), float(ty)
        if not (math.isfinite(exf) and math.isfinite(eyf) and math.isfinite(txf) and math.isfinite(tyf)):
            raise ValueError(
                f"Non-finite coordinate in compute_cep50: est=({ex!r}, {ey!r}), true=({tx!r}, {ty!r})"
            )
        dists.append(math.hypot(exf - txf, eyf - tyf))
    errs = sorted(dists)
    n = len(errs)
    mid = n // 2
    if n % 2 == 1:
        return errs[mid]
    return 0.5 * (errs[mid - 1] + errs[mid])


def compute_shift_f1_and_boundary_error(
    est_shifts: Sequence[tuple[float, float]],
    true_shifts: Sequence[tuple[float, float]],
    *,
    iou_thresh: float = 0.5,
) -> tuple[float, float]:
    """Match estimated (start_s, end_s) intervals to truth by max IoU; return (F1, max_boundary_err_s)."""
    if not est_shifts and not true_shifts:
        return 1.0, 0.0
    if not est_shifts or not true_shifts:
        return 0.0, float("inf")

    used_est: set[int] = set()
    tp = 0
    max_bnd_err = 0.0
    for ts, te in true_shifts:
        best_idx = -1
        best_iou = 0.0
        for idx, (es, ee) in enumerate(est_shifts):
            if idx in used_est:
                continue
            inter = max(0.0, min(te, ee) - max(ts, es))
            union = max(te, ee) - min(ts, es)
            iou = inter / union if union > 0.0 else 0.0
            if iou > best_iou:
                best_iou = iou
                best_idx = idx
        if best_idx >= 0 and best_iou >= iou_thresh:
            used_est.add(best_idx)
            tp += 1
            es, ee = est_shifts[best_idx]
            max_bnd_err = max(max_bnd_err, abs(es - ts), abs(ee - te))

    precision = tp / len(est_shifts)
    recall = tp / len(true_shifts)
    f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) > 0.0 else 0.0
    if tp == 0:
        max_bnd_err = float("inf")
    return f1, max_bnd_err


def compute_distance_pct_error(est_dist_m: float, true_dist_m: float) -> float:
    est_f = float(est_dist_m)
    true_f = float(true_dist_m)
    if not (math.isfinite(est_f) and math.isfinite(true_f)):
        raise ValueError(f"Non-finite distance in compute_distance_pct_error: est={est_dist_m!r}, true={true_dist_m!r}")
    if true_f <= 0.0:
        raise ValueError(f"true_dist_m must be > 0, got {true_dist_m}")
    return abs(est_f - true_f) / true_f * 100.0


def align_trajectory_by_time(
    est_traj: Sequence[dict[str, Any]],
    truth_traj: Sequence[dict[str, Any]],
    *,
    min_points: int = 2,
    duration_tol_frac: float = 0.05,
    min_duration_tol_s: float = 2.0,
) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    """Timestamp-align est_traj against truth_traj via linear interpolation and verify duration coverage."""
    if len(est_traj) < min_points or len(est_traj) <= 1:
        raise ValueError(f"Estimated trajectory too short ({len(est_traj)} points; require > 1)")
    if len(truth_traj) < 2:
        raise ValueError(f"Truth trajectory too short ({len(truth_traj)} points; require >= 2)")

    t_truth: list[float] = []
    truth_clean: list[dict[str, float]] = []
    for idx, pt in enumerate(truth_traj):
        t_val = float(pt["t"])
        x_val = float(pt["x"])
        y_val = float(pt["y"])
        s_val = float(pt["speed"])
        if not (math.isfinite(t_val) and math.isfinite(x_val) and math.isfinite(y_val) and math.isfinite(s_val)):
            raise ValueError(f"Non-finite truth trajectory entry at index {idx}")
        if t_truth and t_val < t_truth[-1]:
            raise ValueError(f"Truth trajectory timestamps not monotonic at index {idx}: {t_val} < {t_truth[-1]}")
        t_truth.append(t_val)
        truth_clean.append({"t": t_val, "x": x_val, "y": y_val, "speed": s_val})

    est_clean: list[dict[str, float]] = []
    t_est: list[float] = []
    for idx, pt in enumerate(est_traj):
        t_val = float(pt["t"])
        x_val = float(pt["x"])
        y_val = float(pt["y"])
        s_val = float(pt["speed"])
        if not (math.isfinite(t_val) and math.isfinite(x_val) and math.isfinite(y_val) and math.isfinite(s_val)):
            raise ValueError(f"Non-finite estimated trajectory entry at index {idx}")
        if t_est and t_val < t_est[-1]:
            raise ValueError(f"Estimated trajectory timestamps not monotonic at index {idx}: {t_val} < {t_est[-1]}")
        t_est.append(t_val)
        est_clean.append({"t": t_val, "x": x_val, "y": y_val, "speed": s_val})

    truth_span = t_truth[-1] - t_truth[0]
    allowed_gap = max(float(min_duration_tol_s), float(duration_tol_frac) * max(truth_span, 0.0))
    if t_est[0] > t_truth[0] + allowed_gap:
        raise ValueError(
            f"Estimated trajectory starts too late ({t_est[0]:.3f}s vs truth {t_truth[0]:.3f}s)"
        )
    if t_est[-1] < t_truth[-1] - allowed_gap:
        raise ValueError(
            f"Estimated trajectory truncated at {t_est[-1]:.3f}s < truth {t_truth[-1]:.3f}s "
            f"(allowed gap {allowed_gap:.3f}s)"
        )
    if t_est[-1] > t_truth[-1] + allowed_gap:
        raise ValueError(
            f"Estimated trajectory exceeds truth duration ({t_est[-1]:.3f}s > {t_truth[-1]:.3f}s)"
        )

    aligned_truth: list[dict[str, float]] = []
    t_min, t_max = t_truth[0], t_truth[-1]
    n_truth = len(t_truth)
    for pt in est_clean:
        tq = min(max(pt["t"], t_min), t_max)
        pos = bisect_right(t_truth, tq)
        if pos <= 0:
            i0 = i1 = 0
        elif pos >= n_truth:
            i0 = i1 = n_truth - 1
        else:
            i0 = pos - 1
            i1 = pos
        p0 = truth_clean[i0]
        p1 = truth_clean[i1]
        dt = p1["t"] - p0["t"]
        w = ((tq - p0["t"]) / dt) if dt > 0.0 else 0.0
        aligned_truth.append(
            {
                "t": pt["t"],
                "x": p0["x"] + w * (p1["x"] - p0["x"]),
                "y": p0["y"] + w * (p1["y"] - p0["y"]),
                "speed": p0["speed"] + w * (p1["speed"] - p0["speed"]),
            }
        )

    return est_clean, aligned_truth


# ---------------------------------------------------------------------------
# 5-Pillar Production-Readiness Rubric Evaluator & Table Formatter
# ---------------------------------------------------------------------------

@dataclass
class RubricCheck:
    name: str
    passed: bool
    detail: str


@dataclass
class RubricPillar:
    pillar_id: int
    title: str
    passed: bool
    checks: list[RubricCheck]


@dataclass
class RubricReport:
    mode: str  # "CONTRACT-ONLY" or "FULL"
    overall_pass: bool
    passed_checks: int
    total_checks: int
    pillars: list[RubricPillar]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _check_m1_five_metrics(contract: dict[str, Any], spec_text: str) -> list[RubricCheck]:
    ut = contract.get("units_and_tolerances")
    if not isinstance(ut, dict):
        ut = {}
    examples = contract.get("examples")
    rep = examples.get("analytics_report") if isinstance(examples, dict) else None

    schema_err: str | None = None
    if not isinstance(rep, dict):
        schema_err = "missing examples.analytics_report"
        rep = {}
    else:
        try:
            get_validator("analytics_report").validate(rep)
        except ValidationError as exc:
            schema_err = exc.message

    schema_ok = schema_err is None
    err_suffix = f" [analytics_report invalid: {schema_err}]" if schema_err else ""

    vel = ut.get("velocity") if isinstance(ut.get("velocity"), dict) else {}
    vel_tol = str(vel.get("tolerance", "<missing>"))

    acc = ut.get("acceleration") if isinstance(ut.get("acceleration"), dict) else {}
    acc_range = acc.get("operating_range", "<missing>")
    acc_tol = str(acc.get("tolerance", "<missing>"))

    pos = ut.get("position") if isinstance(ut.get("position"), dict) else {}
    pos_tol = str(pos.get("tolerance", "<missing>"))

    sh = ut.get("shift_duration") if isinstance(ut.get("shift_duration"), dict) else {}
    sh_tol = str(sh.get("tolerance", "<missing>"))

    dist = ut.get("distance") if isinstance(ut.get("distance"), dict) else {}
    dist_tol = str(dist.get("tolerance", "<missing>"))

    traj = rep.get("trajectory") if isinstance(rep.get("trajectory"), list) else []
    shifts = rep.get("shifts") if isinstance(rep.get("shifts"), list) else []
    ss = rep.get("session_summary") if isinstance(rep.get("session_summary"), dict) else {}
    presets = contract.get("rink_geometry_presets") if isinstance(contract.get("rink_geometry_presets"), dict) else {}
    rink_preset = ss.get("rink_preset")

    speed_ok = (
        schema_ok
        and "velocity" in ut
        and bool(re.search(r"(?<![\d.])0\.4\s*m/s(?![A-Za-z0-9_])", vel_tol))
        and bool(traj)
        and isinstance(traj[0], dict)
        and "speed" in traj[0]
    )
    accel_ok = (
        schema_ok
        and "acceleration" in ut
        and acc_range == [-2008.40192, 2008.40192]
        and bool(re.search(r"(?<![\d.])200\s*g(?![A-Za-z0-9_])", acc_tol))
    )
    pos_ok = (
        schema_ok
        and "position" in ut
        and bool(re.search(r"\bCEP50\b", pos_tol))
        and bool(traj)
        and isinstance(rink_preset, str)
        and rink_preset in presets
        and all(
            isinstance(pt, dict)
            and "x" in pt
            and "y" in pt
            and is_inside_rink(pt["x"], pt["y"], presets[rink_preset])
            for pt in traj
        )
    )
    shift_ok = (
        schema_ok
        and "shift_duration" in ut
        and bool(re.search(r"(?<![\d.])1\.5\s*s(?![A-Za-z0-9_])", sh_tol))
        and len(shifts) == ss.get("shift_count")
    )
    dist_ok = (
        schema_ok
        and "distance" in ut
        and bool(re.search(r"(?<![\d.])5%", dist_tol))
        and "R1.1" in spec_text
    )

    checks = [
        RubricCheck("speed_contract_bounds", speed_ok, f"velocity tol='{vel_tol}'{err_suffix}"),
        RubricCheck("accel_0_200g_contract", accel_ok, f"accel range={acc_range}{err_suffix}"),
        RubricCheck("position_2d_cep50_contract", pos_ok, f"pos tol='{pos_tol}'{err_suffix}"),
        RubricCheck("shift_length_contract", shift_ok, f"shift tol='{sh_tol}'{err_suffix}"),
        RubricCheck("distance_skated_contract", dist_ok, f"distance tol='{dist_tol}'{err_suffix}"),
    ]
    return checks


def _check_m1_zero_gps(contract: dict[str, Any], spec_text: str) -> list[RubricCheck]:
    c = contract["constants"]
    hw = contract["hardware_abstraction"]
    ut = contract["units_and_tolerances"]
    rep_sum = contract["examples"]["analytics_report"]["session_summary"]
    return [
        RubricCheck(
            "constants_gps_anchors_disabled",
            c["gps_enabled"] is False and c["external_anchors_enabled"] is False,
            f"gps_enabled={c['gps_enabled']}, external_anchors_enabled={c['external_anchors_enabled']}",
        ),
        RubricCheck(
            "hw_uses_gps_anchors_false",
            hw["uses_gps"] is False and hw["uses_external_anchors"] is False and ut["gps_or_anchors_used"] is False,
            f"uses_gps={hw['uses_gps']}, uses_external_anchors={hw['uses_external_anchors']}",
        ),
        RubricCheck(
            "analytics_gps_anchors_const_false",
            rep_sum["gps_used"] is False and rep_sum["external_anchors_used"] is False and "zero reliance on GPS" in spec_text,
            f"report gps_used={rep_sum['gps_used']}, external_anchors_used={rep_sum['external_anchors_used']}",
        ),
    ]


def _check_m1_packaging(contract: dict[str, Any], spec_text: str) -> list[RubricCheck]:
    env = contract["hardware_abstraction"]["enclosure_envelope_mm"]
    mount_enum = contract["schemas"]["flash_session_header"]["properties"]["mount_location"]["enum"]
    return [
        RubricCheck(
            "enclosure_dimensions_mass_ip67",
            env["length_mm"] <= 52.0
            and env["width_mm"] <= 36.0
            and env["thickness_mm"] <= 10.5
            and env["mass_g"] <= 28.0
            and env["ip_rating"] == "IP67",
            f"{env['length_mm']}x{env['width_mm']}x{env['thickness_mm']} mm, {env['mass_g']} g, {env['ip_rating']}",
        ),
        RubricCheck(
            "under_jersey_retention_and_eptfe",
            set(mount_enum) == {"STERNUM_POCKET", "CHEST_STRAP"} and "ePTFE" in spec_text and "Shore 70A" in spec_text,
            f"mount_location={mount_enum}",
        ),
    ]


def _check_m1_cold_battery_flash(contract: dict[str, Any], spec_text: str) -> list[RubricCheck]:
    hw = contract["hardware_abstraction"]
    bff = contract["binary_flash_format"]
    usable_bytes = hw["flash"]["usable_ring_blocks"] * hw["flash"]["block_bytes"]
    return [
        RubricCheck(
            "min_battery_life_ge_2h_and_cold_derate",
            hw["min_battery_life_hours"] >= 2.0 and "292.5 mAh" in spec_text and "91.3 h" in spec_text,
            f"min_battery_life_hours={hw['min_battery_life_hours']}",
        ),
        RubricCheck(
            "flash_ring_ge_2h_capacity",
            hw["flash"]["usable_ring_blocks"] >= 998 and usable_bytes >= 130_809_856,
            f"usable_ring_blocks={hw['flash']['usable_ring_blocks']} ({usable_bytes} B)",
        ),
        RubricCheck(
            "uint64_timestamp_no_71min_rollover",
            bff["timestamp_bits"] >= 64
            and bff["timestamp_type"] == "uint64_t"
            and contract["examples"]["imu_batch_packet"]["timestamp_us"] > 0xFFFFFFFF,
            f"timestamp_type={bff['timestamp_type']} ({bff['timestamp_bits']}-bit)",
        ),
    ]


def _check_m1_download_integrity(contract: dict[str, Any]) -> list[RubricCheck]:
    dp = contract["download_protocol"]
    bff = contract["binary_flash_format"]
    return [
        RubricCheck(
            "download_commands_and_crc32_ieee",
            len(dp["commands"]) == 7
            and dp["status_codes"].get("ERR_NOT_VERIFIED") == "0x05"
            and bff["crc_polynomial"] == "0xEDB88320"
            and crc32_ieee(b"123456789") == 0xCBF43926,
            f"commands={len(dp['commands'])}, CRC32('123456789')=0xCBF43926",
        ),
        RubricCheck(
            "pre_footer_sha256_and_schemas_valid",
            "pre_footer_sha256" in contract["schemas"]["flash_session_footer"]["required"]
            and dp["request_sync_u16"] == "0xAA55"
            and dp["response_sync_u16"] == "0x55AA",
            "pre_footer_sha256 + 0xAA55/0x55AA enforced",
        ),
    ]


def extract_power_budget_metrics(budget: dict[str, Any]) -> dict[str, Any]:
    """Normalize flat or nested power_budget.py --json keys into canonical metric names."""
    bat = budget.get("battery", {})
    modes = budget.get("operating_modes", {})
    sag = budget.get("ohmic_sag_cold_0c", {})
    nand = budget.get("nand_storage", {})

    derated_mah = budget.get(
        "derated_capacity_mah",
        bat.get("usable_capacity_add_mah", bat.get("usable_capacity_mul_mah")),
    )
    nom_h = budget.get(
        "nominal_runtime_h",
        modes.get("nominal_play", {}).get("runtime_add_h"),
    )
    wc_h = budget.get(
        "worst_case_runtime_h",
        modes.get("worst_case_logging", {}).get("runtime_add_h"),
    )
    ohmic_mv = budget.get("ohmic_sag_mv", sag.get("continuous_wc_sag_mv"))
    flash_bytes = budget.get("flash_usable_bytes", nand.get("usable_ring_bytes"))
    flash_nom_h = budget.get(
        "flash_nominal_hours",
        nand.get("nominal_burst_3200hz", {}).get("retention_hours"),
    )
    flash_400_h = budget.get(
        "flash_cont_400hz_hours",
        nand.get("continuous_400hz_n40", {}).get(
            "retention_hours",
            nand.get("continuous_400hz_n50", {}).get("retention_hours"),
        ),
    )
    ble_mah = budget.get(
        "ble_offload_mah",
        modes.get("ble_2m_bulk_offload", {}).get("offload_energy_mah"),
    )
    return {
        "derated_capacity_mah": float(derated_mah),
        "nominal_runtime_h": float(nom_h),
        "worst_case_runtime_h": float(wc_h),
        "ohmic_sag_mv": float(ohmic_mv) if ohmic_mv is not None else None,
        "flash_usable_bytes": int(flash_bytes) if flash_bytes is not None else None,
        "flash_nominal_hours": float(flash_nom_h),
        "flash_cont_400hz_hours": float(flash_400_h),
        "ble_offload_mah": float(ble_mah) if ble_mah is not None else None,
    }


def scan_zero_gps_compliance(ee_bom_path: Path, pinmux_path: Path) -> list[str]:
    """Check EE BOM CSV rows and pinmux devices/invariants for GPS/UWB/anchor reliance."""
    import csv
    import re

    hits: list[str] = []
    negation_re = re.compile(r"\b(?:anchor[-_ ]?free|no[-_ ]gps|zero[-_ ]gps|non[-_ ]gps)\b", re.IGNORECASE)
    with ee_bom_path.open("r", encoding="utf-8") as f:
        for idx, row in enumerate(csv.DictReader(f), start=2):
            part_blob = " ".join(
                str(row.get(k, ""))
                for k in ("refdes", "subsystem_key", "category", "part_number", "manufacturer", "bus_or_function")
            ).lower()
            notes_clean = negation_re.sub("", str(row.get("notes", ""))).lower()
            for kw in FORBIDDEN_BOM_KEYWORDS:
                if kw in part_blob or kw in notes_clean:
                    hits.append(f"bom.csv row {idx}: '{kw}'")

    pinmux = load_json_strict(pinmux_path)
    inv = pinmux.get("invariants", pinmux)
    for flag in ("uses_gps", "uses_external_anchors", "uses_uwb"):
        if flag in inv and inv[flag] is not False:
            hits.append(f"pinmux.invariants.{flag}={inv[flag]!r}")

    def _walk_devices(node: Any, path_str: str) -> None:
        if isinstance(node, dict):
            for k, v in node.items():
                if k in ("part_number", "full_part_number", "antenna_part", "connector_part", "esd_ic"):
                    sval = str(v).lower()
                    for kw in FORBIDDEN_BOM_KEYWORDS:
                        if kw in sval:
                            hits.append(f"{path_str}.{k}: '{kw}' in {v}")
                elif k not in ("invariants", "deployment_mode", "notes", "rationale", "silicon_erratum_note"):
                    _walk_devices(v, f"{path_str}.{k}")
        elif isinstance(node, list):
            for i, item in enumerate(node):
                _walk_devices(item, f"{path_str}[{i}]")

    _walk_devices(pinmux.get("buses", pinmux), "pinmux.buses")
    return hits


def extract_mechanical_dims(dims: dict[str, Any]) -> dict[str, Any]:
    """Normalize flat or nested dimensions.json keys into standard envelope/mass/ip_rating fields."""
    env = dims.get("envelope_mm", dims)
    mass_blk = dims.get("mass_budget_g", dims)
    env_imp = dims.get("environmental_and_impact", dims)
    length_mm = float(env.get("length_mm", dims.get("length_mm")))
    width_mm = float(env.get("width_mm", dims.get("width_mm")))
    thickness_mm = float(env.get("thickness_mm", dims.get("thickness_mm")))
    mass_val = mass_blk.get("target_pod_mass_g", mass_blk.get("mass_g", dims.get("mass_g")))
    ip_rating = env_imp.get("ip_rating", dims.get("ip_rating"))
    return {
        "length_mm": length_mm,
        "width_mm": width_mm,
        "thickness_mm": thickness_mm,
        "mass_g": float(mass_val),
        "ip_rating": ip_rating,
    }


def _check_full_downstream_pillar(pillar_id: int) -> list[RubricCheck]:
    """Evaluate downstream M2/M3/M4 runtime checks for Strict Full Mode."""
    checks: list[RubricCheck] = []
    if pillar_id == 1:
        try:
            require_m4()
            checks.append(RubricCheck("m4_sim_modules_present", True, "sim/ modules present"))
        except AssertionError as exc:
            checks.append(RubricCheck("m4_sim_modules_present", False, str(exc)))
            return checks

        proc = run_sim_unit_tests_cached()
        checks.append(
            RubricCheck(
                "m4_sim_unit_tests_pass",
                proc.returncode == 0,
                f"sim/tests rc={proc.returncode}",
            )
        )
    elif pillar_id == 2:
        try:
            m2 = require_m2()
            hits = scan_zero_gps_compliance(m2["ee_bom"], m2["pinmux"])
            checks.append(
                RubricCheck(
                    "m2_bom_pinmux_zero_gps_anchors",
                    len(hits) == 0,
                    "zero forbidden GPS/UWB/anchor parts" if not hits else f"forbidden hits: {hits}",
                )
            )
        except (AssertionError, KeyError, ValueError) as exc:
            checks.append(RubricCheck("m2_bom_pinmux_zero_gps_anchors", False, str(exc)))
    elif pillar_id == 3:
        try:
            m2 = require_m2()
            raw_dims = load_json_strict(m2["dimensions"])
            dims = extract_mechanical_dims(raw_dims)
            enc_txt = m2["enclosure_spec"].read_text(encoding="utf-8")
            ok_dim = (
                dims["length_mm"] <= 52.0
                and dims["width_mm"] <= 36.0
                and dims["thickness_mm"] <= 10.5
                and dims["mass_g"] <= 28.0
                and dims["ip_rating"] == "IP67"
            )
            checks.append(
                RubricCheck(
                    "m2_mechanical_dimensions_and_enclosure",
                    ok_dim and ("ePTFE" in enc_txt or "eptfe" in enc_txt.lower()),
                    f"dims={dims['length_mm']}x{dims['width_mm']}x{dims['thickness_mm']} mm, {dims['mass_g']} g",
                )
            )
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            checks.append(RubricCheck("m2_mechanical_dimensions_and_enclosure", False, str(exc)))
    elif pillar_id == 4:
        try:
            budget = extract_power_budget_metrics(run_power_budget_json())
            ok = (
                0.0 < budget["derated_capacity_mah"] <= 306.0
                and budget["nominal_runtime_h"] >= 2.0
                and budget["worst_case_runtime_h"] >= 2.0
                and budget["flash_nominal_hours"] >= 2.0
                and budget["flash_cont_400hz_hours"] >= 2.0
            )
            checks.append(
                RubricCheck(
                    "m2_cold_power_and_flash_budget_json",
                    ok,
                    f"derated_mah={budget['derated_capacity_mah']}, worst_case_h={budget['worst_case_runtime_h']}, flash_nom_h={budget['flash_nominal_hours']}",
                )
            )
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            checks.append(RubricCheck("m2_cold_power_and_flash_budget_json", False, str(exc)))
    elif pillar_id == 5:
        try:
            require_m3()
            proc = run_firmware_make_test_cached()
            checks.append(
                RubricCheck(
                    "m3_firmware_c11_build_and_test",
                    proc.returncode == 0,
                    f"make -C firmware test rc={proc.returncode}",
                )
            )
        except AssertionError as exc:
            checks.append(RubricCheck("m3_firmware_c11_build_and_test", False, str(exc)))
    return checks


def evaluate_rubric(*, contract_only: bool | None = None) -> RubricReport:
    if contract_only is None:
        contract_only = is_contract_only_mode()

    contract = load_contract()
    spec_text = load_spec_text()

    pillar_specs: list[tuple[int, str, list[RubricCheck]]] = [
        (1, "Five Tracked Metrics (Speed, Accel, 2D Pos, Shift, Distance)", _check_m1_five_metrics(contract, spec_text)),
        (2, "Zero GPS & Zero External Rink-Side Anchors", _check_m1_zero_gps(contract, spec_text)),
        (3, "Low-Profile Chest-Worn Under-Jersey Form Factor", _check_m1_packaging(contract, spec_text)),
        (4, "Cold-Rated (>= 2 h) Battery & Flash Budget", _check_m1_cold_battery_flash(contract, spec_text)),
        (5, "Post-Session Download & Binary Integrity", _check_m1_download_integrity(contract)),
    ]

    pillars: list[RubricPillar] = []
    total_checks = 0
    passed_checks = 0
    for pid, title, m1_checks in pillar_specs:
        checks = list(m1_checks)
        if not contract_only:
            checks.extend(_check_full_downstream_pillar(pid))
        p_ok = all(c.passed for c in checks)
        total_checks += len(checks)
        passed_checks += sum(1 for c in checks if c.passed)
        pillars.append(RubricPillar(pillar_id=pid, title=title, passed=p_ok, checks=checks))

    overall_pass = all(p.passed for p in pillars)
    mode_label = "CONTRACT-ONLY" if contract_only else "FULL"
    return RubricReport(
        mode=mode_label,
        overall_pass=overall_pass,
        passed_checks=passed_checks,
        total_checks=total_checks,
        pillars=pillars,
    )


def format_rubric_table(report: RubricReport) -> str:
    mode_banner = (
        "MODE: CONTRACT-ONLY (M2/M3/M4 runtime checks excluded)"
        if report.mode == "CONTRACT-ONLY"
        else "MODE: FULL (M1 + M2 + M3 + M4 strict verification)"
    )
    lines = [
        "=" * 88,
        f"HOCKEY_SENSOR PRODUCTION-READINESS RUBRIC  |  {mode_banner}",
        "=" * 88,
        f"{'Pillar':<6} | {'Status':<6} | {'Checks':<7} | {'Acceptance Pillar'}",
        "-" * 88,
    ]
    for p in report.pillars:
        p_pass = sum(1 for c in p.checks if c.passed)
        status = "PASS" if p.passed else "FAIL"
        lines.append(f"P{p.pillar_id:<5} | {status:<6} | {p_pass}/{len(p.checks):<5} | {p.title}")
        for chk in p.checks:
            mark = "OK  " if chk.passed else "FAIL"
            lines.append(f"       | [{mark}] | {chk.name}: {chk.detail}")
    lines.append("-" * 88)
    verdict = "PASS" if report.overall_pass else "FAIL"
    lines.append(
        f"OVERALL RUBRIC VERDICT: {verdict} ({report.passed_checks}/{report.total_checks} checks passed)"
    )
    lines.append("=" * 88)
    return "\n".join(lines)
