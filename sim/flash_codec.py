"""Binary .hks flash log encoder/decoder matching INTERFACE_CONTRACT.json v1.0.0 and C11 firmware."""

from __future__ import annotations

import hashlib
import json
import math
import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import Any, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Protocol & Hardware Constants (locked to INTERFACE_CONTRACT.json)
# ---------------------------------------------------------------------------

FRAME_SYNC = 0xAA55
FRAME_HEADER_FMT = "<HBBHHQI"
FRAME_HEADER_STRUCT = struct.Struct(FRAME_HEADER_FMT)
FRAME_HEADER_BYTES = FRAME_HEADER_STRUCT.size  # 20 B
FRAME_CRC_STRUCT = struct.Struct("<I")
FRAME_CRC_BYTES = FRAME_CRC_STRUCT.size  # 4 B
FRAME_OVERHEAD_BYTES = FRAME_HEADER_BYTES + FRAME_CRC_BYTES  # 24 B
MAX_PAYLOAD_BYTES = 4072
MAX_FRAME_BYTES = 4096

SESSION_MAGIC = b"HKS1"
SESSION_FORMAT_VERSION = 1
DEFAULT_HW_REV = 0x0100
DEFAULT_FW_VERSION = 0x01000000

GRAVITY_MPS2 = 9.80665
DEFAULT_ACCEL_SCALE = GRAVITY_MPS2 / 2048.0  # 0.0047884033203125 (m/s^2)/LSB
DEFAULT_GYRO_SCALE = (math.pi / 180.0) / 16.4  # 0.001064225153655079 (rad/s)/LSB
DEFAULT_HIGH_G_SCALE = 0.1 * GRAVITY_MPS2  # 0.980665 (m/s^2)/LSB
DEFAULT_MAG_SCALE = 100.0 / 16384.0  # 0.006103515625 uT/LSB
DEFAULT_PRESSURE_SCALE = 1.0 / 64.0  # 0.015625 Pa/LSB

CANONICAL_R_BS: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
)
IDENTITY_3X3: tuple[tuple[float, float, float], ...] = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

# Payload struct formats
SESSION_HEADER_STRUCT = struct.Struct("<4sHHIIQQQHHHHffffBBHffI")  # 80 B
IMU_BATCH_HDR_STRUCT = struct.Struct("<HHhH")  # 8 B
IMU_SAMPLE_STRUCT = struct.Struct("<hhhhhh")  # 12 B
HIGH_G_BATCH_HDR_STRUCT = struct.Struct("<HHBBH")  # 8 B
HIGH_G_SAMPLE_STRUCT = struct.Struct("<hhh")  # 6 B
AUX_ENV_STRUCT = struct.Struct("<iiiIhHhHHH")  # 28 B
EVENT_MARKER_STRUCT = struct.Struct("<HHfQ")  # 16 B
CAL_BLOCK_BODY_STRUCT = struct.Struct("<HHQ9f9f3f9f3f9f9f3f3f9fff")  # 284 B
CAL_BLOCK_STRUCT = struct.Struct("<HHQ9f9f3f9f3f9f9f3f3f9fffI")  # 288 B
SESSION_FOOTER_STRUCT = struct.Struct("<QIIIIIIQ32s")  # 72 B

IMU_MAX_SAMPLES = 338
HIGH_G_MAX_SAMPLES = 677
SESSION_FOOTER_FRAME_BYTES = FRAME_OVERHEAD_BYTES + SESSION_FOOTER_STRUCT.size  # 96 B

VALID_IMU_ODRS = frozenset((100, 200, 400))
VALID_HIGH_G_HEADER_ODRS = frozenset((0, 400, 800, 1600, 3200))
VALID_HIGH_G_BATCH_ODRS = frozenset((400, 800, 1600, 3200))
VALID_MAG_ODRS = frozenset((25, 50, 100))
VALID_BARO_ODRS = frozenset((10, 25, 50))


class MsgType(IntEnum):
    SESSION_HEADER = 0x01
    IMU_BATCH = 0x02
    HIGH_G_BATCH = 0x03
    AUX_ENV = 0x04
    EVENT_MARKER = 0x05
    CALIBRATION_BLOCK = 0x06
    SESSION_FOOTER = 0x07


class FrameFlag(IntFlag):
    ACCEL_SAT = 1 << 0
    GYRO_SAT = 1 << 1
    MAG_ANOM = 1 << 2
    LOW_BATT = 1 << 3
    HIGH_G_ACTIVE = 1 << 4
    UTC_SYNCED = 1 << 5


class AuxValidityFlag(IntFlag):
    MAG_FRESH = 1 << 0
    BARO_FRESH = 1 << 1
    PMIC_FRESH = 1 << 2
    MAG_SET_RESET = 1 << 3


ID_TO_RINK_PRESET: dict[int, str] = {
    0: "NHL_200X85FT",
    1: "IIHF_60X30M",
    2: "CUSTOM",
}
RINK_PRESET_TO_ID: dict[str, int] = {v: k for k, v in ID_TO_RINK_PRESET.items()}

ID_TO_MOUNT: dict[int, str] = {
    0: "STERNUM_POCKET",
    1: "CHEST_STRAP",
}
MOUNT_TO_ID: dict[str, int] = {v: k for k, v in ID_TO_MOUNT.items()}

ID_TO_TRIGGER_MODE: dict[int, str] = {
    0: "CONTINUOUS",
    1: "IMPACT_BURST",
}
TRIGGER_MODE_TO_ID: dict[str, int] = {v: k for k, v in ID_TO_TRIGGER_MODE.items()}

ID_TO_EVENT_CODE: dict[int, str] = {
    1: "SESSION_START",
    2: "BUTTON_MARK",
    3: "IMPACT_SPIKE",
    4: "SHIFT_HINT_ON",
    5: "SHIFT_HINT_OFF",
    6: "LOW_BATT",
    7: "BOARD_COLLISION",
    8: "SESSION_END",
}
EVENT_CODE_TO_ID: dict[str, int] = {v: k for k, v in ID_TO_EVENT_CODE.items()}

ID_TO_END_REASON: dict[int, str] = {
    0: "BUTTON_STOP",
    1: "USB_DOCK",
    2: "LOW_BATT",
    3: "FLASH_FULL",
    4: "WATCHDOG_RECOVER",
}
END_REASON_TO_ID: dict[str, int] = {v: k for k, v in ID_TO_END_REASON.items()}


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class CodecError(ValueError):
    """Base error for binary .hks framing, payload, or integrity failures."""


class BadSyncError(CodecError):
    """Frame sync marker != 0xAA55."""


class BadCRCError(CodecError):
    """Frame CRC32-IEEE mismatch."""


class TruncatedFrameError(CodecError):
    """Buffer too short for complete frame header, payload, and CRC."""


class PayloadLengthError(CodecError):
    """Payload length invalid or mismatched with message format."""


class UnknownMessageTypeError(CodecError):
    """Unrecognized msg_type outside 0x01..0x07."""


class BadMagicError(CodecError):
    """Session header magic != b'HKS1'."""


class UnsupportedVersionError(CodecError):
    """Session format_version != 1."""


class CalibrationCRCError(CodecError):
    """Calibration block internal cal_crc32 mismatch."""


class CalibrationMismatchError(CodecError):
    """SessionHeader.cal_crc32 does not match CalibrationBlock.cal_crc32."""


class Sha256MismatchError(CodecError):
    """Session footer pre_footer_sha256 mismatch."""


class FooterError(CodecError):
    """Missing, misplaced, or inconsistent SESSION_FOOTER."""


class InvalidFieldError(CodecError):
    """Out-of-range, non-finite, or invalid semantic field value."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def crc32_ieee(data: bytes | bytearray | memoryview) -> int:
    """IEEE 802.3 CRC32 (reflected poly 0xEDB88320, init/xorout 0xFFFFFFFF)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def _require_finite(val: float, name: str) -> float:
    f = float(val)
    if not math.isfinite(f):
        raise InvalidFieldError(f"{name} must be finite, got {val!r}")
    return f


def _validate_mat3(m: Any, name: str, *, check_so3: bool = False, tol: float = 1e-5) -> tuple[tuple[float, float, float], ...]:
    if isinstance(m, np.ndarray):
        arr = m.tolist()
    else:
        arr = m
    if not isinstance(arr, (list, tuple)) or len(arr) != 3 or any(not isinstance(r, (list, tuple)) or len(r) != 3 for r in arr):
        raise InvalidFieldError(f"{name} must be a 3x3 matrix")
    rows = tuple(tuple(_require_finite(arr[i][j], f"{name}[{i}][{j}]") for j in range(3)) for i in range(3))
    if check_so3:
        mat = np.asarray(rows, dtype=np.float64)
        ortho_err = float(np.max(np.abs(mat.T @ mat - np.eye(3))))
        if ortho_err > tol:
            raise InvalidFieldError(f"{name} is not orthogonal (max |R^T R - I| = {ortho_err:.6g} > {tol})")
        det = float(np.linalg.det(mat))
        if abs(det - 1.0) > tol:
            raise InvalidFieldError(f"{name} must have det=+1, got {det:.6g}")
    return rows


def _validate_vec3(v: Any, name: str) -> tuple[float, float, float]:
    if isinstance(v, np.ndarray):
        arr = v.tolist()
    else:
        arr = v
    if not isinstance(arr, (list, tuple)) or len(arr) != 3:
        raise InvalidFieldError(f"{name} must be a 3-element vector")
    return tuple(_require_finite(arr[i], f"{name}[{i}]") for i in range(3))  # type: ignore[return-value]


def _flatten_mat3(m: Sequence[Sequence[float]]) -> list[float]:
    return [float(m[i][j]) for i in range(3) for j in range(3)]


def _unflatten_mat3(vals: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    return (
        (float(vals[0]), float(vals[1]), float(vals[2])),
        (float(vals[3]), float(vals[4]), float(vals[5])),
        (float(vals[6]), float(vals[7]), float(vals[8])),
    )


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FrameHeader:
    sync: int
    msg_type: int
    flags: int
    payload_len: int
    seq: int
    timestamp_us: int
    reserved: int = 0


@dataclass(frozen=True)
class SessionHeader:
    session_id: int
    start_utc_ms: int = 0
    device_id: int = 0x00005CC800010203
    cal_crc32: int = 0
    magic: bytes = SESSION_MAGIC
    format_version: int = SESSION_FORMAT_VERSION
    hw_rev: int = DEFAULT_HW_REV
    fw_version: int = DEFAULT_FW_VERSION
    imu_odr_hz: int = 200
    high_g_odr_hz: int = 3200
    mag_odr_hz: int = 50
    baro_odr_hz: int = 25
    accel_scale_mps2_per_lsb: float = DEFAULT_ACCEL_SCALE
    gyro_scale_rads_per_lsb: float = DEFAULT_GYRO_SCALE
    high_g_scale_mps2_per_lsb: float = DEFAULT_HIGH_G_SCALE
    mag_scale_ut_per_lsb: float = DEFAULT_MAG_SCALE
    rink_preset: str = "NHL_200X85FT"
    mount_location: str = "STERNUM_POCKET"
    player_height_cm: int = 183
    bench_gate_xy_m: tuple[float, float] = (-6.0, 12.954)
    reserved: int = 0
    # Frame envelope fields
    flags: int = 0
    seq: int = 0
    timestamp_us: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.SESSION_HEADER)

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.SESSION_HEADER),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "magic": self.magic.decode("ascii"),
            "format_version": int(self.format_version),
            "hw_rev": int(self.hw_rev),
            "fw_version": int(self.fw_version),
            "cal_crc32": int(self.cal_crc32),
            "session_id": int(self.session_id),
            "start_utc_ms": int(self.start_utc_ms),
            "device_id": int(self.device_id),
            "imu_odr_hz": int(self.imu_odr_hz),
            "high_g_odr_hz": int(self.high_g_odr_hz),
            "mag_odr_hz": int(self.mag_odr_hz),
            "baro_odr_hz": int(self.baro_odr_hz),
            "accel_scale_mps2_per_lsb": float(self.accel_scale_mps2_per_lsb),
            "gyro_scale_rads_per_lsb": float(self.gyro_scale_rads_per_lsb),
            "high_g_scale_mps2_per_lsb": float(self.high_g_scale_mps2_per_lsb),
            "mag_scale_ut_per_lsb": float(self.mag_scale_ut_per_lsb),
            "rink_preset": str(self.rink_preset),
            "mount_location": str(self.mount_location),
            "player_height_cm": int(self.player_height_cm),
            "bench_gate_xy_m": [float(self.bench_gate_xy_m[0]), float(self.bench_gate_xy_m[1])],
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class CalibrationBlock:
    cal_version: int = 1
    flags: int = 0x000F
    cal_utc_ms: int = 1763852400000
    sensor_to_body_rotation_matrix: tuple[tuple[float, float, float], ...] = CANONICAL_R_BS
    accel_misalign_scale: tuple[tuple[float, float, float], ...] = IDENTITY_3X3
    accel_bias_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_misalign_scale: tuple[tuple[float, float, float], ...] = IDENTITY_3X3
    gyro_bias_rads: tuple[float, float, float] = (0.0, 0.0, 0.0)
    high_g_to_body_rotation_matrix: tuple[tuple[float, float, float], ...] = CANONICAL_R_BS
    high_g_misalign_scale: tuple[tuple[float, float, float], ...] = IDENTITY_3X3
    high_g_bias_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_hard_iron_ut: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_soft_iron_matrix: tuple[tuple[float, float, float], ...] = IDENTITY_3X3
    baseline_mag_norm_ut: float = 51.2
    baseline_mag_dip_deg: float = 66.4
    cal_crc32: int = 0
    # Frame metadata (not emitted in calibration_block JSON schema)
    frame_flags: int = 0
    frame_seq: int = 1
    frame_timestamp_us: int = 0
    frame_crc32: int = 0

    def compute_crc32(self) -> int:
        raw = pack_calibration_payload(self, recompute_crc=True)
        return struct.unpack_from("<I", raw, 284)[0]

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "cal_version": int(self.cal_version),
            "flags": int(self.flags),
            "cal_utc_ms": int(self.cal_utc_ms),
            "sensor_to_body_rotation_matrix": [[float(x) for x in r] for r in self.sensor_to_body_rotation_matrix],
            "accel_misalign_scale": [[float(x) for x in r] for r in self.accel_misalign_scale],
            "accel_bias_mps2": [float(x) for x in self.accel_bias_mps2],
            "gyro_misalign_scale": [[float(x) for x in r] for r in self.gyro_misalign_scale],
            "gyro_bias_rads": [float(x) for x in self.gyro_bias_rads],
            "high_g_to_body_rotation_matrix": [[float(x) for x in r] for r in self.high_g_to_body_rotation_matrix],
            "high_g_misalign_scale": [[float(x) for x in r] for r in self.high_g_misalign_scale],
            "high_g_bias_mps2": [float(x) for x in self.high_g_bias_mps2],
            "mag_hard_iron_ut": [float(x) for x in self.mag_hard_iron_ut],
            "mag_soft_iron_matrix": [[float(x) for x in r] for r in self.mag_soft_iron_matrix],
            "baseline_mag_norm_ut": float(self.baseline_mag_norm_ut),
            "baseline_mag_dip_deg": float(self.baseline_mag_dip_deg),
            "cal_crc32": int(self.cal_crc32),
        }


@dataclass(frozen=True)
class ImuSample:
    ax_i16: int
    ay_i16: int
    az_i16: int
    gx_i16: int
    gy_i16: int
    gz_i16: int


@dataclass(frozen=True)
class ImuBatchPacket:
    timestamp_us: int
    odr_hz: int
    temp_c_x100: int
    samples: tuple[ImuSample, ...]
    flags: int = 0
    seq: int = 0
    reserved: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.IMU_BATCH)
    _raw_bytes: bytes = b""

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    def to_numpy(
        self,
        accel_scale: float = DEFAULT_ACCEL_SCALE,
        gyro_scale: float = DEFAULT_GYRO_SCALE,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        n = len(self.samples)
        dt_us = 1_000_000.0 / float(self.odr_hz)
        ts = (float(self.timestamp_us) + np.arange(n, dtype=np.float64) * dt_us) * 1e-6
        if self._raw_bytes and len(self._raw_bytes) == n * 12:
            arr = np.frombuffer(self._raw_bytes, dtype="<i2").reshape(n, 6).astype(np.float64)
        else:
            arr = np.array(
                [(s.ax_i16, s.ay_i16, s.az_i16, s.gx_i16, s.gy_i16, s.gz_i16) for s in self.samples],
                dtype=np.float64,
            )
        accel = arr[:, :3] * float(accel_scale)
        gyro = arr[:, 3:6] * float(gyro_scale)
        return ts, accel, gyro

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.IMU_BATCH),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "sample_count": int(len(self.samples)),
            "odr_hz": int(self.odr_hz),
            "temp_c_x100": int(self.temp_c_x100),
            "samples": [
                {
                    "ax_i16": int(s.ax_i16),
                    "ay_i16": int(s.ay_i16),
                    "az_i16": int(s.az_i16),
                    "gx_i16": int(s.gx_i16),
                    "gy_i16": int(s.gy_i16),
                    "gz_i16": int(s.gz_i16),
                }
                for s in self.samples
            ],
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class HighGSample:
    ax_i16: int
    ay_i16: int
    az_i16: int


@dataclass(frozen=True)
class HighGBatchPacket:
    timestamp_us: int
    odr_hz: int
    trigger_mode: str
    samples: tuple[HighGSample, ...]
    flags: int = 0
    seq: int = 0
    reserved0: int = 0
    reserved1: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.HIGH_G_BATCH)

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    def to_numpy(self, high_g_scale: float = DEFAULT_HIGH_G_SCALE) -> tuple[np.ndarray, np.ndarray]:
        """Convert right-aligned >>4 12-bit int16 samples directly to m/s^2 (NO >>4 shift)."""
        n = len(self.samples)
        dt_us = 1_000_000.0 / float(self.odr_hz)
        ts = (float(self.timestamp_us) + np.arange(n, dtype=np.float64) * dt_us) * 1e-6
        arr = np.array([(s.ax_i16, s.ay_i16, s.az_i16) for s in self.samples], dtype=np.float64)
        return ts, arr * float(high_g_scale)

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.HIGH_G_BATCH),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "sample_count": int(len(self.samples)),
            "odr_hz": int(self.odr_hz),
            "trigger_mode": str(self.trigger_mode),
            "samples": [
                {"ax_i16": int(s.ax_i16), "ay_i16": int(s.ay_i16), "az_i16": int(s.az_i16)}
                for s in self.samples
            ],
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class AuxEnvPacket:
    timestamp_us: int
    mx_i32: int
    my_i32: int
    mz_i32: int
    pressure_pa_x64: int
    baro_temp_c_x100: int
    vbatt_mv: int
    batt_temp_c_x100: int
    soc_pct_x10: int
    pmic_status: int
    validity_flags: int
    flags: int = 0
    seq: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.AUX_ENV)

    def mag_ut(self, mag_scale: float = DEFAULT_MAG_SCALE) -> np.ndarray:
        return np.array([self.mx_i32, self.my_i32, self.mz_i32], dtype=np.float64) * float(mag_scale)

    @property
    def pressure_pa(self) -> float:
        return float(self.pressure_pa_x64) * DEFAULT_PRESSURE_SCALE

    @property
    def baro_temp_c(self) -> float:
        return float(self.baro_temp_c_x100) * 0.01

    @property
    def batt_temp_c(self) -> float:
        return float(self.batt_temp_c_x100) * 0.01

    @property
    def soc_pct(self) -> float:
        return float(self.soc_pct_x10) * 0.1

    @property
    def pmic_status_u8(self) -> int:
        return int(self.pmic_status) & 0xFF

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.AUX_ENV),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "mx_i32": int(self.mx_i32),
            "my_i32": int(self.my_i32),
            "mz_i32": int(self.mz_i32),
            "pressure_pa_x64": int(self.pressure_pa_x64),
            "baro_temp_c_x100": int(self.baro_temp_c_x100),
            "vbatt_mv": int(self.vbatt_mv),
            "batt_temp_c_x100": int(self.batt_temp_c_x100),
            "soc_pct_x10": int(self.soc_pct_x10),
            "pmic_status": int(self.pmic_status),
            "validity_flags": int(self.validity_flags),
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class EventMarkerPacket:
    timestamp_us: int
    event_code: str
    severity: int
    value_f32: float
    detail_u64: int
    flags: int = 0
    seq: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.EVENT_MARKER)

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.EVENT_MARKER),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "event_code": str(self.event_code),
            "severity": int(self.severity),
            "value_f32": float(self.value_f32),
            "detail_u64": int(self.detail_u64),
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class SessionFooter:
    timestamp_us: int
    end_utc_ms: int
    total_imu_samples: int
    dropped_imu_samples: int
    total_high_g_samples: int
    dropped_high_g_samples: int
    total_aux_samples: int
    end_reason: str
    total_bytes: int
    pre_footer_sha256: str
    flags: int = 0
    seq: int = 0
    crc32: int = 0
    sync: int = FRAME_SYNC
    msg_type: int = int(MsgType.SESSION_FOOTER)

    def to_contract_dict(self) -> dict[str, Any]:
        return {
            "sync": int(self.sync),
            "msg_type": int(MsgType.SESSION_FOOTER),
            "flags": int(self.flags),
            "seq": int(self.seq),
            "timestamp_us": int(self.timestamp_us),
            "end_utc_ms": int(self.end_utc_ms),
            "total_imu_samples": int(self.total_imu_samples),
            "dropped_imu_samples": int(self.dropped_imu_samples),
            "total_high_g_samples": int(self.total_high_g_samples),
            "dropped_high_g_samples": int(self.dropped_high_g_samples),
            "total_aux_samples": int(self.total_aux_samples),
            "end_reason": str(self.end_reason),
            "total_bytes": int(self.total_bytes),
            "pre_footer_sha256": str(self.pre_footer_sha256).lower(),
            "crc32": int(self.crc32),
        }


@dataclass(frozen=True)
class DecodedHksSession:
    header: SessionHeader
    calibration: CalibrationBlock | None
    imu_batches: list[ImuBatchPacket]
    high_g_batches: list[HighGBatchPacket]
    aux_env_packets: list[AuxEnvPacket]
    event_markers: list[EventMarkerPacket]
    footer: SessionFooter | None
    frames: list[Any]
    raw_size_bytes: int
    file_crc32: int
    file_sha256: bytes


# ---------------------------------------------------------------------------
# Frame & Payload Pack/Unpack Functions
# ---------------------------------------------------------------------------


def pack_frame(
    msg_type: int,
    payload: bytes,
    *,
    seq: int,
    timestamp_us: int,
    flags: int = 0,
    reserved: int = 0,
) -> bytes:
    """Pack a single .hks binary frame (20 B header + payload + 4 B CRC32-IEEE)."""
    if not (0 <= msg_type <= 0xFF):
        raise InvalidFieldError(f"msg_type out of u8 range: {msg_type}")
    if not (0 <= flags <= 0xFF):
        raise InvalidFieldError(f"flags out of u8 range: {flags}")
    if not (0 <= seq <= 0xFFFF):
        raise InvalidFieldError(f"seq out of u16 range: {seq}")
    if not (0 <= timestamp_us < (1 << 64)):
        raise InvalidFieldError(f"timestamp_us out of u64 range: {timestamp_us}")
    if not (0 <= reserved <= 0xFFFFFFFF):
        raise InvalidFieldError(f"reserved out of u32 range: {reserved}")
    plen = len(payload)
    if plen > MAX_PAYLOAD_BYTES:
        raise PayloadLengthError(f"payload_len {plen} exceeds MAX_PAYLOAD_BYTES ({MAX_PAYLOAD_BYTES})")

    hdr = FRAME_HEADER_STRUCT.pack(FRAME_SYNC, msg_type, flags, plen, seq, timestamp_us, reserved)
    body = hdr + payload
    crc = crc32_ieee(body)
    return body + FRAME_CRC_STRUCT.pack(crc)


def unpack_frame(buf: bytes | bytearray | memoryview, offset: int = 0) -> tuple[FrameHeader, bytes, int, int]:
    """Unpack and CRC-verify a single .hks frame at `offset`."""
    mv = memoryview(buf)
    rem = len(mv) - offset
    if rem < FRAME_OVERHEAD_BYTES:
        raise TruncatedFrameError(f"Need at least {FRAME_OVERHEAD_BYTES} bytes at offset {offset}, got {rem}")

    sync, msg_type, flags, payload_len, seq, timestamp_us, reserved = FRAME_HEADER_STRUCT.unpack_from(mv, offset)
    if sync != FRAME_SYNC:
        raise BadSyncError(f"Expected frame sync 0x{FRAME_SYNC:04X} at offset {offset}, got 0x{sync:04X}")
    if payload_len > MAX_PAYLOAD_BYTES:
        raise PayloadLengthError(f"Frame payload_len {payload_len} > {MAX_PAYLOAD_BYTES} at offset {offset}")

    total_len = FRAME_OVERHEAD_BYTES + payload_len
    if rem < total_len:
        raise TruncatedFrameError(
            f"Truncated frame at offset {offset}: expected {total_len} bytes, only {rem} available"
        )

    frame_end = offset + total_len
    crc_offset = offset + FRAME_HEADER_BYTES + payload_len
    expected_crc = crc32_ieee(mv[offset:crc_offset])
    (actual_crc,) = FRAME_CRC_STRUCT.unpack_from(mv, crc_offset)
    if expected_crc != actual_crc:
        raise BadCRCError(
            f"Frame CRC mismatch at offset {offset} (msg_type=0x{msg_type:02X}, seq={seq}): "
            f"expected 0x{expected_crc:08X}, got 0x{actual_crc:08X}"
        )

    payload = bytes(mv[offset + FRAME_HEADER_BYTES : crc_offset])
    hdr = FrameHeader(
        sync=sync,
        msg_type=msg_type,
        flags=flags,
        payload_len=payload_len,
        seq=seq,
        timestamp_us=timestamp_us,
        reserved=reserved,
    )
    return hdr, payload, actual_crc, frame_end


def pack_session_header_payload(hdr: SessionHeader) -> bytes:
    if hdr.magic != SESSION_MAGIC:
        raise BadMagicError(f"SessionHeader magic must be {SESSION_MAGIC!r}, got {hdr.magic!r}")
    if hdr.format_version != SESSION_FORMAT_VERSION:
        raise UnsupportedVersionError(f"Unsupported format_version {hdr.format_version}")
    if hdr.rink_preset not in RINK_PRESET_TO_ID:
        raise InvalidFieldError(f"Unknown rink_preset {hdr.rink_preset!r}")
    if hdr.mount_location not in MOUNT_TO_ID:
        raise InvalidFieldError(f"Unknown mount_location {hdr.mount_location!r}")
    if hdr.session_id < 1 or hdr.session_id >= (1 << 64):
        raise InvalidFieldError(f"Invalid session_id {hdr.session_id}")
    if hdr.device_id < 1 or hdr.device_id >= (1 << 64):
        raise InvalidFieldError(f"Invalid device_id {hdr.device_id}")
    if hdr.imu_odr_hz not in VALID_IMU_ODRS:
        raise InvalidFieldError(f"Invalid imu_odr_hz {hdr.imu_odr_hz}")
    if hdr.high_g_odr_hz not in VALID_HIGH_G_HEADER_ODRS:
        raise InvalidFieldError(f"Invalid high_g_odr_hz {hdr.high_g_odr_hz}")
    if hdr.mag_odr_hz not in VALID_MAG_ODRS:
        raise InvalidFieldError(f"Invalid mag_odr_hz {hdr.mag_odr_hz}")
    if hdr.baro_odr_hz not in VALID_BARO_ODRS:
        raise InvalidFieldError(f"Invalid baro_odr_hz {hdr.baro_odr_hz}")
    if not (0 <= hdr.player_height_cm <= 250):
        raise InvalidFieldError(f"player_height_cm out of range: {hdr.player_height_cm}")
    bx = _require_finite(hdr.bench_gate_xy_m[0], "bench_gate_xy_m[0]")
    by = _require_finite(hdr.bench_gate_xy_m[1], "bench_gate_xy_m[1]")
    s_a = _require_finite(hdr.accel_scale_mps2_per_lsb, "accel_scale_mps2_per_lsb")
    s_g = _require_finite(hdr.gyro_scale_rads_per_lsb, "gyro_scale_rads_per_lsb")
    s_hg = _require_finite(hdr.high_g_scale_mps2_per_lsb, "high_g_scale_mps2_per_lsb")
    s_m = _require_finite(hdr.mag_scale_ut_per_lsb, "mag_scale_ut_per_lsb")
    if s_a <= 0 or s_g <= 0 or s_hg <= 0 or s_m <= 0:
        raise InvalidFieldError("Sensor scales must be strictly positive")

    return SESSION_HEADER_STRUCT.pack(
        hdr.magic,
        hdr.format_version,
        hdr.hw_rev,
        hdr.fw_version,
        hdr.cal_crc32 & 0xFFFFFFFF,
        hdr.session_id,
        hdr.start_utc_ms,
        hdr.device_id,
        hdr.imu_odr_hz,
        hdr.high_g_odr_hz,
        hdr.mag_odr_hz,
        hdr.baro_odr_hz,
        s_a,
        s_g,
        s_hg,
        s_m,
        RINK_PRESET_TO_ID[hdr.rink_preset],
        MOUNT_TO_ID[hdr.mount_location],
        hdr.player_height_cm,
        bx,
        by,
        hdr.reserved & 0xFFFFFFFF,
    )


def unpack_session_header_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> SessionHeader:
    if len(payload) != SESSION_HEADER_STRUCT.size:
        raise PayloadLengthError(f"SESSION_HEADER payload must be {SESSION_HEADER_STRUCT.size} B, got {len(payload)}")
    (
        magic,
        format_version,
        hw_rev,
        fw_version,
        cal_crc32,
        session_id,
        start_utc_ms,
        device_id,
        imu_odr_hz,
        high_g_odr_hz,
        mag_odr_hz,
        baro_odr_hz,
        accel_scale,
        gyro_scale,
        high_g_scale,
        mag_scale,
        rink_id,
        mount_id,
        player_height_cm,
        bx,
        by,
        reserved,
    ) = SESSION_HEADER_STRUCT.unpack(payload)

    if magic != SESSION_MAGIC:
        raise BadMagicError(f"Invalid session magic {magic!r}, expected {SESSION_MAGIC!r}")
    if format_version != SESSION_FORMAT_VERSION:
        raise UnsupportedVersionError(f"Unsupported session format_version {format_version}")
    if hw_rev < 1 or fw_version < 1 or session_id < 1 or device_id < 1:
        raise InvalidFieldError("hw_rev, fw_version, session_id, and device_id must be >= 1")
    if imu_odr_hz not in VALID_IMU_ODRS:
        raise InvalidFieldError(f"Invalid imu_odr_hz {imu_odr_hz}")
    if high_g_odr_hz not in VALID_HIGH_G_HEADER_ODRS:
        raise InvalidFieldError(f"Invalid high_g_odr_hz {high_g_odr_hz}")
    if mag_odr_hz not in VALID_MAG_ODRS:
        raise InvalidFieldError(f"Invalid mag_odr_hz {mag_odr_hz}")
    if baro_odr_hz not in VALID_BARO_ODRS:
        raise InvalidFieldError(f"Invalid baro_odr_hz {baro_odr_hz}")
    if rink_id not in ID_TO_RINK_PRESET:
        raise InvalidFieldError(f"Unknown rink_preset ID {rink_id}")
    if mount_id not in ID_TO_MOUNT:
        raise InvalidFieldError(f"Unknown mount_location ID {mount_id}")
    if player_height_cm > 250:
        raise InvalidFieldError(f"Invalid player_height_cm {player_height_cm}")

    for name, val in (
        ("accel_scale", accel_scale),
        ("gyro_scale", gyro_scale),
        ("high_g_scale", high_g_scale),
        ("mag_scale", mag_scale),
    ):
        _require_finite(val, name)
        if val <= 0.0:
            raise InvalidFieldError(f"{name} must be > 0, got {val}")
    _require_finite(bx, "bench_gate_x_m")
    _require_finite(by, "bench_gate_y_m")
    if not (-35.0 <= bx <= 35.0 and -35.0 <= by <= 35.0):
        raise InvalidFieldError(f"bench_gate_xy_m out of [-35, 35] bounds: ({bx}, {by})")

    return SessionHeader(
        session_id=session_id,
        start_utc_ms=start_utc_ms,
        device_id=device_id,
        cal_crc32=cal_crc32,
        magic=magic,
        format_version=format_version,
        hw_rev=hw_rev,
        fw_version=fw_version,
        imu_odr_hz=imu_odr_hz,
        high_g_odr_hz=high_g_odr_hz,
        mag_odr_hz=mag_odr_hz,
        baro_odr_hz=baro_odr_hz,
        accel_scale_mps2_per_lsb=accel_scale,
        gyro_scale_rads_per_lsb=gyro_scale,
        high_g_scale_mps2_per_lsb=high_g_scale,
        mag_scale_ut_per_lsb=mag_scale,
        rink_preset=ID_TO_RINK_PRESET[rink_id],
        mount_location=ID_TO_MOUNT[mount_id],
        player_height_cm=player_height_cm,
        bench_gate_xy_m=(round(float(bx), 6), round(float(by), 6)),
        reserved=reserved,
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        crc32=frame_crc32,
    )


def pack_calibration_payload(cal: CalibrationBlock, *, recompute_crc: bool = True) -> bytes:
    if not (1 <= cal.cal_version <= 0xFFFF):
        raise InvalidFieldError(f"cal_version must be in [1, 65535], got {cal.cal_version}")
    if not (0 <= cal.flags <= 0xFFFF):
        raise InvalidFieldError(f"cal flags out of u16 range: {cal.flags}")
    if not (0 <= cal.cal_utc_ms < (1 << 64)):
        raise InvalidFieldError(f"cal_utc_ms out of u64 range: {cal.cal_utc_ms}")

    r_bs = _validate_mat3(cal.sensor_to_body_rotation_matrix, "sensor_to_body_rotation_matrix", check_so3=True)
    m_a = _validate_mat3(cal.accel_misalign_scale, "accel_misalign_scale")
    b_a = _validate_vec3(cal.accel_bias_mps2, "accel_bias_mps2")
    m_g = _validate_mat3(cal.gyro_misalign_scale, "gyro_misalign_scale")
    b_g = _validate_vec3(cal.gyro_bias_rads, "gyro_bias_rads")
    r_bs_hg = _validate_mat3(cal.high_g_to_body_rotation_matrix, "high_g_to_body_rotation_matrix", check_so3=True)
    m_hg = _validate_mat3(cal.high_g_misalign_scale, "high_g_misalign_scale")
    b_hg = _validate_vec3(cal.high_g_bias_mps2, "high_g_bias_mps2")
    h_m = _validate_vec3(cal.mag_hard_iron_ut, "mag_hard_iron_ut")
    s_m = _validate_mat3(cal.mag_soft_iron_matrix, "mag_soft_iron_matrix")
    b0 = _require_finite(cal.baseline_mag_norm_ut, "baseline_mag_norm_ut")
    dip = _require_finite(cal.baseline_mag_dip_deg, "baseline_mag_dip_deg")
    if not (10.0 <= b0 <= 150.0):
        raise InvalidFieldError(f"baseline_mag_norm_ut out of [10, 150]: {b0}")
    if not (-90.0 <= dip <= 90.0):
        raise InvalidFieldError(f"baseline_mag_dip_deg out of [-90, 90]: {dip}")

    body = CAL_BLOCK_BODY_STRUCT.pack(
        cal.cal_version,
        cal.flags,
        cal.cal_utc_ms,
        *_flatten_mat3(r_bs),
        *_flatten_mat3(m_a),
        *b_a,
        *_flatten_mat3(m_g),
        *b_g,
        *_flatten_mat3(r_bs_hg),
        *_flatten_mat3(m_hg),
        *b_hg,
        *h_m,
        *_flatten_mat3(s_m),
        b0,
        dip,
    )
    crc = crc32_ieee(body) if recompute_crc else (cal.cal_crc32 & 0xFFFFFFFF)
    return body + FRAME_CRC_STRUCT.pack(crc)


def unpack_calibration_payload(
    payload: bytes,
    *,
    verify_crc: bool = True,
    verify_so3: bool = True,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> CalibrationBlock:
    if len(payload) != CAL_BLOCK_STRUCT.size:
        raise PayloadLengthError(f"CALIBRATION_BLOCK payload must be {CAL_BLOCK_STRUCT.size} B, got {len(payload)}")

    unpacked = CAL_BLOCK_STRUCT.unpack(payload)
    cal_version = unpacked[0]
    flags = unpacked[1]
    cal_utc_ms = unpacked[2]
    idx = 3

    def take(n: int) -> tuple[float, ...]:
        nonlocal idx
        sl = unpacked[idx : idx + n]
        idx += n
        return sl  # type: ignore[return-value]

    r_bs = _unflatten_mat3(take(9))
    m_a = _unflatten_mat3(take(9))
    b_a = take(3)
    m_g = _unflatten_mat3(take(9))
    b_g = take(3)
    r_bs_hg = _unflatten_mat3(take(9))
    m_hg = _unflatten_mat3(take(9))
    b_hg = take(3)
    h_m = take(3)
    s_m = _unflatten_mat3(take(9))
    b0 = float(unpacked[idx])
    dip = float(unpacked[idx + 1])
    cal_crc32 = int(unpacked[idx + 2])

    if verify_crc:
        expected_crc = crc32_ieee(payload[:284])
        if cal_crc32 != expected_crc:
            raise CalibrationCRCError(
                f"Calibration block CRC32 mismatch: expected 0x{expected_crc:08X}, got 0x{cal_crc32:08X}"
            )

    if cal_version < 1:
        raise InvalidFieldError(f"cal_version must be >= 1, got {cal_version}")
    r_bs = _validate_mat3(r_bs, "sensor_to_body_rotation_matrix", check_so3=verify_so3)
    m_a = _validate_mat3(m_a, "accel_misalign_scale")
    b_a = _validate_vec3(b_a, "accel_bias_mps2")
    m_g = _validate_mat3(m_g, "gyro_misalign_scale")
    b_g = _validate_vec3(b_g, "gyro_bias_rads")
    r_bs_hg = _validate_mat3(r_bs_hg, "high_g_to_body_rotation_matrix", check_so3=verify_so3)
    m_hg = _validate_mat3(m_hg, "high_g_misalign_scale")
    b_hg = _validate_vec3(b_hg, "high_g_bias_mps2")
    h_m = _validate_vec3(h_m, "mag_hard_iron_ut")
    s_m = _validate_mat3(s_m, "mag_soft_iron_matrix")
    b0 = _require_finite(b0, "baseline_mag_norm_ut")
    dip = _require_finite(dip, "baseline_mag_dip_deg")
    if not (10.0 <= b0 <= 150.0):
        raise InvalidFieldError(f"baseline_mag_norm_ut out of [10, 150]: {b0}")
    if not (-90.0 <= dip <= 90.0):
        raise InvalidFieldError(f"baseline_mag_dip_deg out of [-90, 90]: {dip}")

    return CalibrationBlock(
        cal_version=cal_version,
        flags=flags,
        cal_utc_ms=cal_utc_ms,
        sensor_to_body_rotation_matrix=r_bs,
        accel_misalign_scale=m_a,
        accel_bias_mps2=b_a,  # type: ignore[arg-type]
        gyro_misalign_scale=m_g,
        gyro_bias_rads=b_g,  # type: ignore[arg-type]
        high_g_to_body_rotation_matrix=r_bs_hg,
        high_g_misalign_scale=m_hg,
        high_g_bias_mps2=b_hg,  # type: ignore[arg-type]
        mag_hard_iron_ut=h_m,  # type: ignore[arg-type]
        mag_soft_iron_matrix=s_m,
        baseline_mag_norm_ut=b0,
        baseline_mag_dip_deg=dip,
        cal_crc32=cal_crc32,
        frame_flags=frame_hdr.flags if frame_hdr else 0,
        frame_seq=frame_hdr.seq if frame_hdr else 0,
        frame_timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        frame_crc32=frame_crc32,
    )


def pack_imu_batch_payload(pkt: ImuBatchPacket) -> bytes:
    n = len(pkt.samples)
    if not (1 <= n <= IMU_MAX_SAMPLES):
        raise PayloadLengthError(f"IMU_BATCH sample_count must be in [1, {IMU_MAX_SAMPLES}], got {n}")
    if pkt.odr_hz not in VALID_IMU_ODRS:
        raise InvalidFieldError(f"Invalid IMU_BATCH odr_hz {pkt.odr_hz}")
    if not (-4000 <= pkt.temp_c_x100 <= 8500):
        raise InvalidFieldError(f"IMU temp_c_x100 out of [-4000, 8500]: {pkt.temp_c_x100}")

    buf = bytearray(IMU_BATCH_HDR_STRUCT.size + n * IMU_SAMPLE_STRUCT.size)
    IMU_BATCH_HDR_STRUCT.pack_into(buf, 0, n, pkt.odr_hz, pkt.temp_c_x100, pkt.reserved & 0xFFFF)
    off = IMU_BATCH_HDR_STRUCT.size
    for s in pkt.samples:
        IMU_SAMPLE_STRUCT.pack_into(buf, off, s.ax_i16, s.ay_i16, s.az_i16, s.gx_i16, s.gy_i16, s.gz_i16)
        off += IMU_SAMPLE_STRUCT.size
    return bytes(buf)


def unpack_imu_batch_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> ImuBatchPacket:
    if len(payload) < IMU_BATCH_HDR_STRUCT.size:
        raise PayloadLengthError(f"IMU_BATCH payload too short: {len(payload)} B")
    sample_count, odr_hz, temp_c_x100, reserved = IMU_BATCH_HDR_STRUCT.unpack_from(payload, 0)
    if not (1 <= sample_count <= IMU_MAX_SAMPLES):
        raise InvalidFieldError(f"IMU_BATCH sample_count out of range [1, {IMU_MAX_SAMPLES}]: {sample_count}")
    if odr_hz not in VALID_IMU_ODRS:
        raise InvalidFieldError(f"Invalid IMU_BATCH odr_hz {odr_hz}")
    if not (-4000 <= temp_c_x100 <= 8500):
        raise InvalidFieldError(f"IMU_BATCH temp_c_x100 out of [-4000, 8500]: {temp_c_x100}")

    expected_len = IMU_BATCH_HDR_STRUCT.size + sample_count * IMU_SAMPLE_STRUCT.size
    if len(payload) != expected_len:
        raise PayloadLengthError(f"IMU_BATCH payload length {len(payload)} != expected {expected_len}")

    raw_samples = payload[IMU_BATCH_HDR_STRUCT.size :]
    samples = tuple(
        ImuSample(*vals)
        for vals in IMU_SAMPLE_STRUCT.iter_unpack(raw_samples)
    )
    return ImuBatchPacket(
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        odr_hz=odr_hz,
        temp_c_x100=temp_c_x100,
        samples=samples,
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        reserved=reserved,
        crc32=frame_crc32,
        _raw_bytes=raw_samples,
    )


def pack_high_g_batch_payload(pkt: HighGBatchPacket) -> bytes:
    n = len(pkt.samples)
    if not (1 <= n <= HIGH_G_MAX_SAMPLES):
        raise PayloadLengthError(f"HIGH_G_BATCH sample_count must be in [1, {HIGH_G_MAX_SAMPLES}], got {n}")
    if pkt.odr_hz not in VALID_HIGH_G_BATCH_ODRS:
        raise InvalidFieldError(f"Invalid HIGH_G_BATCH odr_hz {pkt.odr_hz}")
    if pkt.trigger_mode not in TRIGGER_MODE_TO_ID:
        raise InvalidFieldError(f"Invalid trigger_mode {pkt.trigger_mode!r}")

    buf = bytearray(HIGH_G_BATCH_HDR_STRUCT.size + n * HIGH_G_SAMPLE_STRUCT.size)
    HIGH_G_BATCH_HDR_STRUCT.pack_into(
        buf,
        0,
        n,
        pkt.odr_hz,
        TRIGGER_MODE_TO_ID[pkt.trigger_mode],
        pkt.reserved0 & 0xFF,
        pkt.reserved1 & 0xFFFF,
    )
    off = HIGH_G_BATCH_HDR_STRUCT.size
    for s in pkt.samples:
        for axis_name, axis_val in (("ax_i16", s.ax_i16), ("ay_i16", s.ay_i16), ("az_i16", s.az_i16)):
            if not (-2048 <= axis_val <= 2047):
                raise InvalidFieldError(f"ADXL372 {axis_name}={axis_val} outside 12-bit signed [-2048, +2047]")
        HIGH_G_SAMPLE_STRUCT.pack_into(buf, off, s.ax_i16, s.ay_i16, s.az_i16)
        off += HIGH_G_SAMPLE_STRUCT.size
    return bytes(buf)


def unpack_high_g_batch_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> HighGBatchPacket:
    if len(payload) < HIGH_G_BATCH_HDR_STRUCT.size:
        raise PayloadLengthError(f"HIGH_G_BATCH payload too short: {len(payload)} B")
    sample_count, odr_hz, trigger_id, reserved0, reserved1 = HIGH_G_BATCH_HDR_STRUCT.unpack_from(payload, 0)
    if not (1 <= sample_count <= HIGH_G_MAX_SAMPLES):
        raise InvalidFieldError(f"HIGH_G_BATCH sample_count out of [1, {HIGH_G_MAX_SAMPLES}]: {sample_count}")
    if odr_hz not in VALID_HIGH_G_BATCH_ODRS:
        raise InvalidFieldError(f"Invalid HIGH_G_BATCH odr_hz {odr_hz}")
    if trigger_id not in ID_TO_TRIGGER_MODE:
        raise InvalidFieldError(f"Invalid HIGH_G_BATCH trigger_mode ID {trigger_id}")

    expected_len = HIGH_G_BATCH_HDR_STRUCT.size + sample_count * HIGH_G_SAMPLE_STRUCT.size
    if len(payload) != expected_len:
        raise PayloadLengthError(f"HIGH_G_BATCH payload length {len(payload)} != expected {expected_len}")

    samples_list: list[HighGSample] = []
    for ax, ay, az in HIGH_G_SAMPLE_STRUCT.iter_unpack(payload[HIGH_G_BATCH_HDR_STRUCT.size :]):
        if not (-2048 <= ax <= 2047 and -2048 <= ay <= 2047 and -2048 <= az <= 2047):
            raise InvalidFieldError(
                f"ADXL372 sample ({ax}, {ay}, {az}) outside right-aligned 12-bit [-2048, +2047] range "
                "(check firmware >> 4 unpacking)"
            )
        samples_list.append(HighGSample(ax, ay, az))

    return HighGBatchPacket(
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        odr_hz=odr_hz,
        trigger_mode=ID_TO_TRIGGER_MODE[trigger_id],
        samples=tuple(samples_list),
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        reserved0=reserved0,
        reserved1=reserved1,
        crc32=frame_crc32,
    )


def pack_aux_env_payload(pkt: AuxEnvPacket) -> bytes:
    for name, val in (("mx_i32", pkt.mx_i32), ("my_i32", pkt.my_i32), ("mz_i32", pkt.mz_i32)):
        if not (-131072 <= val <= 131071):
            raise InvalidFieldError(f"AUX_ENV {name}={val} outside [-131072, 131071]")
    if not (1920000 <= pkt.pressure_pa_x64 <= 8000000):
        raise InvalidFieldError(f"AUX_ENV pressure_pa_x64={pkt.pressure_pa_x64} outside [1920000, 8000000]")
    if not (-4000 <= pkt.baro_temp_c_x100 <= 8500):
        raise InvalidFieldError(f"AUX_ENV baro_temp_c_x100={pkt.baro_temp_c_x100} outside [-4000, 8500]")
    if not (2500 <= pkt.vbatt_mv <= 4400):
        raise InvalidFieldError(f"AUX_ENV vbatt_mv={pkt.vbatt_mv} outside [2500, 4400]")
    if not (-4000 <= pkt.batt_temp_c_x100 <= 8500):
        raise InvalidFieldError(f"AUX_ENV batt_temp_c_x100={pkt.batt_temp_c_x100} outside [-4000, 8500]")
    if not (0 <= pkt.soc_pct_x10 <= 1000):
        raise InvalidFieldError(f"AUX_ENV soc_pct_x10={pkt.soc_pct_x10} outside [0, 1000]")

    return AUX_ENV_STRUCT.pack(
        pkt.mx_i32,
        pkt.my_i32,
        pkt.mz_i32,
        pkt.pressure_pa_x64,
        pkt.baro_temp_c_x100,
        pkt.vbatt_mv,
        pkt.batt_temp_c_x100,
        pkt.soc_pct_x10,
        pkt.pmic_status & 0xFFFF,
        pkt.validity_flags & 0xFFFF,
    )


def unpack_aux_env_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> AuxEnvPacket:
    if len(payload) != AUX_ENV_STRUCT.size:
        raise PayloadLengthError(f"AUX_ENV payload must be {AUX_ENV_STRUCT.size} B, got {len(payload)}")
    mx, my, mz, p_x64, b_temp, vbatt, batt_temp, soc, pmic_st, val_flags = AUX_ENV_STRUCT.unpack(payload)
    for name, val in (("mx_i32", mx), ("my_i32", my), ("mz_i32", mz)):
        if not (-131072 <= val <= 131071):
            raise InvalidFieldError(f"AUX_ENV {name}={val} outside [-131072, 131071]")
    if not (1920000 <= p_x64 <= 8000000):
        raise InvalidFieldError(f"AUX_ENV pressure_pa_x64={p_x64} outside [1920000, 8000000]")
    if not (-4000 <= b_temp <= 8500):
        raise InvalidFieldError(f"AUX_ENV baro_temp_c_x100={b_temp} outside [-4000, 8500]")
    if not (2500 <= vbatt <= 4400):
        raise InvalidFieldError(f"AUX_ENV vbatt_mv={vbatt} outside [2500, 4400]")
    if not (-4000 <= batt_temp <= 8500):
        raise InvalidFieldError(f"AUX_ENV batt_temp_c_x100={batt_temp} outside [-4000, 8500]")
    if not (0 <= soc <= 1000):
        raise InvalidFieldError(f"AUX_ENV soc_pct_x10={soc} outside [0, 1000]")

    return AuxEnvPacket(
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        mx_i32=mx,
        my_i32=my,
        mz_i32=mz,
        pressure_pa_x64=p_x64,
        baro_temp_c_x100=b_temp,
        vbatt_mv=vbatt,
        batt_temp_c_x100=batt_temp,
        soc_pct_x10=soc,
        pmic_status=pmic_st,
        validity_flags=val_flags,
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        crc32=frame_crc32,
    )


def pack_event_marker_payload(pkt: EventMarkerPacket) -> bytes:
    if pkt.event_code not in EVENT_CODE_TO_ID:
        raise InvalidFieldError(f"Unknown event_code {pkt.event_code!r}")
    if not (0 <= pkt.severity <= 3):
        raise InvalidFieldError(f"Event severity out of [0, 3]: {pkt.severity}")
    val = _require_finite(pkt.value_f32, "value_f32")
    if not (0 <= pkt.detail_u64 < (1 << 64)):
        raise InvalidFieldError(f"detail_u64 out of u64 range: {pkt.detail_u64}")
    return EVENT_MARKER_STRUCT.pack(EVENT_CODE_TO_ID[pkt.event_code], pkt.severity, val, pkt.detail_u64)


def unpack_event_marker_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> EventMarkerPacket:
    if len(payload) != EVENT_MARKER_STRUCT.size:
        raise PayloadLengthError(f"EVENT_MARKER payload must be {EVENT_MARKER_STRUCT.size} B, got {len(payload)}")
    code_id, severity, val_f32, detail_u64 = EVENT_MARKER_STRUCT.unpack(payload)
    if code_id not in ID_TO_EVENT_CODE:
        raise InvalidFieldError(f"Unknown event_code ID {code_id}")
    if not (0 <= severity <= 3):
        raise InvalidFieldError(f"Invalid event severity {severity}")
    _require_finite(val_f32, "value_f32")
    return EventMarkerPacket(
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        event_code=ID_TO_EVENT_CODE[code_id],
        severity=severity,
        value_f32=val_f32,
        detail_u64=detail_u64,
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        crc32=frame_crc32,
    )


def pack_session_footer_payload(ftr: SessionFooter) -> bytes:
    if ftr.end_reason not in END_REASON_TO_ID:
        raise InvalidFieldError(f"Unknown end_reason {ftr.end_reason!r}")
    sha_hex = ftr.pre_footer_sha256.strip().lower()
    if len(sha_hex) != 64:
        raise InvalidFieldError(f"pre_footer_sha256 must be 64 hex chars, got len {len(sha_hex)}")
    try:
        sha_bytes = bytes.fromhex(sha_hex)
    except ValueError as exc:
        raise InvalidFieldError(f"Invalid pre_footer_sha256 hex: {exc}") from exc

    return SESSION_FOOTER_STRUCT.pack(
        ftr.end_utc_ms,
        ftr.total_imu_samples,
        ftr.dropped_imu_samples,
        ftr.total_high_g_samples,
        ftr.dropped_high_g_samples,
        ftr.total_aux_samples,
        END_REASON_TO_ID[ftr.end_reason],
        ftr.total_bytes,
        sha_bytes,
    )


def unpack_session_footer_payload(
    payload: bytes,
    *,
    frame_hdr: FrameHeader | None = None,
    frame_crc32: int = 0,
) -> SessionFooter:
    if len(payload) != SESSION_FOOTER_STRUCT.size:
        raise PayloadLengthError(f"SESSION_FOOTER payload must be {SESSION_FOOTER_STRUCT.size} B, got {len(payload)}")
    (
        end_utc_ms,
        total_imu,
        dropped_imu,
        total_hg,
        dropped_hg,
        total_aux,
        end_reason_id,
        total_bytes,
        sha_bytes,
    ) = SESSION_FOOTER_STRUCT.unpack(payload)
    if end_reason_id not in ID_TO_END_REASON:
        raise InvalidFieldError(f"Unknown end_reason ID {end_reason_id}")

    return SessionFooter(
        timestamp_us=frame_hdr.timestamp_us if frame_hdr else 0,
        end_utc_ms=end_utc_ms,
        total_imu_samples=total_imu,
        dropped_imu_samples=dropped_imu,
        total_high_g_samples=total_hg,
        dropped_high_g_samples=dropped_hg,
        total_aux_samples=total_aux,
        end_reason=ID_TO_END_REASON[end_reason_id],
        total_bytes=total_bytes,
        pre_footer_sha256=sha_bytes.hex(),
        flags=frame_hdr.flags if frame_hdr else 0,
        seq=frame_hdr.seq if frame_hdr else 0,
        crc32=frame_crc32,
    )


# ---------------------------------------------------------------------------
# High-Level Session Encoder / Decoder
# ---------------------------------------------------------------------------


def encode_hks_session(
    header: SessionHeader,
    calibration: CalibrationBlock,
    body_frames: Sequence[ImuBatchPacket | HighGBatchPacket | AuxEnvPacket | EventMarkerPacket],
    *,
    end_utc_ms: int,
    end_reason: str = "BUTTON_STOP",
    dropped_imu_samples: int = 0,
    dropped_high_g_samples: int = 0,
    footer_timestamp_us: int | None = None,
    footer_seq: int | None = None,
    footer_flags: int | None = None,
) -> bytes:
    """Encode a complete .hks binary session with 0x01 header, 0x06 cal, body frames, and 0x07 footer."""
    cal_payload = pack_calibration_payload(calibration, recompute_crc=True)
    (cal_crc,) = FRAME_CRC_STRUCT.unpack_from(cal_payload, 284)

    hdr_with_cal = SessionHeader(
        session_id=header.session_id,
        start_utc_ms=header.start_utc_ms,
        device_id=header.device_id,
        cal_crc32=cal_crc,
        magic=header.magic,
        format_version=header.format_version,
        hw_rev=header.hw_rev,
        fw_version=header.fw_version,
        imu_odr_hz=header.imu_odr_hz,
        high_g_odr_hz=header.high_g_odr_hz,
        mag_odr_hz=header.mag_odr_hz,
        baro_odr_hz=header.baro_odr_hz,
        accel_scale_mps2_per_lsb=header.accel_scale_mps2_per_lsb,
        gyro_scale_rads_per_lsb=header.gyro_scale_rads_per_lsb,
        high_g_scale_mps2_per_lsb=header.high_g_scale_mps2_per_lsb,
        mag_scale_ut_per_lsb=header.mag_scale_ut_per_lsb,
        rink_preset=header.rink_preset,
        mount_location=header.mount_location,
        player_height_cm=header.player_height_cm,
        bench_gate_xy_m=header.bench_gate_xy_m,
        reserved=header.reserved,
        flags=header.flags,
        seq=header.seq,
        timestamp_us=header.timestamp_us,
    )
    hdr_payload = pack_session_header_payload(hdr_with_cal)

    cal_seq = (
        calibration.frame_seq
        if calibration.frame_seq != 1
        else ((hdr_with_cal.seq + 1) & 0xFFFF)
    ) & 0xFFFF

    chunks: list[bytes] = [
        pack_frame(
            MsgType.SESSION_HEADER,
            hdr_payload,
            seq=hdr_with_cal.seq & 0xFFFF,
            timestamp_us=hdr_with_cal.timestamp_us,
            flags=hdr_with_cal.flags,
        ),
        pack_frame(
            MsgType.CALIBRATION_BLOCK,
            cal_payload,
            seq=cal_seq,
            timestamp_us=calibration.frame_timestamp_us,
            flags=calibration.frame_flags if calibration.frame_flags else hdr_with_cal.flags,
        ),
    ]

    seq_cursor = (cal_seq + 1) & 0xFFFF
    last_ts_us = max(hdr_with_cal.timestamp_us, calibration.frame_timestamp_us)
    last_flags = hdr_with_cal.flags
    total_imu = 0
    total_hg = 0
    total_aux = 0

    for pkt in body_frames:
        if isinstance(pkt, ImuBatchPacket):
            mtype = MsgType.IMU_BATCH
            pl = pack_imu_batch_payload(pkt)
            total_imu += len(pkt.samples)
            if pkt.sample_count > 0 and pkt.odr_hz > 0:
                end_sample_ts = pkt.timestamp_us + int(round((pkt.sample_count - 1) * 1_000_000.0 / pkt.odr_hz))
            else:
                end_sample_ts = pkt.timestamp_us
            last_ts_us = max(last_ts_us, end_sample_ts)
        elif isinstance(pkt, HighGBatchPacket):
            mtype = MsgType.HIGH_G_BATCH
            pl = pack_high_g_batch_payload(pkt)
            total_hg += len(pkt.samples)
            if pkt.sample_count > 0 and pkt.odr_hz > 0:
                end_sample_ts = pkt.timestamp_us + int(round((pkt.sample_count - 1) * 1_000_000.0 / pkt.odr_hz))
            else:
                end_sample_ts = pkt.timestamp_us
            last_ts_us = max(last_ts_us, end_sample_ts)
        elif isinstance(pkt, AuxEnvPacket):
            mtype = MsgType.AUX_ENV
            pl = pack_aux_env_payload(pkt)
            total_aux += 1
            last_ts_us = max(last_ts_us, pkt.timestamp_us)
        elif isinstance(pkt, EventMarkerPacket):
            mtype = MsgType.EVENT_MARKER
            pl = pack_event_marker_payload(pkt)
            last_ts_us = max(last_ts_us, pkt.timestamp_us)
        else:
            raise TypeError(f"Unsupported body frame packet type: {type(pkt)!r}")

        pkt_seq = pkt.seq if pkt.seq != 0 else seq_cursor
        seq_cursor = (pkt_seq + 1) & 0xFFFF
        last_flags = pkt.flags
        chunks.append(
            pack_frame(
                mtype,
                pl,
                seq=pkt_seq & 0xFFFF,
                timestamp_us=pkt.timestamp_us,
                flags=pkt.flags,
            )
        )

    pre_footer = b"".join(chunks)
    pre_sha_hex = hashlib.sha256(pre_footer).hexdigest()
    total_bytes = len(pre_footer) + SESSION_FOOTER_FRAME_BYTES
    ftr_ts = footer_timestamp_us if footer_timestamp_us is not None else last_ts_us
    ftr_seq = (footer_seq if footer_seq is not None else seq_cursor) & 0xFFFF
    ftr_flg = footer_flags if footer_flags is not None else last_flags

    ftr = SessionFooter(
        timestamp_us=ftr_ts,
        end_utc_ms=end_utc_ms,
        total_imu_samples=total_imu,
        dropped_imu_samples=dropped_imu_samples,
        total_high_g_samples=total_hg,
        dropped_high_g_samples=dropped_high_g_samples,
        total_aux_samples=total_aux,
        end_reason=end_reason,
        total_bytes=total_bytes,
        pre_footer_sha256=pre_sha_hex,
        flags=ftr_flg,
        seq=ftr_seq,
    )
    ftr_payload = pack_session_footer_payload(ftr)
    ftr_frame = pack_frame(
        MsgType.SESSION_FOOTER,
        ftr_payload,
        seq=ftr_seq,
        timestamp_us=ftr_ts,
        flags=ftr_flg,
    )
    return pre_footer + ftr_frame


def _get_contract_validators(contract_path: Path | None = None) -> dict[str, Any]:
    from jsonschema import Draft202012Validator

    path = contract_path or (Path(__file__).resolve().parent.parent / "INTERFACE_CONTRACT.json")
    with path.open("r", encoding="utf-8") as f:
        contract = json.load(f)
    schemas = contract["schemas"]
    return {
        "session_header": Draft202012Validator(schemas["flash_session_header"]),
        "calibration_block": Draft202012Validator(schemas["calibration_block"]),
        "imu_batch": Draft202012Validator(schemas["imu_batch_packet"]),
        "high_g_batch": Draft202012Validator(schemas["high_g_batch_packet"]),
        "aux_env": Draft202012Validator(schemas["aux_env_packet"]),
        "event_marker": Draft202012Validator(schemas["event_marker_packet"]),
        "session_footer": Draft202012Validator(schemas["flash_session_footer"]),
    }


def decode_hks_session(
    raw_bytes: bytes | bytearray | memoryview,
    *,
    require_footer: bool = True,
    verify_so3: bool = True,
    validate_contract_schemas: bool = False,
    contract_path: Path | None = None,
) -> DecodedHksSession:
    """Decode and strictly verify a complete binary .hks session."""
    raw = bytes(raw_bytes)
    if len(raw) < FRAME_OVERHEAD_BYTES:
        raise TruncatedFrameError(f"Empty or truncated .hks stream ({len(raw)} bytes)")

    validators = _get_contract_validators(contract_path) if validate_contract_schemas else None

    offset = 0
    header: SessionHeader | None = None
    calibration: CalibrationBlock | None = None
    imu_batches: list[ImuBatchPacket] = []
    high_g_batches: list[HighGBatchPacket] = []
    aux_env_packets: list[AuxEnvPacket] = []
    event_markers: list[EventMarkerPacket] = []
    footer: SessionFooter | None = None
    frames: list[Any] = []

    seen_imu_samples = 0
    seen_hg_samples = 0
    seen_aux_samples = 0

    while offset < len(raw):
        frame_offset = offset
        fhdr, payload, fcrc, next_offset = unpack_frame(raw, offset)
        offset = next_offset

        if fhdr.msg_type == MsgType.SESSION_HEADER:
            if frame_offset != 0 or header is not None:
                raise CodecError("SESSION_HEADER (0x01) must appear exactly once at offset 0")
            pkt_sh = unpack_session_header_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
            if validators:
                validators["session_header"].validate(pkt_sh.to_contract_dict())
            header = pkt_sh
            frames.append(pkt_sh)
        else:
            if header is None:
                raise CodecError(f"First frame must be SESSION_HEADER (0x01), got 0x{fhdr.msg_type:02X}")

            if fhdr.msg_type == MsgType.CALIBRATION_BLOCK:
                pkt_cal = unpack_calibration_payload(
                    payload,
                    verify_crc=True,
                    verify_so3=verify_so3,
                    frame_hdr=fhdr,
                    frame_crc32=fcrc,
                )
                if pkt_cal.cal_crc32 != header.cal_crc32:
                    raise CalibrationMismatchError(
                        f"CalibrationBlock cal_crc32 (0x{pkt_cal.cal_crc32:08X}) != "
                        f"SessionHeader cal_crc32 (0x{header.cal_crc32:08X})"
                    )
                if validators:
                    validators["calibration_block"].validate(pkt_cal.to_contract_dict())
                calibration = pkt_cal
                frames.append(pkt_cal)

            elif fhdr.msg_type == MsgType.IMU_BATCH:
                pkt_imu = unpack_imu_batch_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
                if validators:
                    validators["imu_batch"].validate(pkt_imu.to_contract_dict())
                imu_batches.append(pkt_imu)
                seen_imu_samples += pkt_imu.sample_count
                frames.append(pkt_imu)

            elif fhdr.msg_type == MsgType.HIGH_G_BATCH:
                pkt_hg = unpack_high_g_batch_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
                if validators:
                    validators["high_g_batch"].validate(pkt_hg.to_contract_dict())
                high_g_batches.append(pkt_hg)
                seen_hg_samples += pkt_hg.sample_count
                frames.append(pkt_hg)

            elif fhdr.msg_type == MsgType.AUX_ENV:
                pkt_aux = unpack_aux_env_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
                if validators:
                    validators["aux_env"].validate(pkt_aux.to_contract_dict())
                aux_env_packets.append(pkt_aux)
                seen_aux_samples += 1
                frames.append(pkt_aux)

            elif fhdr.msg_type == MsgType.EVENT_MARKER:
                pkt_evt = unpack_event_marker_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
                if validators:
                    validators["event_marker"].validate(pkt_evt.to_contract_dict())
                event_markers.append(pkt_evt)
                frames.append(pkt_evt)

            elif fhdr.msg_type == MsgType.SESSION_FOOTER:
                if offset != len(raw):
                    raise FooterError(
                        f"SESSION_FOOTER (0x07) must be the final frame; {len(raw) - offset} trailing bytes found"
                    )
                pkt_ftr = unpack_session_footer_payload(payload, frame_hdr=fhdr, frame_crc32=fcrc)
                expected_pre_sha = hashlib.sha256(raw[:frame_offset]).hexdigest()
                if pkt_ftr.pre_footer_sha256.lower() != expected_pre_sha:
                    raise Sha256MismatchError(
                        f"SESSION_FOOTER pre_footer_sha256 mismatch: expected {expected_pre_sha}, "
                        f"got {pkt_ftr.pre_footer_sha256}"
                    )
                if pkt_ftr.total_bytes != len(raw):
                    raise FooterError(
                        f"SESSION_FOOTER total_bytes ({pkt_ftr.total_bytes}) != actual stream length ({len(raw)})"
                    )
                if pkt_ftr.total_imu_samples != seen_imu_samples:
                    raise FooterError(
                        f"SESSION_FOOTER total_imu_samples ({pkt_ftr.total_imu_samples}) != decoded ({seen_imu_samples})"
                    )
                if pkt_ftr.total_high_g_samples != seen_hg_samples:
                    raise FooterError(
                        f"SESSION_FOOTER total_high_g_samples ({pkt_ftr.total_high_g_samples}) != decoded ({seen_hg_samples})"
                    )
                if pkt_ftr.total_aux_samples != seen_aux_samples:
                    raise FooterError(
                        f"SESSION_FOOTER total_aux_samples ({pkt_ftr.total_aux_samples}) != decoded ({seen_aux_samples})"
                    )
                if validators:
                    validators["session_footer"].validate(pkt_ftr.to_contract_dict())
                footer = pkt_ftr
                frames.append(pkt_ftr)
            else:
                raise UnknownMessageTypeError(f"Unknown message type 0x{fhdr.msg_type:02X} at offset {frame_offset}")

    if header is None:
        raise CodecError("Missing SESSION_HEADER (0x01)")
    if require_footer and footer is None:
        raise FooterError("Missing required SESSION_FOOTER (0x07)")

    return DecodedHksSession(
        header=header,
        calibration=calibration,
        imu_batches=imu_batches,
        high_g_batches=high_g_batches,
        aux_env_packets=aux_env_packets,
        event_markers=event_markers,
        footer=footer,
        frames=frames,
        raw_size_bytes=len(raw),
        file_crc32=crc32_ieee(raw),
        file_sha256=hashlib.sha256(raw).digest(),
    )
