"""hockey_sensor host simulation, binary codec, downloader, and anchor-free fusion package."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_ROOT.parent
DEFAULT_CONTRACT_PATH = REPO_ROOT / "INTERFACE_CONTRACT.json"


def _reject_non_standard_json_constant(constant: str) -> Any:
    raise ValueError(f"Non-standard JSON constant '{constant}' is forbidden (RFC 8259 strict JSON required)")


def load_contract(path: Path = DEFAULT_CONTRACT_PATH) -> dict[str, Any]:
    """Load INTERFACE_CONTRACT.json using strict RFC 8259 JSON parsing."""
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f, parse_constant=_reject_non_standard_json_constant)


from .flash_codec import (
    CANONICAL_R_BS,
    DEFAULT_ACCEL_SCALE,
    DEFAULT_GYRO_SCALE,
    DEFAULT_HIGH_G_SCALE,
    DEFAULT_MAG_SCALE,
    FRAME_HEADER_FMT,
    FRAME_OVERHEAD_BYTES,
    FRAME_SYNC,
    MAX_PAYLOAD_BYTES,
    SESSION_FORMAT_VERSION,
    SESSION_MAGIC,
    AuxEnvPacket,
    BadCRCError,
    BadMagicError,
    BadSyncError,
    CalibrationBlock,
    CalibrationCRCError,
    CalibrationMismatchError,
    CodecError,
    DecodedHksSession,
    EventMarkerPacket,
    FooterError,
    FrameFlag,
    FrameHeader,
    HighGBatchPacket,
    HighGSample,
    ImuBatchPacket,
    ImuSample,
    InvalidFieldError,
    MsgType,
    PayloadLengthError,
    SessionFooter,
    SessionHeader,
    Sha256MismatchError,
    TruncatedFrameError,
    UnknownMessageTypeError,
    UnsupportedVersionError,
    crc32_ieee,
    decode_hks_session,
    encode_hks_session,
    pack_frame,
    unpack_frame,
)
from .downloader import (
    MAX_CHUNK_BYTES,
    REQ_SYNC,
    RSP_SYNC,
    BadResponseCRCError,
    BadResponseFrameError,
    CmdId,
    DeviceCommandError,
    DeviceStatus,
    DeviceTransport,
    DownloadClient,
    DownloadError,
    DownloadedSession,
    SequenceMismatchError,
    SerialCdcTransport,
    SessionListEntry,
    SimulatedDeviceTransport,
    StatusCode,
    TransportError,
)

__all__ = [
    "PACKAGE_ROOT",
    "REPO_ROOT",
    "DEFAULT_CONTRACT_PATH",
    "load_contract",
    # flash_codec
    "CANONICAL_R_BS",
    "DEFAULT_ACCEL_SCALE",
    "DEFAULT_GYRO_SCALE",
    "DEFAULT_HIGH_G_SCALE",
    "DEFAULT_MAG_SCALE",
    "FRAME_HEADER_FMT",
    "FRAME_OVERHEAD_BYTES",
    "FRAME_SYNC",
    "MAX_PAYLOAD_BYTES",
    "SESSION_FORMAT_VERSION",
    "SESSION_MAGIC",
    "MsgType",
    "FrameFlag",
    "CodecError",
    "BadSyncError",
    "BadCRCError",
    "TruncatedFrameError",
    "PayloadLengthError",
    "UnknownMessageTypeError",
    "BadMagicError",
    "UnsupportedVersionError",
    "CalibrationCRCError",
    "CalibrationMismatchError",
    "Sha256MismatchError",
    "FooterError",
    "InvalidFieldError",
    "FrameHeader",
    "SessionHeader",
    "CalibrationBlock",
    "ImuSample",
    "ImuBatchPacket",
    "HighGSample",
    "HighGBatchPacket",
    "AuxEnvPacket",
    "EventMarkerPacket",
    "SessionFooter",
    "DecodedHksSession",
    "crc32_ieee",
    "pack_frame",
    "unpack_frame",
    "encode_hks_session",
    "decode_hks_session",
    # downloader
    "REQ_SYNC",
    "RSP_SYNC",
    "MAX_CHUNK_BYTES",
    "CmdId",
    "StatusCode",
    "DownloadError",
    "TransportError",
    "BadResponseFrameError",
    "BadResponseCRCError",
    "SequenceMismatchError",
    "DeviceCommandError",
    "SessionListEntry",
    "DeviceStatus",
    "DownloadedSession",
    "DeviceTransport",
    "SimulatedDeviceTransport",
    "SerialCdcTransport",
    "DownloadClient",
]
