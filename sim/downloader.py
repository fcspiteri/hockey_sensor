"""USB CDC-ACM / BLE post-session download protocol client & simulated device transport."""

from __future__ import annotations

import base64
import hashlib
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Iterable, Protocol

from .flash_codec import (
    DEFAULT_FW_VERSION,
    DEFAULT_HW_REV,
    DecodedHksSession,
    crc32_ieee,
    decode_hks_session,
)

# ---------------------------------------------------------------------------
# Protocol Constants & Structs (INTERFACE_CONTRACT.json & hks_contract.h)
# ---------------------------------------------------------------------------

REQ_SYNC = 0xAA55
RSP_SYNC = 0x55AA

REQ_HEADER_STRUCT = struct.Struct("<HBBH")  # 6 B: sync, cmd_id, seq, payload_len
RSP_HEADER_STRUCT = struct.Struct("<HBBBBH")  # 8 B: sync, cmd_id, status, seq, reserved, payload_len
CRC_STRUCT = struct.Struct("<I")  # 4 B

MAX_CHUNK_BYTES = 4096
GET_HEADER_BYTES = 416  # 104 B (0x01) + 312 B (0x06)

LIST_HDR_STRUCT = struct.Struct("<HH")  # 4 B: count, reserved
LIST_ENTRY_STRUCT = struct.Struct("<QQIIII")  # 32 B
GET_HEADER_REQ_STRUCT = struct.Struct("<Q")  # 8 B
READ_CHUNK_REQ_STRUCT = struct.Struct("<QIH")  # 14 B
READ_CHUNK_RSP_HDR_STRUCT = struct.Struct("<IH")  # 6 B
ACK_VERIFY_REQ_44_STRUCT = struct.Struct("<QI32s")  # 44 B
ACK_VERIFY_REQ_12_STRUCT = struct.Struct("<QI")  # 12 B
ACK_VERIFY_RSP_STRUCT = struct.Struct("<BB")  # 2 B
DELETE_REQ_STRUCT = struct.Struct("<QB")  # 9 B
DELETE_RSP_STRUCT = struct.Struct("<I")  # 4 B
DEV_STATUS_RSP_STRUCT = struct.Struct("<IHHHBBIQ")  # 24 B
UTC_TIME_STRUCT = struct.Struct("<Q")  # 8 B


class CmdId(IntEnum):
    LIST_SESSIONS = 0x01
    GET_HEADER = 0x02
    READ_CHUNK = 0x03
    ACK_VERIFY = 0x04
    DELETE_SESSION = 0x05
    GET_DEVICE_STATUS = 0x06
    SET_UTC_TIME = 0x07


class StatusCode(IntEnum):
    OK = 0x00
    ERR_BAD_CRC = 0x01
    ERR_UNKNOWN_CMD = 0x02
    ERR_SESSION_NOT_FOUND = 0x03
    ERR_OFFSET_OUT_OF_RANGE = 0x04
    ERR_NOT_VERIFIED = 0x05
    ERR_FLASH_IO = 0x06
    ERR_BUSY_RECORDING = 0x07
    ERR_VERIFY_MISMATCH = 0x08
    ERR_INVALID_PARAM = 0x09


CMD_ID_TO_NAME: dict[int, str] = {c.value: c.name for c in CmdId}
STATUS_CODE_TO_NAME: dict[int, str] = {s.value: s.name for s in StatusCode}


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class DownloadError(RuntimeError):
    """Base error for download protocol or transport failures."""

    def __init__(self, message: str, *, status_code: StatusCode | None = None) -> None:
        super().__init__(message)
        self.status_code = status_code


class TransportError(DownloadError):
    """Wire-level transport, framing, CRC, or sequence error."""


class BadResponseFrameError(TransportError):
    """Malformed, truncated, or bad-sync response envelope."""


class BadResponseCRCError(TransportError):
    """Response envelope CRC32-IEEE verification failed."""


class SequenceMismatchError(TransportError):
    """Response seq does not match transmitted request seq."""


class DeviceCommandError(DownloadError):
    """Device rejected command with non-OK StatusCode."""

    def __init__(self, cmd_id: int, status_code: StatusCode, detail: str = "") -> None:
        cmd_name = CMD_ID_TO_NAME.get(cmd_id, f"0x{cmd_id:02X}")
        msg = f"{cmd_name} failed with {status_code.name} (0x{status_code.value:02X})"
        if detail:
            msg = f"{msg}: {detail}"
        super().__init__(msg, status_code=status_code)
        self.cmd_id = cmd_id
        self.detail = detail


# ---------------------------------------------------------------------------
# Data Records
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SessionListEntry:
    session_id: int
    start_utc_ms: int
    duration_ms: int
    byte_size: int
    sample_count: int
    flags: int


@dataclass(frozen=True)
class DeviceStatus:
    fw_version: int
    hw_rev: int
    vbatt_mv: int
    soc_pct_x10: int
    recording_state: int
    pmic_status: int
    flash_free_kb: int
    utc_ms: int


@dataclass(frozen=True)
class DownloadedSession:
    session_id: int
    header_416b: bytes
    raw_bytes: bytes
    file_crc32: int
    file_sha256: bytes
    verified_status: int
    hash_mode: int
    deleted_freed_bytes: int | None = None
    decoded: DecodedHksSession | None = None


# ---------------------------------------------------------------------------
# Wire Framing & Contract JSON Helpers
# ---------------------------------------------------------------------------


def encode_request_frame(cmd_id: int, seq: int, payload: bytes = b"") -> bytes:
    if not (0 <= cmd_id <= 0xFF):
        raise ValueError(f"cmd_id out of u8 range: {cmd_id}")
    if not (0 <= seq <= 0xFF):
        raise ValueError(f"seq out of u8 range: {seq}")
    if len(payload) > 0xFFFF:
        raise ValueError(f"payload too large: {len(payload)}")
    hdr = REQ_HEADER_STRUCT.pack(REQ_SYNC, cmd_id, seq, len(payload))
    body = hdr + payload
    return body + CRC_STRUCT.pack(crc32_ieee(body))


def decode_request_frame(raw: bytes | bytearray | memoryview) -> tuple[int, int, bytes, bool]:
    """Decode request frame returning (cmd_id, seq, payload, crc_ok)."""
    mv = memoryview(raw)
    if len(mv) < REQ_HEADER_STRUCT.size + CRC_STRUCT.size:
        raise BadResponseFrameError(f"Request frame too short: {len(mv)} B")
    sync, cmd_id, seq, payload_len = REQ_HEADER_STRUCT.unpack_from(mv, 0)
    if sync != REQ_SYNC:
        raise BadResponseFrameError(f"Invalid request sync 0x{sync:04X}")
    expected_total = REQ_HEADER_STRUCT.size + payload_len + CRC_STRUCT.size
    if len(mv) != expected_total:
        raise BadResponseFrameError(f"Request length {len(mv)} != expected {expected_total}")
    crc_off = REQ_HEADER_STRUCT.size + payload_len
    expected_crc = crc32_ieee(mv[:crc_off])
    (actual_crc,) = CRC_STRUCT.unpack_from(mv, crc_off)
    return cmd_id, seq, bytes(mv[REQ_HEADER_STRUCT.size : crc_off]), (expected_crc == actual_crc)


def encode_response_frame(
    cmd_id: int,
    status: int,
    seq: int,
    payload: bytes = b"",
    *,
    reserved: int = 0,
) -> bytes:
    hdr = RSP_HEADER_STRUCT.pack(RSP_SYNC, cmd_id & 0xFF, status & 0xFF, seq & 0xFF, reserved & 0xFF, len(payload))
    body = hdr + payload
    return body + CRC_STRUCT.pack(crc32_ieee(body))


def decode_response_frame(raw: bytes | bytearray | memoryview) -> tuple[int, StatusCode, int, bytes, int]:
    """Decode and verify response frame returning (cmd_id, status_code, seq, payload, crc32)."""
    mv = memoryview(raw)
    min_len = RSP_HEADER_STRUCT.size + CRC_STRUCT.size
    if len(mv) < min_len:
        raise BadResponseFrameError(f"Truncated response frame: {len(mv)} B < {min_len} B")
    sync, cmd_id, status_raw, seq, _reserved, payload_len = RSP_HEADER_STRUCT.unpack_from(mv, 0)
    if sync != RSP_SYNC:
        raise BadResponseFrameError(f"Invalid response sync 0x{sync:04X}, expected 0x{RSP_SYNC:04X}")
    expected_total = RSP_HEADER_STRUCT.size + payload_len + CRC_STRUCT.size
    if len(mv) != expected_total:
        raise BadResponseFrameError(f"Truncated/mismatched response length {len(mv)} != expected {expected_total}")
    crc_off = RSP_HEADER_STRUCT.size + payload_len
    expected_crc = crc32_ieee(mv[:crc_off])
    (actual_crc,) = CRC_STRUCT.unpack_from(mv, crc_off)
    if expected_crc != actual_crc:
        raise BadResponseCRCError(f"Response CRC32 mismatch: expected 0x{expected_crc:08X}, got 0x{actual_crc:08X}")
    try:
        status = StatusCode(status_raw)
    except ValueError as exc:
        raise BadResponseFrameError(f"Unknown response status code 0x{status_raw:02X}") from exc
    return cmd_id, status, seq, bytes(mv[RSP_HEADER_STRUCT.size : crc_off]), actual_crc


def to_contract_command_dict(
    cmd_id: int,
    seq: int,
    payload: dict[str, Any],
    crc32: int,
) -> dict[str, Any]:
    """Build a JSON dict strictly conforming to INTERFACE_CONTRACT.json schemas.download_command."""
    cid = CmdId(cmd_id)
    allowed_keys: dict[CmdId, tuple[str, ...]] = {
        CmdId.LIST_SESSIONS: (),
        CmdId.GET_HEADER: ("session_id",),
        CmdId.READ_CHUNK: ("session_id", "offset_bytes", "length_bytes"),
        CmdId.ACK_VERIFY: ("session_id", "computed_file_crc32", "computed_sha256"),
        CmdId.DELETE_SESSION: ("session_id", "force_flag"),
        CmdId.GET_DEVICE_STATUS: (),
        CmdId.SET_UTC_TIME: ("utc_ms",),
    }
    clean_payload = {k: payload[k] for k in allowed_keys[cid] if k in payload}
    return {
        "sync": REQ_SYNC,
        "command": cid.name,
        "cmd_id": int(cid.value),
        "seq": int(seq) & 0xFF,
        "payload": clean_payload,
        "crc32": int(crc32) & 0xFFFFFFFF,
    }


def to_contract_response_dict(
    cmd_id: int,
    status: StatusCode | int,
    seq: int,
    payload: dict[str, Any],
    crc32: int,
) -> dict[str, Any]:
    """Build a JSON dict strictly conforming to INTERFACE_CONTRACT.json schemas.download_response."""
    cid = CmdId(cmd_id)
    st = StatusCode(status)
    return {
        "sync": RSP_SYNC,
        "command": cid.name,
        "cmd_id": int(cid.value),
        "status": st.name,
        "status_code": int(st.value),
        "seq": int(seq) & 0xFF,
        "payload": dict(payload),
        "crc32": int(crc32) & 0xFFFFFFFF,
    }


# ---------------------------------------------------------------------------
# Transport Protocol & Implementations
# ---------------------------------------------------------------------------


class DeviceTransport(Protocol):
    """Raw framed byte-exchange transport to a hockey_sensor device."""

    def exchange(self, request_frame: bytes) -> bytes: ...


@dataclass
class _StoredSession:
    session_id: int
    start_utc_ms: int
    duration_ms: int
    byte_size: int
    sample_count: int
    flags: int
    raw_bytes: bytes
    header_416b: bytes
    file_crc32: int
    file_sha256: bytes
    verified: bool = False


class SimulatedDeviceTransport:
    """In-memory protocol simulator matching firmware/src/hks_protocol.c & hks_flash_log.c."""

    def __init__(
        self,
        sessions: dict[int, bytes] | Iterable[bytes] = (),
        *,
        fw_version: int = DEFAULT_FW_VERSION,
        hw_rev: int = DEFAULT_HW_REV,
        vbatt_mv: int = 3920,
        soc_pct_x10: int = 845,
        recording_state: int = 0,
        pmic_status: int = 0x40,  # HKS_PMIC_FG_VALID
        flash_total_kb: int = 127744,  # 998 * 128 KiB
        utc_ms: int = 1763852500000,
    ) -> None:
        self.fw_version = int(fw_version)
        self.hw_rev = int(hw_rev)
        self.vbatt_mv = int(vbatt_mv)
        self.soc_pct_x10 = int(soc_pct_x10)
        self.recording_state = int(recording_state)
        self.pmic_status = int(pmic_status) & 0xFF
        self.flash_total_kb = int(flash_total_kb)
        self.utc_ms = int(utc_ms)

        self._sessions: dict[int, _StoredSession] = {}
        self._fault_req_bad_crc = 0
        self._fault_rsp_bad_crc = 0
        self._fault_rsp_truncate = 0
        self._fault_rsp_bad_seq = 0
        self._fault_flash_io = 0

        if isinstance(sessions, dict):
            for raw in sessions.values():
                self.add_session(raw)
        else:
            for raw in sessions:
                self.add_session(raw)

    def add_session(self, raw_bytes: bytes) -> int:
        raw = bytes(raw_bytes)
        decoded = decode_hks_session(raw, require_footer=True, verify_so3=True)
        if len(raw) < GET_HEADER_BYTES:
            raise ValueError(f"Session smaller than 416 B header+cal block ({len(raw)} B)")

        first_ts = decoded.header.timestamp_us
        last_ts = decoded.footer.timestamp_us if decoded.footer else first_ts
        dur_ms = max(0, (last_ts - first_ts) // 1000)
        imu_samples = sum(b.sample_count for b in decoded.imu_batches)
        sess = _StoredSession(
            session_id=decoded.header.session_id,
            start_utc_ms=decoded.header.start_utc_ms,
            duration_ms=int(dur_ms),
            byte_size=len(raw),
            sample_count=int(imu_samples),
            flags=0,
            raw_bytes=raw,
            header_416b=raw[:GET_HEADER_BYTES],
            file_crc32=decoded.file_crc32,
            file_sha256=decoded.file_sha256,
            verified=False,
        )
        self._sessions[sess.session_id] = sess
        return sess.session_id

    @property
    def session_ids(self) -> list[int]:
        return list(self._sessions.keys())

    def is_verified(self, session_id: int) -> bool:
        return self._sessions[session_id].verified

    def set_recording_state(self, state: int) -> None:
        self.recording_state = int(state)

    def inject_request_bad_crc(self, times: int = 1) -> None:
        self._fault_req_bad_crc += times

    def corrupt_next_response_crc(self, times: int = 1) -> None:
        self._fault_rsp_bad_crc += times

    def truncate_next_response(self, times: int = 1) -> None:
        self._fault_rsp_truncate += times

    def corrupt_next_response_seq(self, times: int = 1) -> None:
        self._fault_rsp_bad_seq += times

    def inject_flash_io_error(self, times: int = 1) -> None:
        self._fault_flash_io += times

    def flash_free_kb(self) -> int:
        used_kb = sum((s.byte_size + 1023) // 1024 for s in self._sessions.values())
        return max(0, self.flash_total_kb - used_kb)

    def _apply_response_faults(self, rsp_frame: bytes) -> bytes:
        if self._fault_rsp_bad_seq > 0:
            self._fault_rsp_bad_seq -= 1
            buf = bytearray(rsp_frame)
            buf[4] = (buf[4] + 1) & 0xFF
            body = bytes(buf[:-4])
            buf[-4:] = CRC_STRUCT.pack(crc32_ieee(body))
            rsp_frame = bytes(buf)
        if self._fault_rsp_bad_crc > 0:
            self._fault_rsp_bad_crc -= 1
            buf = bytearray(rsp_frame)
            buf[-1] ^= 0xFF
            rsp_frame = bytes(buf)
        if self._fault_rsp_truncate > 0:
            self._fault_rsp_truncate -= 1
            rsp_frame = rsp_frame[: max(4, len(rsp_frame) - 5)]
        return rsp_frame

    def exchange(self, request_frame: bytes) -> bytes:
        cmd_id, seq, payload, crc_ok = decode_request_frame(request_frame)

        if self._fault_req_bad_crc > 0:
            self._fault_req_bad_crc -= 1
            crc_ok = False

        if not crc_ok:
            return self._apply_response_faults(
                encode_response_frame(cmd_id, StatusCode.ERR_BAD_CRC, seq)
            )

        if cmd_id < CmdId.LIST_SESSIONS or cmd_id > CmdId.SET_UTC_TIME:
            return self._apply_response_faults(
                encode_response_frame(cmd_id, StatusCode.ERR_UNKNOWN_CMD, seq)
            )

        if self._fault_flash_io > 0:
            self._fault_flash_io -= 1
            return self._apply_response_faults(
                encode_response_frame(cmd_id, StatusCode.ERR_FLASH_IO, seq)
            )

        if self.recording_state != 0 and cmd_id != CmdId.GET_DEVICE_STATUS:
            return self._apply_response_frame_safe(
                encode_response_frame(cmd_id, StatusCode.ERR_BUSY_RECORDING, seq)
            )

        rsp = self._dispatch(CmdId(cmd_id), seq, payload)
        return self._apply_response_faults(rsp)

    def _apply_response_frame_safe(self, rsp_frame: bytes) -> bytes:
        return self._apply_response_faults(rsp_frame)

    def _dispatch(self, cmd: CmdId, seq: int, payload: bytes) -> bytes:
        if cmd == CmdId.LIST_SESSIONS:
            if len(payload) != 0:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            entries = sorted(self._sessions.values(), key=lambda s: s.session_id)
            buf = bytearray(LIST_HDR_STRUCT.size + len(entries) * LIST_ENTRY_STRUCT.size)
            LIST_HDR_STRUCT.pack_into(buf, 0, len(entries), 0)
            off = LIST_HDR_STRUCT.size
            for s in entries:
                s_flags = s.flags | (0x02 if s.verified else 0x00) | 0x01
                LIST_ENTRY_STRUCT.pack_into(
                    buf, off, s.session_id, s.start_utc_ms, s.duration_ms, s.byte_size, s.sample_count, s_flags
                )
                off += LIST_ENTRY_STRUCT.size
            return encode_response_frame(cmd, StatusCode.OK, seq, bytes(buf))

        if cmd == CmdId.GET_HEADER:
            if len(payload) != GET_HEADER_REQ_STRUCT.size:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            (session_id,) = GET_HEADER_REQ_STRUCT.unpack(payload)
            if session_id == 0:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            sess = self._sessions.get(session_id)
            if sess is None:
                return encode_response_frame(cmd, StatusCode.ERR_SESSION_NOT_FOUND, seq)
            return encode_response_frame(cmd, StatusCode.OK, seq, sess.header_416b)

        if cmd == CmdId.READ_CHUNK:
            if len(payload) != READ_CHUNK_REQ_STRUCT.size:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            session_id, offset_bytes, length_bytes = READ_CHUNK_REQ_STRUCT.unpack(payload)
            if session_id == 0 or length_bytes == 0 or length_bytes > MAX_CHUNK_BYTES:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            sess = self._sessions.get(session_id)
            if sess is None:
                return encode_response_frame(cmd, StatusCode.ERR_SESSION_NOT_FOUND, seq)
            if offset_bytes >= sess.byte_size:
                return encode_response_frame(cmd, StatusCode.ERR_OFFSET_OUT_OF_RANGE, seq)
            chunk = sess.raw_bytes[offset_bytes : offset_bytes + length_bytes]
            rsp_pl = READ_CHUNK_RSP_HDR_STRUCT.pack(offset_bytes, len(chunk)) + chunk
            return encode_response_frame(cmd, StatusCode.OK, seq, rsp_pl)

        if cmd == CmdId.ACK_VERIFY:
            if len(payload) == ACK_VERIFY_REQ_44_STRUCT.size:
                session_id, comp_crc, comp_sha = ACK_VERIFY_REQ_44_STRUCT.unpack(payload)
                check_sha = True
            elif len(payload) == ACK_VERIFY_REQ_12_STRUCT.size:
                session_id, comp_crc = ACK_VERIFY_REQ_12_STRUCT.unpack(payload)
                comp_sha = b"\x00" * 32
                check_sha = False
            else:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            if session_id == 0:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            sess = self._sessions.get(session_id)
            if sess is None:
                return encode_response_frame(cmd, StatusCode.ERR_SESSION_NOT_FOUND, seq)

            hash_mode = 2 if check_sha else 1
            if comp_crc != sess.file_crc32 or (check_sha and comp_sha != sess.file_sha256):
                return encode_response_frame(
                    cmd,
                    StatusCode.ERR_VERIFY_MISMATCH,
                    seq,
                    ACK_VERIFY_RSP_STRUCT.pack(0, hash_mode),
                )
            sess.verified = True
            return encode_response_frame(
                cmd,
                StatusCode.OK,
                seq,
                ACK_VERIFY_RSP_STRUCT.pack(1, hash_mode),
            )

        if cmd == CmdId.DELETE_SESSION:
            if len(payload) != DELETE_REQ_STRUCT.size:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            session_id, force_flag = DELETE_REQ_STRUCT.unpack(payload)
            if session_id == 0 or force_flag not in (0, 1):
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            sess = self._sessions.get(session_id)
            if sess is None:
                return encode_response_frame(cmd, StatusCode.ERR_SESSION_NOT_FOUND, seq)
            if not sess.verified and force_flag == 0:
                return encode_response_frame(cmd, StatusCode.ERR_NOT_VERIFIED, seq)
            freed_blocks = max(1, (sess.byte_size + 131071) // 131072)
            freed_bytes = freed_blocks * 131072
            del self._sessions[session_id]
            return encode_response_frame(cmd, StatusCode.OK, seq, DELETE_RSP_STRUCT.pack(freed_bytes))

        if cmd == CmdId.GET_DEVICE_STATUS:
            if len(payload) != 0:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            rsp_pl = DEV_STATUS_RSP_STRUCT.pack(
                self.fw_version,
                self.hw_rev,
                self.vbatt_mv,
                self.soc_pct_x10,
                self.recording_state & 0xFF,
                self.pmic_status & 0xFF,
                self.flash_free_kb(),
                self.utc_ms,
            )
            return encode_response_frame(cmd, StatusCode.OK, seq, rsp_pl)

        if cmd == CmdId.SET_UTC_TIME:
            if len(payload) != UTC_TIME_STRUCT.size:
                return encode_response_frame(cmd, StatusCode.ERR_INVALID_PARAM, seq)
            (utc_ms,) = UTC_TIME_STRUCT.unpack(payload)
            self.utc_ms = utc_ms
            return encode_response_frame(cmd, StatusCode.OK, seq, UTC_TIME_STRUCT.pack(utc_ms))

        return encode_response_frame(cmd, StatusCode.ERR_UNKNOWN_CMD, seq)


class SerialCdcTransport:
    """USB CDC-ACM hardware transport using lazy-imported pyserial."""

    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 2.0) -> None:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required for SerialCdcTransport (install pyserial or use SimulatedDeviceTransport)"
            ) from exc
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout_s)

    def close(self) -> None:
        self._ser.close()

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError(f"Serial read timeout waiting for {n - len(buf)} more bytes")
            buf.extend(chunk)
        return bytes(buf)

    def exchange(self, request_frame: bytes) -> bytes:
        self._ser.write(request_frame)
        self._ser.flush()
        hdr = self._read_exact(RSP_HEADER_STRUCT.size)
        sync, _cmd, _st, _seq, _rsv, payload_len = RSP_HEADER_STRUCT.unpack(hdr)
        if sync != RSP_SYNC:
            raise BadResponseFrameError(f"Serial sync mismatch: 0x{sync:04X}")
        tail = self._read_exact(payload_len + CRC_STRUCT.size)
        return hdr + tail


# ---------------------------------------------------------------------------
# Host Download Client
# ---------------------------------------------------------------------------


class DownloadClient:
    """Host-side post-session download protocol client with retry & contract trace support."""

    def __init__(
        self,
        transport: DeviceTransport,
        *,
        max_retries: int = 3,
        record_trace: bool = False,
    ) -> None:
        self.transport = transport
        self.max_retries = max(0, int(max_retries))
        self.record_trace = bool(record_trace)
        self._seq = 0
        self.trace: list[dict[str, Any]] = []

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def _transact(
        self,
        cmd: CmdId,
        req_payload_bytes: bytes,
        req_json_payload: dict[str, Any],
        rsp_payload_formatter: Any,
    ) -> tuple[bytes, int]:
        last_exc: Exception | None = None
        for _attempt in range(self.max_retries + 1):
            seq = self._next_seq()
            req_frame = encode_request_frame(int(cmd), seq, req_payload_bytes)
            (req_crc,) = CRC_STRUCT.unpack_from(req_frame, len(req_frame) - 4)

            try:
                raw_rsp = self.transport.exchange(req_frame)
                rsp_cmd, status, rsp_seq, rsp_payload, rsp_crc = decode_response_frame(raw_rsp)
                if rsp_seq != seq:
                    raise SequenceMismatchError(f"Response seq {rsp_seq} != request seq {seq}")
                if rsp_cmd != int(cmd):
                    raise BadResponseFrameError(f"Response cmd_id {rsp_cmd} != request cmd_id {int(cmd)}")

                if self.record_trace:
                    rsp_json_pl = rsp_payload_formatter(status, rsp_payload)
                    self.trace.append(
                        {
                            "command": to_contract_command_dict(int(cmd), seq, req_json_payload, req_crc),
                            "response": to_contract_response_dict(int(cmd), status, rsp_seq, rsp_json_pl, rsp_crc),
                        }
                    )

                if status == StatusCode.ERR_BAD_CRC:
                    last_exc = DeviceCommandError(int(cmd), status, "device reported request CRC error")
                    continue
                if status != StatusCode.OK:
                    raise DeviceCommandError(int(cmd), status)
                return rsp_payload, rsp_crc
            except (BadResponseFrameError, BadResponseCRCError, SequenceMismatchError, TimeoutError) as exc:
                last_exc = exc
                continue

        assert last_exc is not None
        raise last_exc

    def list_sessions(self) -> list[SessionListEntry]:
        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            count, _rsv = LIST_HDR_STRUCT.unpack_from(pl, 0)
            sessions = []
            off = LIST_HDR_STRUCT.size
            for _ in range(count):
                sid, sutc, dur, bsz, scnt, flg = LIST_ENTRY_STRUCT.unpack_from(pl, off)
                off += LIST_ENTRY_STRUCT.size
                sessions.append(
                    {
                        "session_id": sid,
                        "start_utc_ms": sutc,
                        "duration_ms": dur,
                        "byte_size": bsz,
                        "sample_count": scnt,
                        "flags": flg,
                    }
                )
            return {"sessions": sessions}

        pl, _ = self._transact(CmdId.LIST_SESSIONS, b"", {}, fmt)
        if len(pl) < LIST_HDR_STRUCT.size:
            raise BadResponseFrameError(f"LIST_SESSIONS payload too short: {len(pl)} B")
        count, _rsv = LIST_HDR_STRUCT.unpack_from(pl, 0)
        expected = LIST_HDR_STRUCT.size + count * LIST_ENTRY_STRUCT.size
        if len(pl) != expected:
            raise BadResponseFrameError(f"LIST_SESSIONS payload len {len(pl)} != {expected}")
        entries: list[SessionListEntry] = []
        off = LIST_HDR_STRUCT.size
        for _ in range(count):
            sid, sutc, dur, bsz, scnt, flg = LIST_ENTRY_STRUCT.unpack_from(pl, off)
            off += LIST_ENTRY_STRUCT.size
            entries.append(SessionListEntry(sid, sutc, dur, bsz, scnt, flg))
        return entries

    def get_header(self, session_id: int) -> bytes:
        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            return {"header_b64": base64.b64encode(pl).decode("ascii")}

        pl, _ = self._transact(
            CmdId.GET_HEADER,
            GET_HEADER_REQ_STRUCT.pack(session_id),
            {"session_id": int(session_id)},
            fmt,
        )
        if len(pl) != GET_HEADER_BYTES:
            raise BadResponseFrameError(f"GET_HEADER returned {len(pl)} B, expected {GET_HEADER_BYTES} B")
        return pl

    def read_chunk(self, session_id: int, offset_bytes: int, length_bytes: int = MAX_CHUNK_BYTES) -> tuple[int, bytes]:
        if not (1 <= length_bytes <= MAX_CHUNK_BYTES):
            raise ValueError(f"length_bytes must be in [1, {MAX_CHUNK_BYTES}], got {length_bytes}")

        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            off, alen = READ_CHUNK_RSP_HDR_STRUCT.unpack_from(pl, 0)
            data = pl[READ_CHUNK_RSP_HDR_STRUCT.size : READ_CHUNK_RSP_HDR_STRUCT.size + alen]
            return {
                "offset_bytes": off,
                "actual_len": alen,
                "data_b64": base64.b64encode(data).decode("ascii"),
            }

        pl, _ = self._transact(
            CmdId.READ_CHUNK,
            READ_CHUNK_REQ_STRUCT.pack(session_id, offset_bytes, length_bytes),
            {
                "session_id": int(session_id),
                "offset_bytes": int(offset_bytes),
                "length_bytes": int(length_bytes),
            },
            fmt,
        )
        if len(pl) < READ_CHUNK_RSP_HDR_STRUCT.size:
            raise BadResponseFrameError(f"READ_CHUNK response too short: {len(pl)} B")
        rsp_off, actual_len = READ_CHUNK_RSP_HDR_STRUCT.unpack_from(pl, 0)
        if rsp_off != offset_bytes:
            raise BadResponseFrameError(f"READ_CHUNK offset mismatch: {rsp_off} != {offset_bytes}")
        data = pl[READ_CHUNK_RSP_HDR_STRUCT.size :]
        if len(data) != actual_len:
            raise BadResponseFrameError(f"READ_CHUNK actual_len {actual_len} != data len {len(data)}")
        return rsp_off, data

    def ack_verify(
        self,
        session_id: int,
        computed_file_crc32: int,
        computed_sha256: bytes | str | None = None,
    ) -> tuple[int, int]:
        if computed_sha256 is None:
            req_bytes = ACK_VERIFY_REQ_12_STRUCT.pack(session_id, computed_file_crc32 & 0xFFFFFFFF)
            json_pl: dict[str, Any] = {
                "session_id": int(session_id),
                "computed_file_crc32": int(computed_file_crc32) & 0xFFFFFFFF,
            }
        else:
            sha_bytes = bytes.fromhex(computed_sha256) if isinstance(computed_sha256, str) else bytes(computed_sha256)
            if len(sha_bytes) != 32:
                raise ValueError(f"computed_sha256 must be 32 bytes, got {len(sha_bytes)}")
            req_bytes = ACK_VERIFY_REQ_44_STRUCT.pack(session_id, computed_file_crc32 & 0xFFFFFFFF, sha_bytes)
            json_pl = {
                "session_id": int(session_id),
                "computed_file_crc32": int(computed_file_crc32) & 0xFFFFFFFF,
                "computed_sha256": sha_bytes.hex(),
            }

        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if len(pl) == ACK_VERIFY_RSP_STRUCT.size:
                vs, hm = ACK_VERIFY_RSP_STRUCT.unpack(pl)
                return {"verified_status": vs, "hash_mode": hm}
            return {}

        pl, _ = self._transact(
            CmdId.ACK_VERIFY,
            req_bytes,
            json_pl,
            fmt,
        )
        if len(pl) != ACK_VERIFY_RSP_STRUCT.size:
            raise BadResponseFrameError(f"ACK_VERIFY payload len {len(pl)} != 2")
        verified_status, hash_mode = ACK_VERIFY_RSP_STRUCT.unpack(pl)
        return verified_status, hash_mode

    def delete_session(self, session_id: int, *, force: bool = False) -> int:
        force_flag = 1 if force else 0

        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            (freed,) = DELETE_RSP_STRUCT.unpack(pl)
            return {"freed_bytes": freed}

        pl, _ = self._transact(
            CmdId.DELETE_SESSION,
            DELETE_REQ_STRUCT.pack(session_id, force_flag),
            {"session_id": int(session_id), "force_flag": force_flag},
            fmt,
        )
        if len(pl) != DELETE_RSP_STRUCT.size:
            raise BadResponseFrameError(f"DELETE_SESSION payload len {len(pl)} != 4")
        (freed_bytes,) = DELETE_RSP_STRUCT.unpack(pl)
        return freed_bytes

    def get_device_status(self) -> DeviceStatus:
        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            fw, hw, vb, soc, rec, pmic, free_kb, utc = DEV_STATUS_RSP_STRUCT.unpack(pl)
            return {
                "fw_version": fw,
                "hw_rev": hw,
                "vbatt_mv": vb,
                "soc_pct_x10": soc,
                "recording_state": rec,
                "pmic_status": pmic,
                "flash_free_kb": free_kb,
                "utc_ms": utc,
            }

        pl, _ = self._transact(CmdId.GET_DEVICE_STATUS, b"", {}, fmt)
        if len(pl) != DEV_STATUS_RSP_STRUCT.size:
            raise BadResponseFrameError(
                f"GET_DEVICE_STATUS payload len {len(pl)} != {DEV_STATUS_RSP_STRUCT.size}"
            )
        fw, hw, vb, soc, rec, pmic, free_kb, utc = DEV_STATUS_RSP_STRUCT.unpack(pl)
        return DeviceStatus(
            fw_version=fw,
            hw_rev=hw,
            vbatt_mv=vb,
            soc_pct_x10=soc,
            recording_state=rec,
            pmic_status=pmic,
            flash_free_kb=free_kb,
            utc_ms=utc,
        )

    def set_utc_time(self, utc_ms: int) -> int:
        def fmt(status: StatusCode, pl: bytes) -> dict[str, Any]:
            if status != StatusCode.OK:
                return {}
            (echoed,) = UTC_TIME_STRUCT.unpack(pl)
            return {"utc_ms": echoed}

        pl, _ = self._transact(
            CmdId.SET_UTC_TIME,
            UTC_TIME_STRUCT.pack(utc_ms),
            {"utc_ms": int(utc_ms)},
            fmt,
        )
        if len(pl) != UTC_TIME_STRUCT.size:
            raise BadResponseFrameError(f"SET_UTC_TIME payload len {len(pl)} != 8")
        (echoed_utc_ms,) = UTC_TIME_STRUCT.unpack(pl)
        return echoed_utc_ms

    def download_session(
        self,
        session_id: int,
        *,
        chunk_size: int = MAX_CHUNK_BYTES,
        verify: bool = True,
        delete_after: bool = False,
        strict_decode: bool = True,
    ) -> DownloadedSession:
        if not (1 <= chunk_size <= MAX_CHUNK_BYTES):
            raise ValueError(f"chunk_size must be in [1, {MAX_CHUNK_BYTES}]")

        sessions = self.list_sessions()
        match = next((s for s in sessions if s.session_id == session_id), None)
        if match is None:
            raise DeviceCommandError(CmdId.LIST_SESSIONS, StatusCode.ERR_SESSION_NOT_FOUND, f"session {session_id}")

        header_416b = self.get_header(session_id)
        buf = bytearray()
        offset = 0
        while offset < match.byte_size:
            req_len = min(chunk_size, match.byte_size - offset)
            _off, chunk = self.read_chunk(session_id, offset, req_len)
            if not chunk:
                raise BadResponseFrameError(f"Unexpected empty READ_CHUNK at offset {offset}/{match.byte_size}")
            buf.extend(chunk)
            offset += len(chunk)

        raw = bytes(buf)
        if raw[:GET_HEADER_BYTES] != header_416b:
            raise BadResponseFrameError("Downloaded session prefix does not match GET_HEADER 416 B")

        file_crc = crc32_ieee(raw)
        file_sha = hashlib.sha256(raw).digest()
        decoded = decode_hks_session(raw, require_footer=True, verify_so3=True) if strict_decode else None

        verified_status = 0
        hash_mode = 0
        if verify:
            verified_status, hash_mode = self.ack_verify(session_id, file_crc, file_sha)

        freed_bytes: int | None = None
        if delete_after:
            freed_bytes = self.delete_session(session_id, force=False)

        return DownloadedSession(
            session_id=session_id,
            header_416b=header_416b,
            raw_bytes=raw,
            file_crc32=file_crc,
            file_sha256=file_sha,
            verified_status=verified_status,
            hash_mode=hash_mode,
            deleted_freed_bytes=freed_bytes,
            decoded=decoded,
        )
