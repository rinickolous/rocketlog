from __future__ import annotations

import struct
import time
import zlib
from dataclasses import dataclass
from typing import Final

# ---------------------------------------- #
# Constants                                 #
# ---------------------------------------- #

ROCKETLOG_MAGIC: Final[bytes] = b"RL"
ROCKETLOG_VERSION: Final[int] = 1

MSG_TELEMETRY: Final[int] = 1
MSG_LOG: Final[int] = 2
MSG_TIME_SYNC: Final[int] = 3
MSG_ACK: Final[int] = 4

LOG_DEBUG: Final[int] = 0
LOG_INFO: Final[int] = 1
LOG_WARN: Final[int] = 2
LOG_ERROR: Final[int] = 3

ACK_STATUS_OK: Final[int] = 0


# ---------------------------------------- #
# Types                                     #
# ---------------------------------------- #


@dataclass(frozen=True)
class ReceiverLog:
    """A log message received from the firmware."""

    t_unix_receiver: float | None  # firmware timestamp (None if clock not synced)
    t_unix_host: float             # host time when the frame arrived
    level: int
    msg: str


# ---------------------------------------- #
# CRC                                       #
# ---------------------------------------- #


def _crc32_le(data: bytes) -> int:
    """Standard CRC-32 matching esp_crc32_le()."""
    return zlib.crc32(data) & 0xFFFFFFFF


# ---------------------------------------- #
# COBS                                      #
# ---------------------------------------- #


def cobs_encode(data: bytes) -> bytes:
    """Encode bytes using COBS. Raises ValueError on empty input."""
    if not data:
        raise ValueError("COBS cannot encode empty payload")

    out = bytearray()
    code_index = 0
    out.append(0)  # placeholder for first code byte
    code = 1

    for b in data:
        if b == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(b)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode a COBS-encoded byte string. Raises ValueError on invalid input."""
    if not data:
        raise ValueError("COBS cannot decode empty payload")

    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            raise ValueError("Invalid COBS code byte (0x00 in payload)")
        idx += 1
        copy_len = code - 1
        if idx + copy_len > len(data):
            raise ValueError("COBS overrun")
        out.extend(data[idx : idx + copy_len])
        idx += copy_len
        if code != 0xFF and idx < len(data):
            out.append(0)

    return bytes(out)


# ---------------------------------------- #
# Packet encode / decode                    #
# ---------------------------------------- #


def encode_packet(msg_type: int, payload: bytes) -> bytes:
    """Encode a framed packet with header and CRC32."""
    if len(payload) > 255:
        raise ValueError("payload too large (max 255 bytes)")
    header = struct.pack("<2sBBB", ROCKETLOG_MAGIC, ROCKETLOG_VERSION, msg_type, len(payload))
    without_crc = header + payload
    crc = _crc32_le(without_crc)
    return without_crc + struct.pack("<I", crc)


def decode_packet(packet: bytes) -> tuple[int, bytes]:
    """Validate and unpack a raw packet. Returns (msg_type, payload)."""
    if len(packet) < 5 + 4:
        raise ValueError("packet too short")
    magic, version, msg_type, payload_len = struct.unpack("<2sBBB", packet[:5])
    if magic != ROCKETLOG_MAGIC:
        raise ValueError(f"bad magic: {magic!r}")
    if version != ROCKETLOG_VERSION:
        raise ValueError(f"bad version: {version}")
    expected_len = 5 + payload_len + 4
    if len(packet) != expected_len:
        raise ValueError(f"bad length: expected {expected_len}, got {len(packet)}")
    without_crc = packet[:-4]
    (got_crc,) = struct.unpack("<I", packet[-4:])
    if got_crc != _crc32_le(without_crc):
        raise ValueError("CRC mismatch")
    return msg_type, packet[5:-4]


# ---------------------------------------- #
# Payload helpers                           #
# ---------------------------------------- #


def decode_telemetry_payload(payload: bytes) -> tuple[float, float, float, float, float]:
    """Unpack a TELEMETRY payload. Returns (t_unix_s, alt_m, vel_mps, batt_v, temp_c)."""
    expected = 8 + 4 + 4 + 2 + 2
    if len(payload) != expected:
        raise ValueError(f"unexpected telemetry payload length: {len(payload)}")
    unix_time_us, altitude_cm, velocity_cms, battery_mv, temperature_cC = struct.unpack(
        "<qiiHh", payload
    )
    return (
        unix_time_us / 1_000_000.0,
        altitude_cm / 100.0,
        velocity_cms / 100.0,
        battery_mv / 1000.0,
        temperature_cC / 100.0,
    )


def decode_log_payload(payload: bytes) -> tuple[int, float | None, str]:
    """Unpack a LOG payload. Returns (level, t_unix_receiver_or_None, msg)."""
    if len(payload) < 1 + 8:
        raise ValueError("log payload too short")
    level = payload[0]
    (unix_time_us,) = struct.unpack("<q", payload[1:9])
    msg = payload[9:].decode("utf-8", errors="replace")
    t_unix_receiver = None if unix_time_us == 0 else unix_time_us / 1_000_000.0
    return level, t_unix_receiver, msg


def encode_time_sync(unix_time_s: float, seq: int) -> bytes:
    """Build a TIME_SYNC packet."""
    unix_time_us = int(round(unix_time_s * 1_000_000.0))
    payload = struct.pack("<qI", unix_time_us, seq & 0xFFFFFFFF)
    return encode_packet(MSG_TIME_SYNC, payload)


def decode_ack_payload(payload: bytes) -> tuple[int, int, int, float | None]:
    """Unpack an ACK payload. Returns (ack_type, seq, status, applied_unix_s_or_None)."""
    expected = 1 + 4 + 1 + 8
    if len(payload) != expected:
        raise ValueError(f"unexpected ack payload length: {len(payload)}")
    ack_type = payload[0]
    (seq,) = struct.unpack("<I", payload[1:5])
    status = payload[5]
    (applied_unix_time_us,) = struct.unpack("<q", payload[6:14])
    applied = None if applied_unix_time_us == 0 else applied_unix_time_us / 1_000_000.0
    return ack_type, seq, status, applied


# ---------------------------------------- #
# Utilities                                 #
# ---------------------------------------- #


def format_log(level: int, msg: str) -> str:
    """Format a log level + message into a human-readable string."""
    labels = {LOG_DEBUG: "DEBUG", LOG_INFO: "INFO", LOG_WARN: "WARN", LOG_ERROR: "ERROR"}
    return f"[{labels.get(level, str(level))}] {msg}"


def now_unix() -> float:
    """Return the current Unix time in seconds."""
    return time.time()
