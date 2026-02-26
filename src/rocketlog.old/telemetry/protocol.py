from __future__ import annotations

import struct
import time
import zlib
from dataclasses import dataclass
from typing import Final

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


@dataclass(frozen=True)
class ReceiverLog:
    t_unix_receiver: float | None
    t_unix_host: float
    level: int
    msg: str


# ---------------------------------------- #


def _crc32_le(data: bytes) -> int:
    # zlib.crc32 matches standard CRC-32; we treat stored bytes as little-endian.
    return zlib.crc32(data) & 0xFFFFFFFF


# ---------------------------------------- #


def cobs_encode(data: bytes) -> bytes:
    if not data:
        raise ValueError("COBS cannot encode empty payload")

    out = bytearray()
    code_index = 0
    out.append(0)  # placeholder for code
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


# ---------------------------------------- #


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ValueError("COBS cannot decode empty payload")

    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            raise ValueError("Invalid COBS code byte")
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


def encode_packet(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too large")

    header = struct.pack(
        "<2sBBB", ROCKETLOG_MAGIC, ROCKETLOG_VERSION, msg_type, len(payload)
    )
    without_crc = header + payload
    crc = _crc32_le(without_crc)
    return without_crc + struct.pack("<I", crc)


# ---------------------------------------- #


def decode_packet(packet: bytes) -> tuple[int, bytes]:
    if len(packet) < 5 + 4:
        raise ValueError("packet too short")

    magic, version, msg_type, payload_len = struct.unpack("<2sBBB", packet[:5])
    if magic != ROCKETLOG_MAGIC:
        raise ValueError("bad magic")
    if version != ROCKETLOG_VERSION:
        raise ValueError("bad version")

    expected_len = 5 + payload_len + 4
    if len(packet) != expected_len:
        raise ValueError("bad packet length")

    without_crc = packet[:-4]
    (got_crc,) = struct.unpack("<I", packet[-4:])
    expected_crc = _crc32_le(without_crc)
    if got_crc != expected_crc:
        raise ValueError("bad crc")

    return msg_type, packet[5:-4]


# ---------------------------------------- #


def decode_telemetry_payload(
    payload: bytes,
) -> tuple[float, float, float, float, float]:
    # Matches firmware telemetry_packet.c payload layout.
    if len(payload) != (8 + 4 + 4 + 2 + 2):
        raise ValueError("unexpected telemetry payload length")

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


# ---------------------------------------- #


def decode_log_payload(payload: bytes) -> tuple[int, float | None, str]:
    if len(payload) < 1 + 8:
        raise ValueError("unexpected log payload length")

    level = payload[0]
    (unix_time_us,) = struct.unpack("<q", payload[1:9])
    msg_bytes = payload[9:]
    msg = msg_bytes.decode("utf-8", errors="replace")
    t_unix_receiver = None if unix_time_us == 0 else unix_time_us / 1_000_000.0
    return level, t_unix_receiver, msg


# ---------------------------------------- #


def encode_time_sync(unix_time_s: float, seq: int) -> bytes:
    unix_time_us = int(round(unix_time_s * 1_000_000.0))
    payload = struct.pack("<qI", unix_time_us, seq & 0xFFFFFFFF)
    return encode_packet(MSG_TIME_SYNC, payload)


# ---------------------------------------- #


def decode_ack_payload(payload: bytes) -> tuple[int, int, int, float | None]:
    if len(payload) != (1 + 4 + 1 + 8):
        raise ValueError("unexpected ack payload length")

    ack_type = payload[0]
    (seq,) = struct.unpack("<I", payload[1:5])
    status = payload[5]
    (applied_unix_time_us,) = struct.unpack("<q", payload[6:14])
    applied = None if applied_unix_time_us == 0 else applied_unix_time_us / 1_000_000.0
    return ack_type, seq, status, applied


# ---------------------------------------- #


def format_log(level: int, msg: str) -> str:
    levels = {
        LOG_DEBUG: "DEBUG",
        LOG_INFO: "INFO",
        LOG_WARN: "WARN",
        LOG_ERROR: "ERROR",
    }
    return f"[{levels.get(level, str(level))}] {msg}"


# ---------------------------------------- #


def now_unix() -> float:
    return time.time()
