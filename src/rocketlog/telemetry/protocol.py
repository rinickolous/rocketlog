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
MSG_EVENT: Final[int] = 5

LOG_DEBUG: Final[int] = 0
LOG_INFO: Final[int] = 1
LOG_WARN: Final[int] = 2
LOG_ERROR: Final[int] = 3

ACK_STATUS_OK: Final[int] = 0

# ---------------------------------------- #
# Event codes                               #
# ---------------------------------------- #
#
# Mirrors event_codes.h. A 1-byte code + 4-byte signed param.
# Group by high nibble:
#   0x0x  System / lifecycle
#   0x1x  Flight phase
#   0x2x  Sensor / hardware
#   0x3x  Comms / radio
#   0x4x  Errors / faults

# 0x0x — System
EVT_SYS_BOOT:       Final[int] = 0x00
EVT_SYS_TIME_SYNC:  Final[int] = 0x01
EVT_SYS_REC_START:  Final[int] = 0x02
EVT_SYS_REC_STOP:   Final[int] = 0x03
EVT_SYS_WATCHDOG:   Final[int] = 0x04
EVT_SYS_LOW_BATT:   Final[int] = 0x05

# 0x1x — Flight phase
EVT_FLIGHT_PAD_HOLD: Final[int] = 0x10
EVT_FLIGHT_LIFTOFF:  Final[int] = 0x11
EVT_FLIGHT_BURNOUT:  Final[int] = 0x12
EVT_FLIGHT_APOGEE:   Final[int] = 0x13
EVT_FLIGHT_DROGUE:   Final[int] = 0x14
EVT_FLIGHT_MAIN:     Final[int] = 0x15
EVT_FLIGHT_LANDING:  Final[int] = 0x16

# 0x2x — Sensor / hardware
EVT_SENSOR_GPS_FIX:  Final[int] = 0x20
EVT_SENSOR_GPS_LOST: Final[int] = 0x21
EVT_SENSOR_BARO_OK:  Final[int] = 0x22
EVT_SENSOR_BARO_ERR: Final[int] = 0x23
EVT_SENSOR_IMU_OK:   Final[int] = 0x24
EVT_SENSOR_IMU_ERR:  Final[int] = 0x25

# 0x3x — Comms / radio
EVT_COMMS_LORA_INIT: Final[int] = 0x30
EVT_COMMS_LORA_TX:   Final[int] = 0x31
EVT_COMMS_LORA_RX:   Final[int] = 0x32
EVT_COMMS_LORA_ERR:  Final[int] = 0x33
EVT_COMMS_USB_CONN:  Final[int] = 0x34

# 0x4x — Errors / faults
EVT_ERR_PANIC:          Final[int] = 0x40
EVT_ERR_CRC:            Final[int] = 0x41
EVT_ERR_DECODE:         Final[int] = 0x42
EVT_ERR_ALLOC:          Final[int] = 0x43
EVT_ERR_STACK:          Final[int] = 0x44
EVT_ERR_SENSOR_TIMEOUT: Final[int] = 0x45

# Human-readable names for display.
_EVT_NAMES: dict[int, str] = {
    EVT_SYS_BOOT:           "SYS_BOOT",
    EVT_SYS_TIME_SYNC:      "SYS_TIME_SYNC",
    EVT_SYS_REC_START:      "SYS_REC_START",
    EVT_SYS_REC_STOP:       "SYS_REC_STOP",
    EVT_SYS_WATCHDOG:       "SYS_WATCHDOG",
    EVT_SYS_LOW_BATT:       "SYS_LOW_BATT",
    EVT_FLIGHT_PAD_HOLD:    "FLIGHT_PAD_HOLD",
    EVT_FLIGHT_LIFTOFF:     "FLIGHT_LIFTOFF",
    EVT_FLIGHT_BURNOUT:     "FLIGHT_BURNOUT",
    EVT_FLIGHT_APOGEE:      "FLIGHT_APOGEE",
    EVT_FLIGHT_DROGUE:      "FLIGHT_DROGUE",
    EVT_FLIGHT_MAIN:        "FLIGHT_MAIN",
    EVT_FLIGHT_LANDING:     "FLIGHT_LANDING",
    EVT_SENSOR_GPS_FIX:     "SENSOR_GPS_FIX",
    EVT_SENSOR_GPS_LOST:    "SENSOR_GPS_LOST",
    EVT_SENSOR_BARO_OK:     "SENSOR_BARO_OK",
    EVT_SENSOR_BARO_ERR:    "SENSOR_BARO_ERR",
    EVT_SENSOR_IMU_OK:      "SENSOR_IMU_OK",
    EVT_SENSOR_IMU_ERR:     "SENSOR_IMU_ERR",
    EVT_COMMS_LORA_INIT:    "COMMS_LORA_INIT",
    EVT_COMMS_LORA_TX:      "COMMS_LORA_TX",
    EVT_COMMS_LORA_RX:      "COMMS_LORA_RX",
    EVT_COMMS_LORA_ERR:     "COMMS_LORA_ERR",
    EVT_COMMS_USB_CONN:     "COMMS_USB_CONN",
    EVT_ERR_PANIC:          "ERR_PANIC",
    EVT_ERR_CRC:            "ERR_CRC",
    EVT_ERR_DECODE:         "ERR_DECODE",
    EVT_ERR_ALLOC:          "ERR_ALLOC",
    EVT_ERR_STACK:          "ERR_STACK",
    EVT_ERR_SENSOR_TIMEOUT: "ERR_SENSOR_TIMEOUT",
}


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

# v1 telemetry payload: unix_time_us(i64) + altitude_cm(i32) + velocity_cms(i32)
#                       + battery_mv(u16) + temperature_cC(i16) = 20 bytes
_TELEMETRY_V1_PAYLOAD_LEN = 20
_TELEMETRY_V1_FMT = "<qiiHh"

# v2 telemetry payload: v1 fields + latitude_1e7(i32) + longitude_1e7(i32)
#                       + gps_alt_cm(i32) + gps_sats(u8) + gps_fix(u8) = 34 bytes
_TELEMETRY_V2_PAYLOAD_LEN = 34
_TELEMETRY_V2_FMT = "<qiiHhiiiBB"


def decode_telemetry_payload(
    payload: bytes,
) -> tuple[float, float, float, float, float, float | None, float | None, float | None, int | None, bool | None]:
    """
    Unpack a TELEMETRY payload (v1 or v2).

    Returns:
        (t_unix_s, alt_m, vel_mps, batt_v, temp_c,
         lat, lon, gps_alt_m, gps_sats, gps_fix)

    GPS fields are None for v1 packets.
    """
    if len(payload) == _TELEMETRY_V1_PAYLOAD_LEN:
        unix_time_us, altitude_cm, velocity_cms, battery_mv, temperature_cC = struct.unpack(
            _TELEMETRY_V1_FMT, payload
        )
        return (
            unix_time_us / 1_000_000.0,
            altitude_cm / 100.0,
            velocity_cms / 100.0,
            battery_mv / 1000.0,
            temperature_cC / 100.0,
            None, None, None, None, None,
        )

    if len(payload) == _TELEMETRY_V2_PAYLOAD_LEN:
        (
            unix_time_us, altitude_cm, velocity_cms, battery_mv, temperature_cC,
            latitude_1e7, longitude_1e7, gps_alt_cm, gps_sats, gps_fix_raw,
        ) = struct.unpack(_TELEMETRY_V2_FMT, payload)
        return (
            unix_time_us / 1_000_000.0,
            altitude_cm / 100.0,
            velocity_cms / 100.0,
            battery_mv / 1000.0,
            temperature_cC / 100.0,
            latitude_1e7 / 1e7,
            longitude_1e7 / 1e7,
            gps_alt_cm / 100.0,
            gps_sats,
            bool(gps_fix_raw),
        )

    raise ValueError(f"unexpected telemetry payload length: {len(payload)}")


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


def decode_event_payload(payload: bytes) -> tuple[int, int]:
    """Unpack an EVENT payload. Returns (code, param)."""
    if len(payload) != 5:
        raise ValueError(f"unexpected event payload length: {len(payload)}")
    code = payload[0]
    (param,) = struct.unpack("<i", payload[1:5])
    return code, param


def format_event(code: int, param: int) -> str:
    """Format an event code + param into a human-readable string."""
    name = _EVT_NAMES.get(code, f"0x{code:02X}")
    return f"[EVT] {name} param={param}"


def now_unix() -> float:
    """Return the current Unix time in seconds."""
    return time.time()
