from __future__ import annotations

from dataclasses import dataclass

from rocketlog.telemetry.protocol import ReceiverLog
from rocketlog.telemetry.types import Telemetry


# ---------------------------------------- #
# Event types                               #
# ---------------------------------------- #


@dataclass(frozen=True)
class TelemetryEvent:
    """A decoded telemetry packet from the receiver."""

    telemetry: Telemetry


# ---------------------------------------- #


@dataclass(frozen=True)
class ReceiverLogEvent:
    """A log message emitted by the receiver firmware."""

    log: ReceiverLog


# ---------------------------------------- #


@dataclass(frozen=True)
class AckEvent:
    """An ACK response from the receiver (e.g. time-sync confirmation)."""

    ack_type: int
    seq: int
    status: int
    applied_time_unix: float | None


# ---------------------------------------- #


@dataclass(frozen=True)
class ReceiverEventEvent:
    """A compact event code packet from the firmware (LoRa-optimised)."""

    code: int
    param: int


# ---------------------------------------- #

TelemetryStreamEvent = TelemetryEvent | ReceiverLogEvent | AckEvent | ReceiverEventEvent
