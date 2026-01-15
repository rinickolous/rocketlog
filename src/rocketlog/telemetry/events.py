from __future__ import annotations

from dataclasses import dataclass

from rocketlog.telemetry.protocol import ReceiverLog
from rocketlog.telemetry.types import Telemetry


@dataclass(frozen=True)
class TelemetryEvent:
    telemetry: Telemetry


@dataclass(frozen=True)
class ReceiverLogEvent:
    log: ReceiverLog


@dataclass(frozen=True)
class AckEvent:
    ack_type: int
    seq: int
    status: int
    applied_time_unix: float | None


TelemetryStreamEvent = TelemetryEvent | ReceiverLogEvent | AckEvent
