from dataclasses import dataclass, field


@dataclass
class Manifest:
    format: str = "rocketlog"
    version: int = 1
    created_utc: str = ""
    # Sources present in this archive, e.g. ["rocket", "cansat"].
    # Each source has its own telemetry.jsonl + telemetry.csv pair.
    sources: list[str] = field(default_factory=list)
    notes: str = ""
