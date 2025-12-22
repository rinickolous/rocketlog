from dataclasses import dataclass


@dataclass
class Manifest:
    format: str = "rocketlog"
    version: int = 1
    created_utc: str = ""
    video_filename: str = "video.mp4"
    telemetry_filename: str = "telemetry.jsonl"
    pipeline: str = ""
    notes: str = ""
