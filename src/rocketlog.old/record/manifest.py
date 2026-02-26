from dataclasses import dataclass


@dataclass
class Manifest:
    format: str = "rocketlog"
    version: int = 2
    created_utc: str = ""
    video_filename: str = "video.mp4"
    telemetry_filename: str = "telemetry.jsonl"
    receiver_logs_filename: str = "receiver_logs.jsonl"
    pipeline: str = ""
    notes: str = ""
