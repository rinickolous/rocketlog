import json
import zipfile
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from rocketlog.record.manifest import Manifest
from rocketlog.telemetry.protocol import ReceiverLog
from rocketlog.telemetry.types import Telemetry


class SessionRecorder:
    """
    Records:
     - MP4 video (created by GSteamer pipeline) into a session folder.
     - Telemetry JSONL (append-only) into same folder
    Then packages them into a single ZIP with custom extension
    """

    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.session_dir: Optional[Path] = None
        self.video_path: Optional[Path] = None
        self.telemetry_path: Optional[Path] = None
        self.receiver_log_path: Optional[Path] = None
        self.manifest_path: Optional[Path] = None

        self._telemetry_fp = None
        self._receiver_log_fp = None
        self._manifest: Optional[Manifest] = None
        self._is_recording = False

    # ---------------------------------------- #

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ---------------------------------------- #

    def start(self, pipeline_str: str) -> None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.session_dir = self.out_dir / f"session_{ts}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.video_path = self.session_dir / "video.mp4"
        self.telemetry_path = self.session_dir / "telemetry.jsonl"
        self.receiver_log_path = self.session_dir / "receiver_logs.jsonl"
        self.manifest_path = self.session_dir / "manifest.json"

        self._manifest = Manifest(created_utc=ts, pipeline=pipeline_str)

        self._telemetry_fp = open(self.telemetry_path, "w", encoding="utf-8")
        assert self.receiver_log_path is not None
        self._receiver_log_fp = open(self.receiver_log_path, "w", encoding="utf-8")
        self._is_recording = True

        with open(self.manifest_path, "w", encoding="utf-8") as f:
            json.dump(asdict(self._manifest), f, indent=2)

    # ---------------------------------------- #

    def write_telemetry(self, telemetry: Telemetry) -> None:
        if not self._is_recording or self._telemetry_fp is None:
            raise RuntimeError("Recording has not been started.")

        json_line = json.dumps(telemetry, separators=(",", ":"))
        self._telemetry_fp.write(json_line + "\n")

    def write_receiver_log(self, log: ReceiverLog) -> None:
        if not self._is_recording or self._receiver_log_fp is None:
            raise RuntimeError("Recording has not been started.")

        json_line = json.dumps(log.__dict__, separators=(",", ":"))
        self._receiver_log_fp.write(json_line + "\n")

    # ---------------------------------------- #

    def stop_and_package(self) -> Path:
        if not self._is_recording:
            raise RuntimeError("Recording has not been started.")

        self._is_recording = False
        if self._telemetry_fp is not None:
            self._telemetry_fp.close()
            self._telemetry_fp = None
        if self._receiver_log_fp is not None:
            self._receiver_log_fp.close()
            self._receiver_log_fp = None

        assert self.session_dir is not None
        assert self.video_path is not None
        assert self.telemetry_path is not None
        assert self.receiver_log_path is not None
        assert self.manifest_path is not None

        archive_name = (
            self.session_dir.name.replace("session_", "recording_") + ".rocketlog"
        )
        archive_path = self.out_dir / archive_name

        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.write(self.video_path, arcname="video.mp4")
            zf.write(self.telemetry_path, arcname="telemetry.jsonl")
            zf.write(self.receiver_log_path, arcname="receiver_logs.jsonl")
            zf.write(self.manifest_path, arcname="manifest.json")

        return archive_path
