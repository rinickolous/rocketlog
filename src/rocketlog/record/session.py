import csv
import json
import shutil
import zipfile
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import IO

from rocketlog.record.manifest import Manifest
from rocketlog.telemetry.types import Telemetry


# Ordered CSV columns — must match Telemetry keys exactly.
_CSV_FIELDS: list[str] = [
    "t_unix",
    "alt_m",
    "vel_mps",
    "batt_v",
    "temp_c",
    "pressure_pa",
    "lat",
    "lon",
    "gps_alt_m",
    "gps_sats",
    "gps_fix",
]


class SessionRecorder:
    """
    Records telemetry from one or more sources into a .rocketlog archive.

    Archive layout (ZIP):
        manifest.json
        rocket_telemetry.jsonl
        rocket_telemetry.csv
        cansat_telemetry.jsonl
        cansat_telemetry.csv

    Sources are registered at start() time. Any source key is valid; the
    convention is "rocket" and "cansat".

    Usage:
        recorder = SessionRecorder(out_dir=Path("recordings"))
        recorder.start(["rocket", "cansat"])
        recorder.write_telemetry("rocket", t)
        ...
        archive_path = recorder.stop_and_package()
    """

    def __init__(self, out_dir: Path) -> None:
        self._out_dir = out_dir
        self._session_dir: Path | None = None
        self._manifest: Manifest | None = None
        self._is_recording = False

        # Per-source open file handles: source → (jsonl_fp, csv_writer)
        self._jsonl_fps: dict[str, IO[str]] = {}
        self._csv_writers: dict[str, csv.DictWriter] = {}

    # ---------------------------------------- #

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ---------------------------------------- #

    def start(self, sources: list[str]) -> None:
        if self._is_recording:
            raise RuntimeError("Already recording.")

        self._out_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._session_dir = self._out_dir / f"session_{ts}"
        self._session_dir.mkdir(parents=True, exist_ok=True)

        self._manifest = Manifest(created_utc=ts, sources=list(sources))

        manifest_path = self._session_dir / "manifest.json"
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(asdict(self._manifest), f, indent=2)

        self._jsonl_fps.clear()
        self._csv_writers.clear()

        for source in sources:
            jsonl_path = self._session_dir / f"{source}_telemetry.jsonl"
            csv_path = self._session_dir / f"{source}_telemetry.csv"

            jsonl_fp = open(jsonl_path, "w", encoding="utf-8")
            csv_fp = open(csv_path, "w", encoding="utf-8", newline="")
            writer = csv.DictWriter(csv_fp, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            csv_fp.flush()

            self._jsonl_fps[source] = jsonl_fp
            # Stash the underlying file object so we can close it properly.
            self._csv_writers[source] = writer
            # Keep a reference to the csv file for flushing/closing.
            setattr(writer, "_fp", csv_fp)

        self._is_recording = True

    # ---------------------------------------- #

    def write_telemetry(self, source: str, t: Telemetry) -> None:
        if not self._is_recording:
            raise RuntimeError("Not recording.")
        if source not in self._jsonl_fps:
            raise KeyError(f"Unknown source '{source}'. Registered: {list(self._jsonl_fps)}")

        # JSONL
        self._jsonl_fps[source].write(json.dumps(t, separators=(",", ":")) + "\n")

        # CSV — cast bool to int so it round-trips cleanly
        row = {k: (int(v) if isinstance(v, bool) else v) for k, v in t.items()}
        self._csv_writers[source].writerow(row)

    # ---------------------------------------- #

    def stop_and_package(self) -> Path:
        if not self._is_recording:
            raise RuntimeError("Not recording.")

        self._is_recording = False

        # Close all open handles
        for fp in self._jsonl_fps.values():
            fp.close()
        self._jsonl_fps.clear()

        for writer in self._csv_writers.values():
            fp = getattr(writer, "_fp", None)
            if fp is not None:
                fp.close()
        self._csv_writers.clear()

        assert self._session_dir is not None
        assert self._manifest is not None

        ts = self._manifest.created_utc
        archive_name = f"recording_{ts}.rocketlog"
        archive_path = self._out_dir / archive_name

        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for path in sorted(self._session_dir.iterdir()):
                zf.write(path, arcname=path.name)

        shutil.rmtree(self._session_dir, ignore_errors=True)
        self._session_dir = None

        return archive_path
