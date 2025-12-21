# Built-in Imports
import json
import math
import os
import time
import zipfile
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# Library Imports
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402


# ---------------------------------------- #
#  File / Recording Format Pieces          #
# ---------------------------------------- #


@dataclass
class Manifest:
    format: str = "rocketlog"
    version: int = 1
    created_utc: str = ""
    video_filename: str = "video.mp4"
    telemetry_filename: str = "telemetry.jsonl"
    pipeline: str = ""
    notes: str = ""


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
        self.manifest_path: Optional[Path] = None

        self._telemetry_fp = None
        self._manifest: Optional[Manifest] = None
        self._is_recording = False

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    def start(self, pipeline_str: str) -> None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.session_dir = self.out_dir / f"session_{ts}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.video_path = self.session_dir / "video.mp4"
        self.telemetry_path = self.session_dir / "telemetry.jsonl"
        self.manifest_path = self.session_dir / "manifest.json"

        self._manifest = Manifest(created_utc=ts, pipeline=pipeline_str)

        self._telemetry_fp = open(self.telemetry_path, "w", encoding="utf-8")
        self._is_recording = True

        with open(self.manifest_path, "w", encoding="utf-8") as f:
            json.dump(asdict(self._manifest), f, indent=2)

    def write_telemetry(self, telemetry: dict) -> None:
        if not self._is_recording or self._telemetry_fp is None:
            raise RuntimeError("Recording has not been started.")

        json_line = json.dumps(telemetry, separators=(",", ":"))
        self._telemetry_fp.write(json_line + "\n")

    def stop_and_package(self) -> Path:
        if not self._is_recording:
            raise RuntimeError("Recording has not been started.")

        self._is_recording = False
        if self._telemetry_fp is not None:
            self._telemetry_fp.close()
            self._telemetry_fp = None

        assert self.session_dir is not None
        assert self.video_path is not None
        assert self.telemetry_path is not None
        assert self.manifest_path is not None

        archive_name = (
            self.session_dir.name.replace("session_", "recording_") + ".rocketlog"
        )
        archive_path = self.out_dir / archive_name

        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.write(self.video_path, arcname="video.mp4")
            zf.write(self.telemetry_path, arcname="telemetry.jsonl")
            zf.write(self.manifest_path, arcname="manifest.json")

        return archive_path


# ---------------------------------------- #
#   GStreamer Video Pipeline               #
# ---------------------------------------- #


class GstVideo(QtCore.QObject):
    """
    Builds a GStreamer pipeline and delivers frames to Qt via signals.
    Uses appsink for preview so we can display in a QLabel.

    Two modes:
    - preview pipeline: v4l2src -> videoconvert -> RGB -> appsink
    - record pipeline: v4l2src -> videoconvert -> tee -> (RGB -> appsink) + (x264 -> mp4mux -> filesink)
    """

    frame_ready = QtCore.Signal(QtGui.QImage)
    error = QtCore.Signal(str)
    info = QtCore.Signal(str)

    def __init__(self, camera_device: Optional[str] = None, parent=None):
        super().__init__(parent)
        self.camera_device = camera_device

        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsink: Optional[GstApp.AppSink] = None

        self._bus_timer = QtCore.QTimer(self)
        self._bus_timer.setInterval(50)
        self._bus_timer.timeout.connect(self._poll_bus)

        self._is_record_pipeline = False
        self._current_pipeline_str = ""

    @property
    def current_pipeline_str(self) -> str:
        return self._current_pipeline_str

    def start_preview(self) -> None:
        self.stop()

        device_part = f"device={self.camera_device} " if self.camera_device else ""
        # v4l2src: webcam
        # videoconvert to RGB
        # appsink emit-signals so we can receive frames
        pipeline_str = (
            f"v4l2src {device_part}do-timestamp=true ! "
            f"videoconvert ! video/x-raw,format=RGB ! "
            f"appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self._build_and_play(pipeline_str, is_record=False)

    # ---------------------------------------- #

    def start_recording(self, mp4_path: Path) -> None:
        self.stop()

        device_part = f"device={self.camera_device} " if self.camera_device else ""
        # v4l2src: webcam
        # tee to split stream
        # branch 1: RGB -> appsink for preview
        # branch 2: x264enc -> mp4mux -> filesink for recording
        pipeline_str = (
            f"v4l2src {device_part}do-timestamp=true ! "
            f"videoconvert ! tee name=t "
            f"t. ! queue ! video/x-raw,format=RGB ! "
            f"appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true "
            f"t. ! queue ! videoconvert ! "
            f"x264enc speed-preset=ultrafast tune=zerolatency bitrate=4000 key-int-max=30 ! "
            f'mp4mux faststart=true ! filesink location="{str(mp4_path)}"'
        )

        self._build_and_play(pipeline_str, is_record=True)

    def stop(self) -> None:
        self._bus_timer.stop()
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        self.pipeline = None
        self.appsink = None
        self._is_record_pipeline = False
        self._current_pipeline_str = ""

    # ---------------------------------------- #

    def stop_recording_gracefully(self, timeout_s: float = 3.0) -> None:
        """
        Ensure MP4 is finalized:
        - Send EOS
        - wait for EOS message on bus or timeout
        - set NULL
        """
        if self.pipeline is None:
            return

        if not self._is_record_pipeline:
            self.stop()
            return

        try:
            self.pipeline.send_event(Gst.Event.new_eos())
            bus = self.pipeline.get_bus()
            # Wait for EOS or ERROR
            msg = bus.timed_pop_filtered(
                int(timeout_s * Gst.SECOND),
                Gst.MessageType.EOS | Gst.MessageType.ERROR,
            )
            if msg is not None:
                if msg.type == Gst.MessageType.ERROR:
                    err, dbg = msg.parse_error()
                    self.error.emit(
                        f"GStreamer Error during EOS: {err.message}\n{dbg or ''}"
                    )
        except Exception as e:
            self.error.emit(f"Failed to stop recoridng gracefully: {e}")
        finally:
            self.stop()

    # ---------------------------------------- #
    #   Internal                               #
    # ---------------------------------------- #

    def _build_and_play(self, pipeline_str: str, is_record: bool) -> None:
        self._current_pipeline_str = pipeline_str
        self._is_record_pipeline = is_record

        try:
            pipeline = Gst.parse_launch(pipeline_str)
            if not isinstance(pipeline, Gst.Pipeline):
                pipeline = pipeline  # parse_launch returns a Gst.Element; it is still a pipeline-like bin
            self.pipeline = pipeline

            appsink = pipeline.get_by_name("appsink")
            if appsink is None:
                raise RuntimeError("Failed to get appsink from pipeline.")

            # Attach callback
            appsink.connect("new-sample", self._on_new_sample)
            self.appsink = appsink

            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to PLAYING state.")

            self._bus_timer.start()
            self.info.emit("Started GStreamer pipeline.")
        except Exception as e:
            self.error.emit(
                f"Failed to start GStreamer pipeline:\n{e}\n\nPipeline:\n{pipeline_str}"
            )
            self.stop()

    # ---------------------------------------- #

    def _poll_bus(self) -> None:
        if self.pipeline is None:
            return

        bus = self.pipeline.get_bus()
        while True:
            msg = bus.pop()
            if msg is None:
                break

            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self.error.emit(f"GStreamer Error: {err.message}\n{dbg or ''}")
                self.stop()
                break
            elif msg.type == Gst.MessageType.WARNING:
                warn, dbg = msg.parse_warning()
                self.info.emit(f"GStreamer Warning: {warn.message}\n{dbg or ''}")
            elif msg.type == Gst.MessageType.STATE_CHANGED:
                old_state, new_state, pending_state = msg.parse_state_changed()
                if msg.src == self.pipeline:
                    self.info.emit(
                        f"GStreamer Pipeline state changed: {old_state.value_nick} -> {new_state.value_nick}"
                    )

    # ---------------------------------------- #

    def _on_new_sample(self, sink: GstApp.AppSink) -> None:
        """
        Runs on a GStreamer streaming thread.
        Convert buffer -> QImage (RGB) and emit to Qt.
        """
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value("width")
        height = s.get_value("height")

        # Map Buffer
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            # Copy bytes out
            data = bytes(mapinfo.data)

            # Create QImage from RGB packed data
            # bytes_per_line = width * 3
            img = QtGui.QImage(
                data, width, height, width * 3, QtGui.QImage.Format.Format_RGB888
            ).copy()
            self.frame_ready.emit(img)
        finally:
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK


# ---------------------------------------- #
#   Main UI                                #
# ---------------------------------------- #


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketLog Recorder")

        self.video_label = QtWidgets.QLabel("No Video")
        self.video_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("QLabel { background: #111; color: #ccc;")

        self.telemetry_box = QtWidgets.QGroupBox("Telemetry Data")
        self.t_alt = QtWidgets.QLabel("Altitude: - m")
        self.t_vel = QtWidgets.QLabel("Velocity: - m/s")
        self.t_batt = QtWidgets.QLabel("Battery: - V")
        self.t_temp = QtWidgets.QLabel("Temperature: - °C")
        self.t_status = QtWidgets.QLabel("Status: IDLE")

        t_layout = QtWidgets.QVBoxLayout()
        for w in (self.t_alt, self.t_vel, self.t_batt, self.t_temp, self.t_status):
            w.setTextInteractionFlags(
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
            )
            t_layout.addWidget(w)
        self.telemetry_box.setLayout(t_layout)

        self.btn_start = QtWidgets.QPushButton("Start Recording")
        self.btn_stop = QtWidgets.QPushButton("Stop Recording")
        self.btn_stop.setEnabled(False)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(self.video_label, stretch=1)
        left.addLayout(btn_row)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self.telemetry_box)
        right.addStretch(1)

        root = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(root)
        layout.addLayout(left, stretch=3)
        layout.addLayout(right, stretch=1)
        self.setCentralWidget(root)

        # Recording folder
        self.recordings_dir = Path.cwd() / "recordings"
        self.recordings_dir.mkdir(parents=True, exist_ok=True)

        # Recorder + video pipeline
        self._recorder = SessionRecorder(out_dir=self.recordings_dir)
        self._gst = GstVideo(camera_device=None, parent=self)
        self._gst.frame_ready.connect(self.on_frame)
        self._gst.error.connect(self.on_error)
        self._gst.info.connect(self.on_info)

        # Telemetry timer (10 Hz)
        self._t0 = time.time()
        self._telemetry_timer = QtCore.QTimer(self)
        self._telemetry_timer.setInterval(100)
        self._telemetry_timer.timeout.connect(self.on_telemetry_tick)
        self._telemetry_timer.start()

        # Buttons
        self.btn_start.clicked.connect(self.start_recording)
        self.btn_stop.clicked.connect(self.stop_recording)

        # Start preview
        self._gst.start_preview()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        try:
            if self._recorder.is_recording:
                # finalize recording and package
                self._gst.stop_recording_gracefully()
                archive_path = self._recorder.stop_and_package()
                self.on_info(f"Recording saved to: {archive_path}")
        except Exception:
            pass
        finally:
            self._gst.stop()
        super().closeEvent(event)

    @QtCore.Slot(QtGui.QImage)
    def on_frame(self, img: QtGui.QImage) -> None:
        pix = QtGui.QPixmap.fromImage(img)
        self.video_label.setPixmap(
            pix.scaled(
                self.video_label.size(),
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
        )

    @QtCore.Slot()
    def on_error(self, msg: str) -> None:
        QtWidgets.QMessageBox.critical(self, "Error", msg)

    @QtCore.Slot()
    def on_info(self, msg: str) -> None:
        print(msg)

    def start_recording(self) -> None:
        if self._recorder.is_recording:
            return

        # Start session first so we have a file path for the pipeline
        # TODO: implement
        try:
            # We'll fill pipeline string after we build it; for v0 keep it in manifest as what we ran.
            # Start recorder with placeholder then overwrite manifest if you want; easiest is just pass later.
            # Here we start AFTER pipeline start so we can pass the exact pipeline string.
            pass
        except Exception as e:
            self.on_error(str(e))
            return

        # Prepare session dir & paths by calling recorder.start after we know the pipeline string
        # We need the mp4 path before building the record pipeline, so we generate a session dir first.
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        session_dir = self.recordings_dir / f"session_{ts}"
        session_dir.mkdir(parents=True, exist_ok=True)
        video_path = session_dir / "video.mp4"

        # Start recording pipeline (this stops preview pipeline and replaces it)
        self._gst.start_recording(mp4_path=video_path)

        # Now start recorder and point it at this session dir
        # We are reusing the recorder but forcing its session_dir to match
        self._recorder.session_dir = session_dir
        self._recorder.video_path = video_path
        self._recorder.telemetry_path = session_dir / "telemetry.jsonl"
        self._recorder.manifest_path = session_dir / "manifest.json"
        self._recorder._manifest = Manifest(
            created_utc=ts, pipeline=self._gst.current_pipeline_str
        )
        self._recorder._telemetry_fp = open(
            self._recorder.telemetry_path, "w", encoding="utf-8"
        )
        self._recorder._is_recording = True
        with open(self._recorder.manifest_path, "w", encoding="utf-8") as f:
            json.dump(asdict(self._recorder._manifest), f, indent=2)

        self.t_status.setText("Status: RECORDING")
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_recording(self) -> None:
        if not self._recorder.is_recording:
            return

        # Finalize MP4 properly (EOS), then package
        self._gst.stop_recording_gracefully()
        try:
            archive_path = self._recorder.stop_and_package()
        except Exception as e:
            self.on_error(f"Failed to finalize recording: {e}")
            return

        # Restart preview
        self._gst.start_preview()

        self.t_status.setText("Status: IDLE")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        QtWidgets.QMessageBox.information(
            self, "Saved", f"Recording saved to:\n{archive_path}"
        )

    def on_telemetry_tick(self) -> None:
        t = time.time() - self._t0

        alt = max(0.0, 5.0 * t - 0.02 * t * t) * 10.0
        vel = (5.0 - 0.04 * t) * 10.0
        batt = 12.6 - 0.002 * t
        temp = 22.0 + 2.0 * math.sin(t / 5.0)

        telemetry = {
            "t_unix": time.time(),
            "alt_m": round(alt, 2),
            "vel_mps": round(vel, 2),
            "batt_v": round(batt, 3),
            "temp_c": round(temp, 2),
        }

        self.t_alt.setText(f"Altitude: {telemetry['alt_m']} m")
        self.t_vel.setText(f"Velocity: {telemetry['vel_mps']} m/s")
        self.t_batt.setText(f"Battery: {telemetry['batt_v']} V")
        self.t_temp.setText(f"Temperature: {telemetry['temp_c']} °C")

        if self._recorder.is_recording:
            self._recorder.write_telemetry(telemetry)


def main():
    # Initialize GStreamer once
    Gst.init(None)

    app = QtWidgets.QApplication([])
    w = MainWindow()
    w.resize(1200, 700)
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
