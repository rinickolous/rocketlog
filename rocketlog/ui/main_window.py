import json
import math
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.record.manifest import Manifest
from rocketlog.record.recorder import SessionRecorder
from rocketlog.video.gst_pipeline import GstVideo


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
