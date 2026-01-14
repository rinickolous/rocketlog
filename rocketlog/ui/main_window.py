import json
from dataclasses import asdict

from datetime import datetime, timezone
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.input.shortcuts import install_shortcuts
from rocketlog.record.manifest import Manifest
from rocketlog.record.recorder import SessionRecorder
from rocketlog.telemetry.worker import TelemetryWorker
from rocketlog.telemetry.types import Telemetry
from rocketlog.ui.hud import HudVideoWidget
from rocketlog.util.time import format_timestamp
from rocketlog.video.gst_pipeline import GstVideo
from rocketlog.video.devices import list_camera_devices, CameraDevice


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RocketLog Recorder")

        # Video Preview Panel
        self.video_view = HudVideoWidget(self)
        self.video_view.setMinimumSize(640, 480)

        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.setMinimumWidth(260)
        self.camera_combo.addItem("Detecting cameras…")
        self.camera_combo.setEnabled(False)

        self._cameras: list[CameraDevice] = []
        self._initialized = False

        self.camera_combo.currentIndexChanged.connect(self.on_camera_changed)

        self.video_box = QtWidgets.QGroupBox("Video")
        vg = QtWidgets.QVBoxLayout(self.video_box)
        vg.addWidget(QtWidgets.QLabel("Camera"))
        vg.addWidget(self.camera_combo)

        self.telemetry_box = QtWidgets.QGroupBox("Telemetry Data")
        self.t_time = QtWidgets.QLabel("Clock: 00:00:00")
        self.t_alt = QtWidgets.QLabel("Altitude: - m")
        self.t_vel = QtWidgets.QLabel("Velocity: - m/s")
        self.t_batt = QtWidgets.QLabel("Battery: - V")
        self.t_temp = QtWidgets.QLabel("Temperature: - °C")
        self.t_status = QtWidgets.QLabel("Status: IDLE")

        t_layout = QtWidgets.QVBoxLayout()
        for w in (
            self.t_time,
            self.t_alt,
            self.t_vel,
            self.t_batt,
            self.t_temp,
            self.t_status,
        ):
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
        # left.addWidget(self.video_label, stretch=1)
        left.addWidget(self.video_view, stretch=1)
        left.addLayout(btn_row)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self.video_box)
        right.addWidget(self.telemetry_box)
        right.addStretch(1)

        self.ann_bar = QtWidgets.QFrame()
        self.ann_bar.setObjectName("AnnBar")
        ann_layout = QtWidgets.QHBoxLayout(self.ann_bar)
        ann_layout.setContentsMargins(12, 8, 12, 8)
        ann_layout.setSpacing(10)

        def chip(text: str, object_name: str) -> QtWidgets.QLabel:
            lbl = QtWidgets.QLabel(text)
            lbl.setObjectName(object_name)
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lbl.setMinimumWidth(100)
            return lbl

        self.chip_mode = chip("IDLE", "ChipNeutral")
        self.chip_link = chip("LINK OK", "ChipGood")
        self.chip_gps = chip("GPS SIM", "ChipNeutral")
        self.chip_storage = chip("STORAGE OK", "ChipGood")
        self.chip_rec = chip("REC OFF", "ChipNeutral")

        ann_layout.addWidget(self.chip_mode)
        ann_layout.addStretch(1)
        ann_layout.addWidget(self.chip_link)
        ann_layout.addWidget(self.chip_gps)
        ann_layout.addWidget(self.chip_storage)
        ann_layout.addWidget(self.chip_rec)

        root = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)

        main_row = QtWidgets.QHBoxLayout()
        main_row.addLayout(left, stretch=3)
        main_row.addLayout(right, stretch=1)

        outer.addWidget(self.ann_bar)
        outer.addLayout(main_row)

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

        # Telemetry (async)
        self._telemetry_thread = QtCore.QThread(self)
        self._telemetry_worker = TelemetryWorker(parent=None)
        self._telemetry_worker.moveToThread(self._telemetry_thread)
        self._telemetry_thread.started.connect(self._telemetry_worker.start)
        self._telemetry_worker.telemetry.connect(self.on_telemetry)
        self._telemetry_worker.error.connect(self.on_info)
        self._telemetry_worker.info.connect(self.on_info)
        self._telemetry_thread.start()

        self._telemetry: Telemetry | None = None

        # Telemetry timer (10 Hz) - ticks worker thread
        self._telemetry_timer = QtCore.QTimer(self)
        self._telemetry_timer.setInterval(100)
        self._telemetry_timer.timeout.connect(self._telemetry_worker.tick)
        self._telemetry_timer.start()

        # Buttons
        self.btn_start.clicked.connect(self.start_recording)
        self.btn_stop.clicked.connect(self.stop_recording)

        install_shortcuts(self)

        # Defer camera enumeration + preview start until the event loop
        QtCore.QTimer.singleShot(0, self._finish_startup)

    @QtCore.Slot()
    def _finish_startup(self) -> None:
        if self._initialized:
            return
        self._initialized = True

        # Camera enumeration might touch the system; do it after first paint.
        try:
            self._cameras = list_camera_devices()
        except Exception as e:
            self._cameras = []
            self.on_info(f"Camera discovery failed: {e}")

        self.camera_combo.blockSignals(True)
        try:
            self.camera_combo.clear()
            if not self._cameras:
                self.camera_combo.addItem("No cameras found")
                self.camera_combo.setEnabled(False)
            else:
                for cam in self._cameras:
                    self.camera_combo.addItem(cam.label, cam.id)
                self.camera_combo.setEnabled(True)
        finally:
            self.camera_combo.blockSignals(False)

        # Start preview with whatever camera is selected.
        device = self.camera_combo.currentData()
        if device:
            self._gst.set_source(str(device))
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
            self._gst.stop_recording()

        try:
            self._telemetry_timer.stop()
        except Exception:
            pass

        try:
            if self._telemetry_thread.isRunning():
                self._telemetry_worker.stop()
                self._telemetry_thread.quit()
                self._telemetry_thread.wait(1500)
        except Exception:
            pass

        super().closeEvent(event)

    # ---------------------------------------- #

    @QtCore.Slot(int)
    def on_camera_changed(self, idx: int) -> None:
        if idx < 0:
            return
        device_id = self.camera_combo.itemData(idx)
        if not device_id:
            return
        self._gst.set_source(str(device_id))

    # ---------------------------------------- #

    @QtCore.Slot(QtGui.QImage)
    def on_frame(self, img: QtGui.QImage) -> None:
        self.video_view.set_frame(img)

    # ---------------------------------------- #

    @QtCore.Slot()
    def on_error(self, msg: str) -> None:
        QtWidgets.QMessageBox.critical(self, "Error", msg)

    # ---------------------------------------- #

    @QtCore.Slot()
    def on_info(self, msg: str) -> None:
        print(msg)

    # ---------------------------------------- #

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

    # ---------------------------------------- #

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

    # ---------------------------------------- #

    def on_telemetry_tick(self) -> None:
        # Kept for compatibility; telemetry now arrives via worker thread.
        return

    @QtCore.Slot(object)
    def on_telemetry(self, t: Telemetry) -> None:
        self._telemetry = t

        self.t_time.setText(f"Clock: {format_timestamp(t['t_unix'])}")
        self.t_alt.setText(f"Altitude: {t['alt_m']} m")
        self.t_vel.setText(f"Velocity: {t['vel_mps']} m/s")
        self.t_batt.setText(f"Battery: {t['batt_v']} V")
        self.t_temp.setText(f"Temperature: {t['temp_c']} °C")

        self.video_view.set_telemetry(t)

        if self._recorder.is_recording:
            self._recorder.write_telemetry(t)

    # ---------------------------------------- #

    def toggle_recording(self) -> None:
        if self._recorder.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    # ---------------------------------------- #

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    # ---------------------------------------- #

    def _set_rec_ui(self, on: bool) -> None:
        self.video_view.set_recording(on)
        self.chip_rec.setText("REC ON" if on else "REC OFF")
        self.chip_rec.setObjectName("ChipWarn" if on else "ChipNeutral")
        self.chip_mode.setText("RECORD" if on else "IDLE")
        self.chip_mode.setObjectName("ChipCaution" if on else "ChipNeutral")

        # reapply QSS when objectName changes
        for w in (self.chip_rec, self.chip_mode):
            w.style().unpolish(w)
            w.style().polish(w)
            w.update()
