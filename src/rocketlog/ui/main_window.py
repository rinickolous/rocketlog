from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.record.session import SessionRecorder
from rocketlog.telemetry.protocol import ReceiverLog, format_log
from rocketlog.telemetry.types import Telemetry
from rocketlog.telemetry.worker import TelemetryWorker
from rocketlog.ui.log_panel import LogPanel
from rocketlog.ui.settings_tab import SettingsTab, read_str
from rocketlog.ui.sim_panel import SimPanel
from rocketlog.ui.tab_bar import LiveTabBar, PlaybackTabBar
from rocketlog.ui.telemetry_panel import TelemetryPanel
from rocketlog.ui.video_panel import VideoPanel

_RECORDINGS_DIR = Path.home() / "rocketlog_recordings"


class _SourceTab(QtWidgets.QWidget):
    """
    One tab in the main view (Rocket or CanSat).

    Layout:
      ┌─────────────┬─────────────┐
      │  VIDEO      │  SIM        │
      ├─────────────┼─────────────┤
      │  TELEMETRY  │  LOG        │
      └─────────────┴─────────────┘
      │  LiveTabBar (link + rec)  │
      └───────────────────────────┘
    """

    def __init__(self, link_label: str, parent=None):
        super().__init__(parent)

        self.video = VideoPanel(self)
        self.sim = SimPanel(self)
        self.telemetry = TelemetryPanel(self)
        self.log = LogPanel(self)
        self.tab_bar = LiveTabBar(link_label, self)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        grid_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_widget)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setSpacing(6)

        grid.addWidget(self._panel("VIDEO", self.video), 0, 0)
        grid.addWidget(self._panel("3D SIM", self.sim), 0, 1)
        grid.addWidget(self._panel("TELEMETRY", self.telemetry), 1, 0)
        grid.addWidget(self._panel("LOG", self.log), 1, 1)

        # Top row taller than bottom row
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 2)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        root.addWidget(grid_widget, stretch=1)
        root.addWidget(self.tab_bar)

    # ---------------------------------------- #

    @staticmethod
    def _panel(title: str, content: QtWidgets.QWidget) -> QtWidgets.QFrame:
        """Wrap a widget in a titled panel frame."""
        frame = QtWidgets.QFrame()
        frame.setObjectName("Panel")
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        header = QtWidgets.QLabel(title)
        header.setObjectName("PanelTitle")

        layout.addWidget(header)
        layout.addWidget(content, stretch=1)

        return frame

    # ---------------------------------------- #

    def update_telemetry(self, t: Telemetry) -> None:
        """Push a telemetry packet to the telemetry and sim panels."""
        self.telemetry.update_telemetry(t)
        self.sim.update_telemetry(t)

    def on_link_lost(self) -> None:
        """Clear all live panels when the link drops."""
        self.telemetry.clear()
        self.sim.clear()


# ---------------------------------------- #
# MainWindow                                #
# ---------------------------------------- #


class MainWindow(QtWidgets.QMainWindow):
    """
    Top-level application window.

    Layout:
      - Tab widget: "Rocket" | "CanSat" | "Playback"
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("RocketLog")

        self._recorder = SessionRecorder(out_dir=_RECORDINGS_DIR)

        self._build_tabs()
        self._wire_recording()

        root = QtWidgets.QWidget()
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)
        root_layout.addWidget(self._tabs, stretch=1)
        self.setCentralWidget(root)

        # Worker + thread — created here, started in _finish_startup.
        self._worker_thread = QtCore.QThread(self)
        self._worker = TelemetryWorker()
        self._worker.moveToThread(self._worker_thread)
        self._worker.telemetry.connect(self.on_rocket_telemetry)
        self._worker.receiver_log.connect(self._on_receiver_log)
        self._worker.state.connect(self.on_link_state)
        self._worker.error.connect(self._on_worker_error)
        # start()/tick() are called on the worker thread via thread.started signal.
        self._worker_thread.started.connect(self._worker.start)
        # settings_changed must reconnect on the worker thread — use QueuedConnection.
        self._settings_tab.settings_changed.connect(
            self._worker.reconnect,
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

        # Defer any IO-touching initialisation until after first paint.
        QtCore.QTimer.singleShot(0, self._finish_startup)

    # ---------------------------------------- #

    def _build_tabs(self) -> None:
        """Create Rocket, CanSat, Playback, and Settings tabs."""
        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setObjectName("MainTabs")

        self._rocket_tab = _SourceTab("ROCKET LINK")
        self._cansat_tab = _SourceTab("CANSAT LINK")

        self._playback_tab = QtWidgets.QWidget()
        self._build_playback_tab()

        self._settings_tab = SettingsTab()

        self._tabs.addTab(self._rocket_tab, "Rocket")
        self._tabs.addTab(self._cansat_tab, "CanSat")
        self._tabs.addTab(self._playback_tab, "Playback")
        self._tabs.addTab(self._settings_tab, "Settings")

    # ---------------------------------------- #

    def _build_playback_tab(self) -> None:
        """Build the Playback tab: reuses a _SourceTab for panels, adds a PlaybackTabBar."""
        outer = QtWidgets.QVBoxLayout(self._playback_tab)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._playback_source_tab = _SourceTab("PLAYBACK")
        # Playback has its own bar, so hide the one embedded in _SourceTab.
        self._playback_source_tab.tab_bar.hide()

        self._playback_bar = PlaybackTabBar()

        outer.addWidget(self._playback_source_tab, stretch=1)
        outer.addWidget(self._playback_bar)

        self._playback_bar.open_requested.connect(self._on_playback_open)

    # ---------------------------------------- #

    def _wire_recording(self) -> None:
        """Connect LiveTabBar recording signals to the shared SessionRecorder."""
        self._rocket_tab.tab_bar.start_recording.connect(self._on_start_recording)
        self._rocket_tab.tab_bar.stop_recording.connect(self._on_stop_recording)
        self._cansat_tab.tab_bar.start_recording.connect(self._on_start_recording)
        self._cansat_tab.tab_bar.stop_recording.connect(self._on_stop_recording)

    # ---------------------------------------- #

    def _set_recording_ui(self, active: bool) -> None:
        """Sync all per-tab LiveTabBars to recording state."""
        self._rocket_tab.tab_bar.set_recording(active)
        self._cansat_tab.tab_bar.set_recording(active)

    # ---------------------------------------- #
    # Recording slots                           #
    # ---------------------------------------- #

    @QtCore.Slot()
    def _on_start_recording(self) -> None:
        """Start a new recording session for both sources."""
        if self._recorder.is_recording:
            return
        try:
            self._recorder.start(["rocket", "cansat"])
        except Exception as exc:  # noqa: BLE001
            QtWidgets.QMessageBox.warning(self, "Recording Error", str(exc))
            return
        self._set_recording_ui(True)

    @QtCore.Slot()
    def _on_stop_recording(self) -> None:
        """Stop the active recording session and show the archive path."""
        if not self._recorder.is_recording:
            return
        try:
            archive = self._recorder.stop_and_package()
        except Exception as exc:  # noqa: BLE001
            QtWidgets.QMessageBox.warning(self, "Recording Error", str(exc))
            return
        self._set_recording_ui(False)
        QtWidgets.QMessageBox.information(
            self, "Recording Saved", f"Saved to:\n{archive}"
        )

    # ---------------------------------------- #
    # Playback slots                            #
    # ---------------------------------------- #

    @QtCore.Slot()
    def _on_playback_open(self) -> None:
        """Open a file dialog and load the selected .rocketlog archive."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Recording",
            str(Path.home()),
            "RocketLog archives (*.rocketlog);;All files (*)",
        )
        if path:
            # Placeholder: just update the bar label for now.
            self._playback_bar.set_file(path, sample_count=0)

    # ---------------------------------------- #
    # Startup / shutdown                        #
    # ---------------------------------------- #

    @QtCore.Slot()
    def _finish_startup(self) -> None:
        """Deferred post-paint init: start the telemetry worker thread."""
        self._worker_thread.start()
        # TODO: enumerate serial ports / start GStreamer preview

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Stop the worker thread and finalise any open recording before closing."""
        self._worker.stop()
        self._worker_thread.quit()
        self._worker_thread.wait(3000)
        if self._recorder.is_recording:
            try:
                self._recorder.stop_and_package()
            except Exception:  # noqa: BLE001
                pass
        super().closeEvent(event)

    # ---------------------------------------- #
    # Public slots                              #
    # ---------------------------------------- #

    @QtCore.Slot(object)
    def _on_receiver_log(self, log: ReceiverLog) -> None:
        """Forward a firmware log message to the Rocket log panel."""
        self._rocket_tab.log.append_line(format_log(log.level, log.msg))

    @QtCore.Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        """Log a worker error to the Rocket log panel (non-modal)."""
        self._rocket_tab.log.append_line(msg)

    @QtCore.Slot()
    def _on_settings_changed(self) -> None:
        """Reconnect the telemetry worker when serial settings change."""
        self._worker.reconnect()

    # ---------------------------------------- #

    @QtCore.Slot(object)
    def on_rocket_telemetry(self, t: Telemetry) -> None:
        """Handle a rocket telemetry packet: update UI and write to recorder."""
        self._rocket_tab.update_telemetry(t)
        if self._recorder.is_recording:
            self._recorder.write_telemetry("rocket", t)

    @QtCore.Slot(object)
    def on_cansat_telemetry(self, t: Telemetry) -> None:
        """Handle a CanSat telemetry packet: update UI and write to recorder."""
        self._cansat_tab.update_telemetry(t)
        if self._recorder.is_recording:
            self._recorder.write_telemetry("cansat", t)

    @QtCore.Slot(str)
    def on_rocket_log(self, msg: str) -> None:
        """Append a log message to the Rocket log panel."""
        self._rocket_tab.log.append_line(msg)

    @QtCore.Slot(str)
    def on_cansat_log(self, msg: str) -> None:
        """Append a log message to the CanSat log panel."""
        self._cansat_tab.log.append_line(msg)

    @QtCore.Slot(str)
    def on_link_state(self, state: str) -> None:
        """
        Update per-tab link chips based on connection state string.

        Accepted values: "connected", "disconnected", "reconnecting[:<attempt>]".
        """
        if state == "connected":
            self._rocket_tab.tab_bar.set_link_state_labelled("ROCKET LINK", state)
            self._cansat_tab.tab_bar.set_link_state_labelled("CANSAT LINK", state)
        elif state == "disconnected":
            self._rocket_tab.tab_bar.set_link_state_labelled("ROCKET LINK", state)
            self._cansat_tab.tab_bar.set_link_state_labelled("CANSAT LINK", state)
            self._rocket_tab.on_link_lost()
            self._cansat_tab.on_link_lost()
        elif state.startswith("reconnecting"):
            self._rocket_tab.tab_bar.set_link_state_labelled("ROCKET LINK", state)
            self._cansat_tab.tab_bar.set_link_state_labelled("CANSAT LINK", state)

    def toggle_fullscreen(self) -> None:
        """Toggle between fullscreen and normal window mode."""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
