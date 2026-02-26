from PySide6 import QtCore, QtGui, QtWidgets


def _chip(text: str, object_name: str, min_width: int = 100) -> QtWidgets.QLabel:
    """Create a status chip label with the given text and QSS object name."""
    lbl = QtWidgets.QLabel(text)
    lbl.setObjectName(object_name)
    lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    lbl.setMinimumWidth(min_width)
    return lbl


def _restyle(widget: QtWidgets.QWidget, text: str, object_name: str) -> None:
    """Update a widget's text and QSS object name, forcing a style repaint."""
    if isinstance(widget, QtWidgets.QLabel):
        widget.setText(text)
    widget.setObjectName(object_name)
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


# ---------------------------------------- #
# LiveTabBar                                #
# ---------------------------------------- #


class LiveTabBar(QtWidgets.QFrame):
    """
    Control strip shown at the bottom of each live telemetry tab (Rocket / CanSat).

    Layout:
      [LINK chip]   <stretch>   [REC chip]  [Start Recording]  [Stop Recording]

    The link chip and rec chip are updated by MainWindow.
    The record buttons emit signals; MainWindow connects them to the shared recorder.
    """

    start_recording = QtCore.Signal()
    stop_recording = QtCore.Signal()

    def __init__(self, link_label: str, parent=None):
        super().__init__(parent)
        self.setObjectName("TabBar")

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(8)

        self.chip_link = _chip(f"{link_label} --", "ChipNeutral", min_width=160)
        self.chip_rec = _chip("REC OFF", "ChipNeutral")

        self._btn_start = QtWidgets.QPushButton("Start Recording")
        self._btn_start.setObjectName("RecBtn")
        self._btn_stop = QtWidgets.QPushButton("Stop Recording")
        self._btn_stop.setObjectName("RecBtn")
        self._btn_stop.setEnabled(False)

        self._btn_start.clicked.connect(self.start_recording)
        self._btn_stop.clicked.connect(self.stop_recording)

        layout.addWidget(self.chip_link)
        layout.addStretch(1)
        layout.addWidget(self.chip_rec)
        layout.addWidget(self._btn_start)
        layout.addWidget(self._btn_stop)

    # ---------------------------------------- #

    def set_link_state_labelled(self, prefix: str, state: str) -> None:
        """
        Update the link chip text and colour.

        prefix is the source label, e.g. "ROCKET LINK".
        state is one of: "connected", "disconnected", "reconnecting[:<attempt>]".
        """
        if state == "connected":
            _restyle(self.chip_link, f"{prefix} OK", "ChipGood")
        elif state == "disconnected":
            _restyle(self.chip_link, f"{prefix} LOST", "ChipWarn")
        elif state.startswith("reconnecting"):
            parts = state.split(":", 1)
            suffix = f" ({parts[1]})" if len(parts) == 2 and parts[1].isdigit() else ""
            _restyle(self.chip_link, f"{prefix} RECONNECTING{suffix}", "ChipCaution")
        else:
            _restyle(self.chip_link, f"{prefix} --", "ChipNeutral")

    # ---------------------------------------- #

    def set_recording(self, active: bool) -> None:
        """Sync button states and rec chip to current recording state."""
        self._btn_start.setEnabled(not active)
        self._btn_stop.setEnabled(active)
        if active:
            _restyle(self.chip_rec, "RECORDING", "ChipWarn")
        else:
            _restyle(self.chip_rec, "REC OFF", "ChipNeutral")


# ---------------------------------------- #
# PlaybackTabBar                            #
# ---------------------------------------- #


class PlaybackTabBar(QtWidgets.QFrame):
    """
    Control strip for the Playback tab.

    Layout (two rows):
      [Open…]  [path label]  <stretch>  [Rocket|CanSat]  [Speed combo]
      [■]  [▶]  [‖]   [scrubber]   [timestamp]
    """

    open_requested  = QtCore.Signal()
    source_changed  = QtCore.Signal(str)   # "rocket" or "cansat"
    speed_changed   = QtCore.Signal(float)
    play_requested  = QtCore.Signal()
    pause_requested = QtCore.Signal()
    stop_requested  = QtCore.Signal()
    scrub_requested = QtCore.Signal(int)   # sample index

    _SPEEDS = [("0.5×", 0.5), ("1×", 1.0), ("2×", 2.0), ("4×", 4.0)]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TabBar")

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 4, 8, 4)
        outer.setSpacing(4)

        # --- Top row: open button, path, source selector, speed -------------
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(8)

        self._btn_open = QtWidgets.QPushButton("Open…")
        self._btn_open.clicked.connect(self.open_requested)

        self._path_label = QtWidgets.QLabel("No file loaded")
        self._path_label.setObjectName("PathLabel")
        self._path_label.setMinimumWidth(120)

        self._radio_rocket = QtWidgets.QRadioButton("Rocket")
        self._radio_cansat = QtWidgets.QRadioButton("CanSat")
        self._radio_rocket.setChecked(True)
        self._radio_rocket.toggled.connect(
            lambda checked: self.source_changed.emit("rocket") if checked else None
        )
        self._radio_cansat.toggled.connect(
            lambda checked: self.source_changed.emit("cansat") if checked else None
        )

        self._speed_combo = QtWidgets.QComboBox()
        for label, _ in self._SPEEDS:
            self._speed_combo.addItem(label)
        self._speed_combo.setCurrentIndex(1)   # default 1×
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)

        top.addWidget(self._btn_open)
        top.addWidget(self._path_label, stretch=1)
        top.addStretch(1)
        top.addWidget(self._radio_rocket)
        top.addWidget(self._radio_cansat)
        top.addWidget(QtWidgets.QLabel("Speed:"))
        top.addWidget(self._speed_combo)

        # --- Bottom row: transport controls + scrubber + timestamp ----------
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(8)

        self._btn_stop_pb = QtWidgets.QPushButton("■")
        self._btn_stop_pb.setFixedWidth(34)
        self._btn_play = QtWidgets.QPushButton("▶")
        self._btn_play.setFixedWidth(34)
        self._btn_pause = QtWidgets.QPushButton("‖")
        self._btn_pause.setFixedWidth(34)

        self._btn_stop_pb.clicked.connect(self.stop_requested)
        self._btn_play.clicked.connect(self.play_requested)
        self._btn_pause.clicked.connect(self.pause_requested)

        self._scrubber = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self._scrubber.setObjectName("Scrubber")
        self._scrubber.setRange(0, 0)
        self._scrubber.sliderMoved.connect(self.scrub_requested)

        self._ts_label = QtWidgets.QLabel("--:--:--.---")
        self._ts_label.setObjectName("TelemetryVal")
        self._ts_label.setMinimumWidth(90)

        bottom.addWidget(self._btn_stop_pb)
        bottom.addWidget(self._btn_play)
        bottom.addWidget(self._btn_pause)
        bottom.addWidget(self._scrubber, stretch=1)
        bottom.addWidget(self._ts_label)

        outer.addLayout(top)
        outer.addLayout(bottom)

        self._set_transport_enabled(False)

    # ---------------------------------------- #

    def _on_speed_changed(self, idx: int) -> None:
        """Emit speed_changed with the float value for the selected combo index."""
        self.speed_changed.emit(self._SPEEDS[idx][1])

    # ---------------------------------------- #

    def _set_transport_enabled(self, enabled: bool) -> None:
        """Enable or disable all transport controls (play/pause/stop/scrubber)."""
        for w in (self._btn_play, self._btn_pause, self._btn_stop_pb, self._scrubber):
            w.setEnabled(enabled)

    # ---------------------------------------- #

    def set_file(self, path: str, sample_count: int) -> None:
        """Update the path label and scrubber range after a file is loaded."""
        import os
        self._path_label.setText(os.path.basename(path))
        self._path_label.setToolTip(path)
        self._scrubber.setRange(0, max(0, sample_count - 1))
        self._scrubber.setValue(0)
        self._set_transport_enabled(True)

    # ---------------------------------------- #

    def set_position(self, index: int, timestamp_str: str) -> None:
        """Update the scrubber position and timestamp label during playback."""
        self._scrubber.setValue(index)
        self._ts_label.setText(timestamp_str)

    # ---------------------------------------- #

    def set_playing(self, playing: bool) -> None:
        """Toggle play/pause button enabled states to reflect playback state."""
        self._btn_play.setEnabled(not playing)
        self._btn_pause.setEnabled(playing)

    # ---------------------------------------- #

    @property
    def current_speed(self) -> float:
        """Return the currently selected playback speed multiplier."""
        return self._SPEEDS[self._speed_combo.currentIndex()][1]

    @property
    def current_source(self) -> str:
        """Return the currently selected source: "rocket" or "cansat"."""
        return "cansat" if self._radio_cansat.isChecked() else "rocket"
