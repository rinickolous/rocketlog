import glob

from PySide6 import QtCore, QtWidgets

# ---------------------------------------- #
# Defaults                                  #
# ---------------------------------------- #

DEFAULTS: dict[str, object] = {
    "serial/port": "/dev/ttyACM1",
    "serial/baud": 115200,
    "serial/reconnect_interval_s": 2,
    "recording/output_dir": str(QtCore.QStandardPaths.writableLocation(
        QtCore.QStandardPaths.StandardLocation.HomeLocation
    ) + "/rocketlog_recordings"),
    "display/trail_length": 80,
    "display/altitude_scale_m": 2000,
}

_BAUD_OPTIONS = [9600, 38400, 115200, 230400, 460800]

_ORG  = "rocketlog"
_APP  = "rocketlog"


# ---------------------------------------- #

def get_settings() -> QtCore.QSettings:
    """Return the app-wide QSettings instance."""
    return QtCore.QSettings(_ORG, _APP)


def read_str(key: str) -> str:
    """Read a string setting, falling back to the default."""
    return str(get_settings().value(key, DEFAULTS[key]))


def read_int(key: str) -> int:
    """Read an integer setting, falling back to the default."""
    return int(get_settings().value(key, DEFAULTS[key]))  # type: ignore[arg-type]


def write(key: str, value: object) -> None:
    """Persist a single setting."""
    get_settings().setValue(key, value)


# ---------------------------------------- #
# SettingsTab                               #
# ---------------------------------------- #


class SettingsTab(QtWidgets.QWidget):
    """
    Settings tab widget.

    Groups settings into labelled sections. Changes are applied and persisted
    immediately when the user edits a field or clicks Apply.

    Signals
    -------
    settings_changed : emitted whenever any value is saved, so other parts
                       of the app can react (e.g. reconnect with new port).
    """

    settings_changed = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(20)

        outer.addWidget(self._build_serial_section())
        outer.addWidget(self._build_recording_section())
        outer.addWidget(self._build_display_section())
        outer.addStretch(1)
        outer.addLayout(self._build_buttons())

        self._load()

    # ---------------------------------------- #

    def _section(self, title: str) -> tuple[QtWidgets.QFrame, QtWidgets.QFormLayout]:
        """Create a titled section frame with a form layout inside."""
        frame = QtWidgets.QFrame()
        frame.setObjectName("Panel")

        vbox = QtWidgets.QVBoxLayout(frame)
        vbox.setContentsMargins(12, 10, 12, 12)
        vbox.setSpacing(8)

        header = QtWidgets.QLabel(title)
        header.setObjectName("PanelTitle")
        vbox.addWidget(header)

        form = QtWidgets.QFormLayout()
        form.setContentsMargins(0, 4, 0, 0)
        form.setSpacing(10)
        form.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        vbox.addLayout(form)

        return frame, form

    # ---------------------------------------- #

    def _build_serial_section(self) -> QtWidgets.QFrame:
        """Build the Serial / Connection settings section."""
        frame, form = self._section("SERIAL / CONNECTION")

        # Port row: combo + refresh button
        port_row = QtWidgets.QHBoxLayout()
        port_row.setSpacing(6)
        self._port_combo = QtWidgets.QComboBox()
        self._port_combo.setEditable(True)
        self._port_combo.setMinimumWidth(180)
        self._refresh_btn = QtWidgets.QPushButton("Refresh")
        self._refresh_btn.setFixedWidth(80)
        self._refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(self._port_combo, stretch=1)
        port_row.addWidget(self._refresh_btn)
        form.addRow("Serial port", port_row)

        # Baud rate
        self._baud_combo = QtWidgets.QComboBox()
        for b in _BAUD_OPTIONS:
            self._baud_combo.addItem(str(b), b)
        form.addRow("Baud rate", self._baud_combo)

        # Reconnect interval
        self._reconnect_spin = QtWidgets.QSpinBox()
        self._reconnect_spin.setRange(1, 60)
        self._reconnect_spin.setSuffix(" s")
        self._reconnect_spin.setFixedWidth(90)
        form.addRow("Reconnect interval", self._reconnect_spin)

        return frame

    # ---------------------------------------- #

    def _build_recording_section(self) -> QtWidgets.QFrame:
        """Build the Recording settings section."""
        frame, form = self._section("RECORDING")

        dir_row = QtWidgets.QHBoxLayout()
        dir_row.setSpacing(6)
        self._rec_dir_edit = QtWidgets.QLineEdit()
        self._rec_dir_edit.setMinimumWidth(260)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self._browse_rec_dir)
        dir_row.addWidget(self._rec_dir_edit, stretch=1)
        dir_row.addWidget(browse_btn)
        form.addRow("Output directory", dir_row)

        return frame

    # ---------------------------------------- #

    def _build_display_section(self) -> QtWidgets.QFrame:
        """Build the Display settings section."""
        frame, form = self._section("DISPLAY")

        self._trail_spin = QtWidgets.QSpinBox()
        self._trail_spin.setRange(10, 500)
        self._trail_spin.setSuffix(" pts")
        self._trail_spin.setFixedWidth(90)
        form.addRow("Sim trail length", self._trail_spin)

        self._alt_scale_spin = QtWidgets.QSpinBox()
        self._alt_scale_spin.setRange(100, 20000)
        self._alt_scale_spin.setSingleStep(100)
        self._alt_scale_spin.setSuffix(" m")
        self._alt_scale_spin.setFixedWidth(100)
        form.addRow("Altitude scale", self._alt_scale_spin)

        return frame

    # ---------------------------------------- #

    def _build_buttons(self) -> QtWidgets.QHBoxLayout:
        """Build the Apply / Reset row."""
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)

        apply_btn = QtWidgets.QPushButton("Apply")
        apply_btn.clicked.connect(self._apply)

        reset_btn = QtWidgets.QPushButton("Reset to Defaults")
        reset_btn.clicked.connect(self._reset)

        row.addStretch(1)
        row.addWidget(reset_btn)
        row.addWidget(apply_btn)

        return row

    # ---------------------------------------- #

    def _refresh_ports(self) -> None:
        """Scan /dev/ttyACM* and /dev/ttyUSB*, populate the port combo."""
        current = self._port_combo.currentText()
        ports = sorted(
            glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        )
        self._port_combo.clear()
        for p in ports:
            self._port_combo.addItem(p)
        # Restore previous text or keep it editable if not in list
        if current:
            idx = self._port_combo.findText(current)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)
            else:
                self._port_combo.setCurrentText(current)

    # ---------------------------------------- #

    def _browse_rec_dir(self) -> None:
        """Open a directory picker and update the recording path field."""
        current = self._rec_dir_edit.text() or str(
            QtCore.QStandardPaths.writableLocation(
                QtCore.QStandardPaths.StandardLocation.HomeLocation
            )
        )
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Recording Directory", current
        )
        if path:
            self._rec_dir_edit.setText(path)

    # ---------------------------------------- #

    def _load(self) -> None:
        """Load persisted settings into the UI controls."""
        self._refresh_ports()

        port = read_str("serial/port")
        idx = self._port_combo.findText(port)
        if idx >= 0:
            self._port_combo.setCurrentIndex(idx)
        else:
            self._port_combo.setCurrentText(port)

        baud = read_int("serial/baud")
        idx = self._baud_combo.findData(baud)
        self._baud_combo.setCurrentIndex(idx if idx >= 0 else 2)  # default 115200

        self._reconnect_spin.setValue(read_int("serial/reconnect_interval_s"))
        self._rec_dir_edit.setText(read_str("recording/output_dir"))
        self._trail_spin.setValue(read_int("display/trail_length"))
        self._alt_scale_spin.setValue(read_int("display/altitude_scale_m"))

    # ---------------------------------------- #

    def _apply(self) -> None:
        """Persist all current field values and emit settings_changed."""
        write("serial/port", self._port_combo.currentText())
        write("serial/baud", self._baud_combo.currentData())
        write("serial/reconnect_interval_s", self._reconnect_spin.value())
        write("recording/output_dir", self._rec_dir_edit.text())
        write("display/trail_length", self._trail_spin.value())
        write("display/altitude_scale_m", self._alt_scale_spin.value())
        self.settings_changed.emit()

    # ---------------------------------------- #

    def _reset(self) -> None:
        """Reset all settings to defaults and persist."""
        s = get_settings()
        for key, val in DEFAULTS.items():
            s.setValue(key, val)
        self._load()
        self.settings_changed.emit()
