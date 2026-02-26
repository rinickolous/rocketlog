from datetime import datetime, timezone

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.telemetry.types import Telemetry


class TelemetryPanel(QtWidgets.QWidget):
    """
    Live telemetry readout panel.

    Displays time, GPS coordinates, altitude, satellite count, fix state,
    temperature, and barometric pressure. Call update_telemetry() to refresh.
    Call clear() to reset to dashes (link lost).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self._build_ui()

    # ---------------------------------------- #

    def _build_ui(self) -> None:
        """Create the label/value row grid."""
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(6)

        def row(label: str, obj_name: str) -> QtWidgets.QLabel:
            container = QtWidgets.QWidget()
            container.setObjectName("TelemetryRow")
            hl = QtWidgets.QHBoxLayout(container)
            hl.setContentsMargins(6, 3, 6, 3)
            hl.setSpacing(8)

            lbl = QtWidgets.QLabel(label)
            lbl.setObjectName("TelemetryKey")
            lbl.setFixedWidth(110)

            val = QtWidgets.QLabel("--")
            val.setObjectName(obj_name)
            val.setTextInteractionFlags(
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
            )

            hl.addWidget(lbl)
            hl.addWidget(val, stretch=1)
            outer.addWidget(container)
            return val

        self._time = row("TIME (UTC)", "TelemetryVal")
        self._lat = row("LAT", "TelemetryVal")
        self._lon = row("LON", "TelemetryVal")
        self._gps_alt = row("GPS ALT", "TelemetryVal")
        self._alt = row("BARO ALT", "TelemetryVal")
        self._vel = row("VELOCITY", "TelemetryVal")
        self._sats = row("SATELLITES", "TelemetryVal")
        self._fix = row("GPS FIX", "TelemetryVal")
        self._temp = row("TEMPERATURE", "TelemetryVal")
        self._pressure = row("PRESSURE", "TelemetryVal")
        self._batt = row("BATTERY", "TelemetryVal")

        outer.addStretch(1)

    # ---------------------------------------- #

    def update_telemetry(self, t: Telemetry) -> None:
        """Refresh all value labels from a telemetry packet."""
        ts = datetime.fromtimestamp(t["t_unix"], tz=timezone.utc).strftime(
            "%H:%M:%S.%f"
        )[:-3]
        self._time.setText(ts)
        self._lat.setText(f"{t['lat']:.6f}°")
        self._lon.setText(f"{t['lon']:.6f}°")
        self._gps_alt.setText(f"{t['gps_alt_m']:.1f} m")
        self._alt.setText(f"{t['alt_m']:.1f} m")
        self._vel.setText(f"{t['vel_mps']:.2f} m/s")
        self._sats.setText(str(t["gps_sats"]))
        self._fix.setText("YES" if t["gps_fix"] else "NO")
        self._temp.setText(f"{t['temp_c']:.1f} °C")
        self._pressure.setText(f"{t['pressure_pa']:.0f} Pa")
        self._batt.setText(f"{t['batt_v']:.2f} V")

    # ---------------------------------------- #

    def clear(self) -> None:
        """Reset all value labels to "--" (link lost)."""
        for w in (
            self._time, self._lat, self._lon, self._gps_alt, self._alt,
            self._vel, self._sats, self._fix, self._temp, self._pressure,
            self._batt,
        ):
            w.setText("--")
