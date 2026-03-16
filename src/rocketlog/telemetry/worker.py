from __future__ import annotations

import glob
import time

from PySide6 import QtCore

from rocketlog.telemetry.protocol import ReceiverLog, format_log, format_event
from rocketlog.telemetry.reader import TelemetryReader
from rocketlog.telemetry.types import Telemetry
from rocketlog.ui.settings_tab import read_int, read_str

# ---------------------------------------- #
# Helpers                                   #
# ---------------------------------------- #


def _is_candidate_port(device: str) -> bool:
    """Return True for /dev/ttyACM* and /dev/ttyUSB* devices."""
    return device.startswith("/dev/ttyACM") or device.startswith("/dev/ttyUSB")


def _looks_like_esp32(description: str) -> bool:
    """Heuristic: does the port description suggest a common ESP32 USB bridge?"""
    d = (description or "").lower()
    keywords = ("esp32", "espressif", "silicon labs", "cp210", "ch340", "wch", "ftdi", "usb serial")
    return any(k in d for k in keywords)


# ---------------------------------------- #
# TelemetryWorker                           #
# ---------------------------------------- #


class TelemetryWorker(QtCore.QObject):
    """
    QObject worker that drives a TelemetryReader on a background QThread.

    A QTimer on the worker's thread calls tick() repeatedly. The worker
    handles connection, reconnection with back-off, and emits Qt signals
    for telemetry packets, receiver log messages, and link state changes.

    Signals
    -------
    telemetry(object)      : Telemetry TypedDict
    receiver_log(object)   : ReceiverLog dataclass
    state(str)             : "connected" | "disconnected" | "reconnecting:<N>"
    error(str)             : human-readable error description
    info(str)              : human-readable informational message
    """

    telemetry    = QtCore.Signal(object)
    receiver_log = QtCore.Signal(object)
    state        = QtCore.Signal(str)
    error        = QtCore.Signal(str)
    info         = QtCore.Signal(str)

    # ---------------------------------------- #

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        """
        Return candidate serial ports as (device, description) pairs.

        Uses pyserial if available, falls back to a plain glob of /dev/ttyACM*
        and /dev/ttyUSB*. Results are sorted so ESP32-looking ports come first.
        """
        try:
            import serial.tools.list_ports  # type: ignore

            ports: list[tuple[str, str]] = []
            for p in serial.tools.list_ports.comports():
                device = getattr(p, "device", "") or ""
                desc = getattr(p, "description", "") or ""
                if _is_candidate_port(device):
                    ports.append((device, desc or "(unknown)"))
            ports.sort(key=lambda x: (not _looks_like_esp32(x[1]), x[0]))
            return ports
        except Exception:
            devices = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            return [(d, "(unknown)") for d in devices]

    # ---------------------------------------- #

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._reader: TelemetryReader | None = None
        self._running = False
        self._connect_attempts = 0
        self._next_connect_time: float = 0.0
        # Timer lives on whichever thread this object moves to.
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self.tick)

    # ---------------------------------------- #

    def _port(self) -> str:
        """Read the current serial port from settings."""
        return read_str("serial/port")

    def _baud(self) -> int:
        """Read the current baud rate from settings."""
        return read_int("serial/baud")

    def _reconnect_interval(self) -> float:
        """Read the reconnect interval from settings."""
        return float(read_int("serial/reconnect_interval_s"))

    # ---------------------------------------- #

    @QtCore.Slot()
    def start(self) -> None:
        """Begin connecting and polling. Safe to call from any thread via signal."""
        if self._running:
            return
        self._running = True
        self._connect_attempts = 0
        self._next_connect_time = 0.0
        self._timer.start()
        self._maybe_connect()

    # ---------------------------------------- #

    @QtCore.Slot()
    def stop(self) -> None:
        """Stop polling and close the serial port."""
        self._running = False
        self._timer.stop()
        self._close_reader()
        self.state.emit("disconnected")

    # ---------------------------------------- #

    @QtCore.Slot()
    def reconnect(self) -> None:
        """Force an immediate reconnect (e.g. after settings change)."""
        self._close_reader()
        self._connect_attempts = 0
        self._next_connect_time = 0.0
        if self._running:
            self._maybe_connect()

    # ---------------------------------------- #

    @QtCore.Slot()
    def tick(self) -> None:
        """
        Called periodically by an external QTimer.

        Attempts connection if not connected, then drains one telemetry
        sample plus any pending log messages.
        """
        if not self._running:
            return

        if self._reader is None:
            self._maybe_connect()
            return

        try:
            self._drain_logs()
            t = self._reader.sample(timeout_s=0.1)
            if t is not None:
                self.telemetry.emit(t)
            self._drain_logs()
        except Exception as exc:
            self._on_read_error(str(exc))

    # ---------------------------------------- #

    def _drain_logs(self) -> None:
        """Emit all pending receiver log messages and event packets."""
        if self._reader is None:
            return
        while True:
            log: ReceiverLog | None = self._reader.receiver_log()
            if log is None:
                break
            self.receiver_log.emit(log)
        while True:
            evt = self._reader.receiver_event()
            if evt is None:
                break
            code, param = evt
            # Synthesise a ReceiverLog so the existing UI pipeline handles it.
            synthetic = ReceiverLog(
                t_unix_receiver=None,
                t_unix_host=time.time(),
                level=0,  # DEBUG level — events are informational
                msg=format_event(code, param),
            )
            self.receiver_log.emit(synthetic)

    # ---------------------------------------- #

    def _maybe_connect(self) -> None:
        """Attempt to open the serial port if the retry window has elapsed."""
        if self._reader is not None:
            return

        now = time.time()
        if now < self._next_connect_time:
            self.state.emit(f"reconnecting:{self._connect_attempts}")
            return

        self.state.emit(f"reconnecting:{self._connect_attempts}")
        port = self._port()
        baud = self._baud()
        try:
            self._reader = TelemetryReader(port=port, baud=baud)
            self._connect_attempts = 0
            self.state.emit("connected")
            self.info.emit(f"Connected to {port}")
        except Exception as exc:
            self._reader = None
            self._connect_attempts += 1
            self._next_connect_time = now + self._reconnect_interval()
            self.state.emit(f"reconnecting:{self._connect_attempts}")
            self.error.emit(f"Connect failed ({port}): {exc}")

    # ---------------------------------------- #

    def _on_read_error(self, reason: str) -> None:
        """Handle a read-time error: close port, schedule reconnect."""
        self._close_reader()
        self._connect_attempts += 1
        self._next_connect_time = time.time() + self._reconnect_interval()
        self.state.emit(f"reconnecting:{self._connect_attempts}")
        self.error.emit(f"Read error: {reason}")

    # ---------------------------------------- #

    def _close_reader(self) -> None:
        """Close the reader if open, ignoring errors."""
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
            self._reader = None
