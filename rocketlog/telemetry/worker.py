import time
from PySide6 import QtCore

from rocketlog.telemetry.reader import TelemetryReader


class TelemetryWorker(QtCore.QObject):
    telemetry = QtCore.Signal(object)
    error = QtCore.Signal(str)
    info = QtCore.Signal(str)

    def __init__(self, port: str = "/dev/ttyACM1", baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._reader: TelemetryReader | None = None
        self._last_ping_time: float = 0.0
        self._running = False

    @QtCore.Slot()
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        try:
            self._reader = TelemetryReader(port=self._port, baud=self._baud)
            self.info.emit(f"Telemetry connected: {self._port}")
        except Exception as e:
            self._reader = None
            self._running = False
            self.error.emit(f"Telemetry connect failed: {e}")
            return

    @QtCore.Slot()
    def stop(self) -> None:
        self._running = False
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
            self._reader = None

    @QtCore.Slot()
    def tick(self) -> None:
        if not self._running or self._reader is None:
            return
        try:
            now = time.time()
            if now - self._last_ping_time >= 1.0:
                self._reader.ping()
                self._last_ping_time = now
            t = self._reader.sample()
            self.telemetry.emit(t)
        except Exception as e:
            self.error.emit(f"Telemetry read error: {e}")
