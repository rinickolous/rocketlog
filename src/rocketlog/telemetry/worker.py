import time
from PySide6 import QtCore

from rocketlog.telemetry.reader import TelemetryReader
from rocketlog.util.firmware_flash_config import load_firmware_flash_config


def _is_candidate_serial_port(device: str) -> bool:
    return device.startswith("/dev/ttyACM") or device.startswith("/dev/ttyUSB")


# ---------------------------------------- #


def _looks_like_esp32(description: str) -> bool:
    d = (description or "").lower()
    keywords = (
        "esp32",
        "espressif",
        "silicon labs",
        "cp210",
        "ch340",
        "wch",
        "ftdi",
        "usb serial",
    )
    return any(k in d for k in keywords)


# ---------------------------------------- #


class TelemetryWorker(QtCore.QObject):
    telemetry = QtCore.Signal(object)
    receiver_log = QtCore.Signal(object)
    state = QtCore.Signal(str)
    error = QtCore.Signal(str)
    info = QtCore.Signal(str)

    # ---------------------------------------- #

    @staticmethod
    def list_esp32_ports() -> list[tuple[str, str]]:
        """
        Return likely ESP32 serial ports as (device, description).

        Filtered to /dev/ttyACM* and /dev/ttyUSB* and common USB-serial
        bridge descriptors.
        """
        try:
            import serial.tools.list_ports  # type: ignore
        except Exception:
            return []

        ports: list[tuple[str, str]] = []
        for p in serial.tools.list_ports.comports():
            device = getattr(p, "device", "") or ""
            desc = getattr(p, "description", "") or ""
            if _is_candidate_serial_port(device):
                ports.append((device, desc or "(unknown)"))

        # Prefer ESP32-looking descriptions first, then stable by device.
        ports.sort(key=lambda x: (not _looks_like_esp32(x[1]), x[0]))
        return ports

    # ---------------------------------------- #

    def __init__(
        self,
        port: str | None = None,
        baud: int | None = None,
        reconnect_interval_s: float = 2.0,
        parent=None,
    ):
        super().__init__(parent)

        flash_cfg = load_firmware_flash_config("receiver")

        self._port = port or flash_cfg.port or "/dev/ttyACM1"
        self._baud = baud or flash_cfg.baud or 115200
        self._reconnect_interval_s = reconnect_interval_s

        self._reader: TelemetryReader | None = None
        # legacy ping support (unused with framed protocol)
        self._last_ping_time: float = 0.0
        self._running = False
        self._next_connect_attempt_time: float = 0.0
        self._connect_attempts: int = 0

    # ---------------------------------------- #
    def set_port(self, port: str) -> None:
        self._port = port
        self._connect_attempts = 0
        self._next_connect_attempt_time = 0.0

    # ---------------------------------------- #

    def _disconnect(self, reason: str) -> None:
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
        self._reader = None
        self._last_ping_time = 0.0
        self._next_connect_attempt_time = time.time() + self._reconnect_interval_s
        self.state.emit(f"reconnecting:{self._connect_attempts}")
        self.error.emit(reason)

    # ---------------------------------------- #

    def _maybe_connect(self) -> None:
        if self._reader is not None:
            return

        now = time.time()
        if now < self._next_connect_attempt_time:
            # We're waiting for the next retry window.
            self.state.emit(f"reconnecting:{self._connect_attempts}")
            return

        self.state.emit(f"reconnecting:{self._connect_attempts}")
        try:
            self._reader = TelemetryReader(port=str(self._port), baud=int(self._baud))
            self._last_ping_time = 0.0
            self._connect_attempts = 0
            self.state.emit("connected")
            self.info.emit(f"Telemetry connected: {self._port}")
        except Exception as e:
            self._reader = None
            self._connect_attempts += 1
            self._next_connect_attempt_time = now + self._reconnect_interval_s
            self.state.emit(f"reconnecting:{self._connect_attempts}")
            self.error.emit(f"Telemetry connect failed: {e}")

    # ---------------------------------------- #

    @QtCore.Slot()
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._connect_attempts = 0
        self._next_connect_attempt_time = 0.0
        self.state.emit(f"reconnecting:{self._connect_attempts}")
        self._maybe_connect()

    # ---------------------------------------- #

    @QtCore.Slot()
    def stop(self) -> None:
        self._running = False
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
        self._reader = None
        self._connect_attempts = 0
        self.state.emit("disconnected")

    # ---------------------------------------- #

    @QtCore.Slot()
    def tick(self) -> None:
        if not self._running:
            return

        if self._reader is None:
            self._maybe_connect()
            return

        try:
            # Drain any receiver logs available now.
            while True:
                log = self._reader.receiver_log()
                if log is None:
                    break
                self.receiver_log.emit(log)

            # Telemetry is allowed to block for up to ~1s.
            t = self._reader.sample()
            self.telemetry.emit(t)

            # Drain any logs that arrived with the telemetry frame.
            while True:
                log = self._reader.receiver_log()
                if log is None:
                    break
                self.receiver_log.emit(log)
        except Exception as e:
            self._disconnect(f"Telemetry read error: {e}")
