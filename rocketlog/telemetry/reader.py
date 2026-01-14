import time
import serial

from rocketlog.telemetry.types import Telemetry


class TelemetryReader:
    """
    Telemetry data reader.
    """

    def __init__(self, port: str = "/dev/ttyACM1", baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self.last_ping_time: float = 0.0
        self._ser = serial.Serial(self.port, self.baud, timeout=1)

        # Drain any partial line at startup
        self._ser.reset_input_buffer()

        self._send_time_sync()

    # ---------------------------------------- #

    def close(self) -> None:
        self._ser.close()

    # ---------------------------------------- #

    def _send_time_sync(self) -> None:
        t = time.time()
        msg = f"SET_TIME,{t:.6f}\n".encode("ascii")
        self._ser.write(msg)

        # Read a few lines looking for the ack
        deadline = time.time() + 3.0
        while time.time() < deadline:
            line = self._ser.readline()
            if not line:
                continue
            s = line.decode("utf-8", errors="replace").strip()
            print("RX:", s)
            if s.startswith("# TIME_SET"):
                break

    # ---------------------------------------- #

    def ping(self) -> None:
        self._ser.write(b"PING\n")

    # ---------------------------------------- #

    def sample(self) -> Telemetry:
        while True:
            raw = self._ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8").strip()

            parts = line.split(",")
            print("RX:", line)
            if len(parts) != 5:
                raise ValueError("Invalid telemetry data format")

            return Telemetry(
                t_unix=float(parts[0]),
                alt_m=float(parts[1]),
                vel_mps=float(parts[2]),
                batt_v=float(parts[3]),
                temp_c=float(parts[4]),
            )
