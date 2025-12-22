import math
import time

from rocketlog.telemetry.types import Telemetry


class TelemetrySimulator:
    """
    Simple deterministic telemetry simulator for testing purposes.

    Intended for:
    - UI development
    - Replay testing
    - Pipeline bring-up without hardware
    """

    def __init__(self) -> None:
        self._t0 = time.time()

    # ---------------------------------------- #

    def reset(self) -> None:
        self._t0 = time.time()

    # ---------------------------------------- #

    def sample(self) -> Telemetry:
        t_now = time.time()
        t = t_now - self._t0

        # Fake ascent / descent curve
        alt = max(0.0, 5.0 * t - 0.02 * t * t) * 10.0
        vel = (5.0 - 0.04 * t) * 10.0

        # Battery droop
        batt = 12.6 - 0.002 * t

        # Temperature oscillation
        temp = 22.0 + 2.0 * math.sin(t / 5.0)

        return Telemetry(
            t_unix=t_now,
            alt_m=round(alt, 2),
            vel_mps=round(vel, 2),
            batt_v=round(batt, 3),
            temp_c=round(temp, 2),
        )
