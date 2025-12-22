from typing import TypedDict


class Telemetry(TypedDict):
    """
    Canonical telemetry data structure

    All units are SI unless states otherwise.
    Timestamps are Unix time (seconds, float).
    """

    t_unix: float  # Unix timestamp in seconds
    alt_m: float  # Altitude in meters
    vel_mps: float  # Velocity in meters per second
    batt_v: float  # Battery voltage in volts
    temp_c: float  # Temperature in degrees Celsius
