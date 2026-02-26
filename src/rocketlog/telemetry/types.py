from typing import TypedDict


class Telemetry(TypedDict):
    """
    Canonical telemetry sample. All units SI unless noted.
    Timestamps are Unix time (float, seconds).
    """

    t_unix: float       # Unix timestamp (seconds)
    alt_m: float        # Altitude (metres above launch site)
    vel_mps: float      # Vertical velocity (m/s)
    batt_v: float       # Battery voltage (V)
    temp_c: float       # Temperature (°C)
    pressure_pa: float  # Barometric pressure (Pa)
    lat: float          # GPS latitude (decimal degrees)
    lon: float          # GPS longitude (decimal degrees)
    gps_alt_m: float    # GPS altitude (metres)
    gps_sats: int       # Satellites in use
    gps_fix: bool       # GPS fix acquired
