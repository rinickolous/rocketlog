from datetime import datetime, timezone


def format_timestamp(t_unix: float) -> str:
    dt = datetime.fromtimestamp(t_unix, tz=timezone.utc)
    return dt.strftime("%H:%M:%S")
