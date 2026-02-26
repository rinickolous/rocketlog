from dataclasses import dataclass
from typing import Any


def _gst() -> Any:
    # Returns the gi.repository.Gst module (runtime import)
    try:
        import gi  # type: ignore

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # type: ignore
    except ModuleNotFoundError as exc:
        if exc.name != "gi":
            raise
        raise RuntimeError(
            "Missing GStreamer GI bindings. Install OS packages for Python GI + GStreamer "
            "(e.g. python3-gi/python-gobject and gstreamer plugins)."
        ) from exc

    return Gst


@dataclass(frozen=True)
class CameraDevice:
    """
    GStreamer-discovered video source device.

    id: stable identifier you can feed back into your pipeline builder
    label: human-readable name shown in the UI
    """

    id: str
    label: str


# ---------------------------------------- #
#  Device Discovery                        #
# ---------------------------------------- #


def list_camera_devices() -> list[CameraDevice]:
    """
    Return available video capture devices as seen by GStreamer.

    This usually finds v4l2 webcams (/dev/video*) and may also find libcamera sources
    depending on the system.
    """
    Gst = _gst()
    mon = Gst.DeviceMonitor()
    mon.add_filter("Video/Source", None)

    devices: list[CameraDevice] = []

    mon.start()
    try:
        for dev in mon.get_devices() or []:
            display = dev.get_display_name() or "Unknown Camera"
            props = dev.get_properties()  # Gst.Structure or None

            # Best case (v4l2): props often include "device.path" = "/dev/video0"
            dev_id = ""
            if props is not None:
                for key in ("device.path", "api.v4l2.path", "node", "path"):
                    if props.has_field(key):
                        dev_id = str(props.get_value(key))
                        break

            # Fallback: use display name if no path is available
            if not dev_id:
                dev_id = display

            devices.append(CameraDevice(id=dev_id, label=display))
    finally:
        mon.stop()

    # Stable ordering for UI
    devices.sort(key=lambda d: d.label.lower())
    return devices
