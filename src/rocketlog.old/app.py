# rocketlog/app.py

import sys

from PySide6 import QtWidgets

from rocketlog.ui.main_window import MainWindow
from rocketlog.ui.style import APP_QSS


def _init_gstreamer() -> None:
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

    Gst.init(None)  # type: ignore


# ---------------------------------------- #


def main() -> int:
    """
    Application entry point.

    Responsibilities:
    - Initialize GStreamer once
    - Create QApplication
    - Apply global stylesheet
    - Create and show the MainWindow
    - Run the Qt event loop
    """
    _init_gstreamer()

    app = QtWidgets.QApplication(sys.argv)

    # Optional: set an explicit app name (helps in logs / window rules)
    app.setApplicationName("RocketLog")

    # Global styling (glass cockpit theme, etc.)
    if APP_QSS:
        app.setStyleSheet(APP_QSS)

    # Optional: ensure consistent Qt behavior in kiosk setups
    # e.g. if running under a Wayland kiosk compositor
    # (leave this off unless you want to force it)
    # os.environ.setdefault("QT_QPA_PLATFORM", "wayland")

    w = MainWindow()
    w.resize(1200, 700)
    w.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
