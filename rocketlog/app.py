# rocketlog/app.py

import sys

from PySide6 import QtWidgets

# GI / GStreamer
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # type: ignore

from rocketlog.ui.main_window import MainWindow
from rocketlog.ui.style import APP_QSS


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
    Gst.init(None)

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
