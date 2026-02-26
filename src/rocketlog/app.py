# rocketlog/app.py

import sys
from PySide6 import QtWidgets

from rocketlog.ui.main_window import MainWindow
from rocketlog.ui.style import APP_QSS


# ---------------------------------------- #


def main() -> int:
    """
    Application entry point.
    """

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("RocketLog")

    # Global styling
    if APP_QSS:
        app.setStyleSheet(APP_QSS)

    w = MainWindow()
    # TODO: Replace with whatever screen resolution the cyberdeck has.
    w.resize(1024, 600)
    w.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
