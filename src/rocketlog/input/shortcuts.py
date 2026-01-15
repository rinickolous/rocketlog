from PySide6 import QtGui, QtWidgets
from typing import Protocol, cast


class ShortcutHost(Protocol):
    def start_recording(self) -> None: ...

    # ---------------------------------------- #

    def stop_recording(self) -> None: ...

    # ---------------------------------------- #

    def toggle_recording(self) -> None: ...

    # ---------------------------------------- #

    def toggle_fullscreen(self) -> None: ...

    # ---------------------------------------- #

    def close(self) -> bool: ...


# ---------------------------------------- #


def install_shortcuts(window: ShortcutHost) -> None:
    """
    Installs QActions on the window so shortcuts work regardless of focus.

    Expected window methods:
    - start_recording()
    - stop_recording()
    - toggle_recording()
    - toggle_fullscreen()
    - close()
    """
    qw = cast(QtWidgets.QWidget, window)

    # ---------------------------------------- #

    def add(name: str, shortcut: str, fn) -> None:
        a = QtGui.QAction(name, qw)
        a.setShortcut(QtGui.QKeySequence(shortcut))
        a.triggered.connect(fn)
        QtWidgets.QWidget.addAction(qw, a)

    add("Start Recording", "R", window.start_recording)
    add("Stop Recording", "S", window.stop_recording)
    add("Toggle Recording", "Space", window.toggle_recording)
    add("Fullscreen", "F", window.toggle_fullscreen)
    add("Quit", "Ctrl+Q", window.close)
