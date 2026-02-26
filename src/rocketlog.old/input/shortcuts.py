from __future__ import annotations

from dataclasses import dataclass

from PySide6 import QtGui, QtWidgets
from typing import Protocol, cast


@dataclass(frozen=True)
class ShortcutSpec:
    name: str
    key: str


# ---------------------------------------- #

SHORTCUTS = {
    "start_recording": ShortcutSpec("Start Recording", "R"),
    "stop_recording": ShortcutSpec("Stop Recording", "S"),
    "toggle_recording": ShortcutSpec("Toggle Recording", "Space"),
    "fullscreen": ShortcutSpec("Fullscreen", "F"),
    "quit": ShortcutSpec("Quit", "Ctrl+Q"),
    "focus_camera": ShortcutSpec("Focus Camera", "C"),
    "focus_telemetry": ShortcutSpec("Focus Telemetry", "T"),
}


# ---------------------------------------- #


class ShortcutHost(Protocol):
    def start_recording(self) -> None: ...

    def stop_recording(self) -> None: ...

    def toggle_recording(self) -> None: ...

    def toggle_fullscreen(self) -> None: ...

    def close(self) -> bool: ...

    def focus_camera(self) -> None: ...

    def focus_telemetry(self) -> None: ...


# ---------------------------------------- #


def shortcut_hint(action: str) -> str:
    spec = SHORTCUTS.get(action)
    return "" if spec is None else spec.key


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

    def add(action: str, fn) -> None:
        spec = SHORTCUTS[action]
        a = QtGui.QAction(spec.name, qw)
        a.setShortcut(QtGui.QKeySequence(spec.key))
        a.triggered.connect(fn)
        QtWidgets.QWidget.addAction(qw, a)

    add("start_recording", window.start_recording)
    add("stop_recording", window.stop_recording)
    add("toggle_recording", window.toggle_recording)
    add("fullscreen", window.toggle_fullscreen)
    add("quit", window.close)
    add("focus_camera", window.focus_camera)
    add("focus_telemetry", window.focus_telemetry)
