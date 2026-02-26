from PySide6 import QtCore, QtGui, QtWidgets


# Log level constants — mirror the firmware's rocketlog_log_level_t values.
LOG_DEBUG = 0
LOG_INFO = 1
LOG_WARN = 2
LOG_ERROR = 3

_LEVEL_SYMBOL = {LOG_DEBUG: "D", LOG_INFO: "I", LOG_WARN: "W", LOG_ERROR: "E"}
_LEVEL_COLOR = {
    LOG_DEBUG: QtGui.QColor("#9CA3AF"),
    LOG_INFO: QtGui.QColor("#D6DADF"),
    LOG_WARN: QtGui.QColor("#ffcc66"),
    LOG_ERROR: QtGui.QColor("#ff4b4b"),
}

_MAX_LINES = 500


class LogPanel(QtWidgets.QWidget):
    """
    Scrolling log panel for transmitter log messages.

    append_line() adds a pre-formatted string.
    append_log() accepts a level int + message string and formats them.

    NOTE: LoRa packet size constraints mean log packets will need to be kept
    short; this panel is future-wired. For now it accepts plain strings.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self._build_ui()

    # ---------------------------------------- #

    def _build_ui(self) -> None:
        """Create the read-only plain text editor."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._text = QtWidgets.QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setMaximumBlockCount(_MAX_LINES)
        self._text.setFont(
            QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        )
        self._text.setObjectName("LogView")
        layout.addWidget(self._text)

    # ---------------------------------------- #

    def append_line(self, line: str, level: int = LOG_INFO) -> None:
        """Append a pre-formatted line at the given log level colour."""
        cursor = self._text.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)

        fmt = QtGui.QTextCharFormat()
        fmt.setForeground(_LEVEL_COLOR.get(level, _LEVEL_COLOR[LOG_INFO]))
        cursor.insertText(line + "\n", fmt)

        self._text.setTextCursor(cursor)
        self._text.ensureCursorVisible()

    # ---------------------------------------- #

    def append_log(self, level: int, msg: str, timestamp: str = "") -> None:
        """Format a log entry and append it."""
        sym = _LEVEL_SYMBOL.get(level, "?")
        ts_part = f" {timestamp}" if timestamp else ""
        self.append_line(f"[{sym}]{ts_part} {msg}", level)

    # ---------------------------------------- #

    def clear_log(self) -> None:
        """Clear all log entries."""
        self._text.clear()
