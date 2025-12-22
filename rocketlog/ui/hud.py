from datetime import datetime

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.telemetry.types import Telemetry


class HudVideoWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._image: QtGui.QImage | None = None
        self._telemetry: Telemetry | None = None
        self._recording: bool = False

        self.setMinimumSize(800, 450)
        self.setAutoFillBackground(True)

    def set_frame(self, img: QtGui.QImage) -> None:
        self._image = img
        self.update()

    def set_telemetry(self, t: Telemetry) -> None:
        self._telemetry = t
        self.update()

    def set_recording(self, on: bool) -> None:
        self._recording = on
        self.update()

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)

        # Background
        p.fillRect(self.rect(), QtGui.QColor("#0b0f14"))

        if self._image is None:
            p.setPen(QtGui.QColor("#8aa2b2"))
            p.drawText(self.rect(), QtCore.Qt.AlignmentFlag.AlignCenter, "NO VIDEO")
            return

        target = self.rect().adjusted(10, 10, -10, -10)
        pix = QtGui.QPixmap.fromImage(self._image)
        scaled = pix.scaled(
            target.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )

        x = target.x() + (target.width() - scaled.width()) // 2
        y = target.y() + (target.height() - scaled.height()) // 2
        img_rect = QtCore.QRect(x, y, scaled.width(), scaled.height())
        p.drawPixmap(img_rect, scaled)

        # HUD overlay clipped to video area
        p.save()
        p.setClipRect(img_rect)

        green = QtGui.QColor("#2fe37a")

        # Crosshair
        c = img_rect.center()
        p.setPen(QtGui.QPen(green, 2))
        p.drawLine(c.x() - 30, c.y(), c.x() + 30, c.y())
        p.drawLine(c.x(), c.y() - 30, c.x(), c.y() + 30)
        p.drawEllipse(c, 18, 18)

        # Timestamp
        ts = datetime.now().strftime("%H:%M:%S")
        self._hud_text(
            p, img_rect.left() + 14, img_rect.top() + 24, f"T+ {ts}", color="#c8d2dc"
        )

        # REC indicator
        if self._recording:
            self._hud_text(
                p,
                img_rect.right() - 110,
                img_rect.top() + 24,
                "REC",
                color="#ff4b4b",
                box=True,
            )

        # Key metrics (bottom-left)
        if self._telemetry:
            alt = self._telemetry["alt_m"]
            vel = self._telemetry["vel_mps"]
            bat = self._telemetry["batt_v"]
            lines = [
                f"ALT {alt:8.2f} m",
                f"VEL {vel:8.2f} m/s",
                f"BAT {bat:8.3f} V",
            ]
            y0 = img_rect.bottom() - 18 - 18 * (len(lines) - 1)
            for i, line in enumerate(lines):
                self._hud_text(
                    p,
                    img_rect.left() + 14,
                    y0 + 18 * i,
                    line,
                    color="#2fe37a",
                    box=True,
                )

        p.restore()

    def _hud_text(
        self,
        p: QtGui.QPainter,
        x: int,
        y: int,
        text: str,
        color: str,
        box: bool = False,
    ) -> None:
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        p.setFont(font)

        fm = QtGui.QFontMetrics(font)
        w = fm.horizontalAdvance(text)
        h = fm.height()

        if box:
            r = QtCore.QRect(x - 6, y - h + 4, w + 12, h + 6)
            p.fillRect(r, QtGui.QColor(0, 0, 0, 120))
            p.setPen(QtGui.QPen(QtGui.QColor("#1b2a34"), 1))
            p.drawRect(r)

        p.setPen(QtGui.QColor(color))
        p.drawText(x, y, text)
