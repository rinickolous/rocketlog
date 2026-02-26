from PySide6 import QtCore, QtGui, QtWidgets


class VideoPanel(QtWidgets.QWidget):
    """
    Panel for displaying video feed from the rocket's camera.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._image: QtGui.QImage | None = None
        self._recording: bool = False

        self.setMinimumSize(160, 120)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self.setAutoFillBackground(True)

    # ---------------------------------------- #

    def set_frame(self, img: QtGui.QImage) -> None:
        """Push a new video frame and trigger a repaint."""
        self._image = img
        self.update()

    # ---------------------------------------- #

    def set_recording(self, value: bool) -> None:
        """Update recording state (reserved for future REC overlay)."""
        self._recording = value
        self.update()

    # ---------------------------------------- #

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        """Draw the current frame scaled to fit, or a "NO VIDEO" placeholder."""
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
