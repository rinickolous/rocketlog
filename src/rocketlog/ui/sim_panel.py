import math

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.telemetry.types import Telemetry


# ---------------------------------------- #
# Flat-plane perspective simulation panel   #
# ---------------------------------------- #
#
# The ground is an infinite flat plane. The camera sits at a fixed height and
# horizontal distance behind the rocket's nadir point, pitching up dynamically
# to keep the rocket centred vertically.
#
# Coordinate system
# -----------------
# World: +X right, +Y up, +Z forward (away from camera).
# Camera is at (0, CAM_HEIGHT, -CAM_DIST): behind and above the nadir origin.
# The camera always looks toward (0, rocket_alt * VERT_BIAS, 0) — pitching up
# as altitude increases so the rocket stays in view.
#
# Projection
# ----------
# Standard pin-hole:  sx = f * x_cam / z_cam + cx
#                     sy = -f * y_cam / z_cam + cy

_SKY_TOP    = QtGui.QColor("#060d06")
_SKY_HORIZ  = QtGui.QColor("#0d200d")
_GRID_BASE  = QtGui.QColor(0x1a, 0x4a, 0x1a)
_HORIZ_LINE = QtGui.QColor(0x2a, 0x7a, 0x2a)
_TRAIL_COL  = QtGui.QColor("#33ff55")
_DOT_COL    = QtGui.QColor("#88ffaa")
_TEXT_COL   = QtGui.QColor("#4dcc66")
_LABEL_COL  = QtGui.QColor("#2a7a3a")
_STEM_COL   = QtGui.QColor("#2a7a3a")

# Camera position in world space (rocket nadir is world origin).
_CAM_HEIGHT = 40.0      # metres above ground
_CAM_DIST   = 120.0     # metres behind nadir (−Z)

# Fraction of rocket altitude the camera look-target is raised to.
# 0.5 keeps the rocket roughly in the upper half of frame; 1.0 centres on it.
_VERT_BIAS  = 0.55

# Horizontal FOV degrees.
_FOV_DEG    = 60.0

# Grid
_GRID_DEPTH = 800.0
_GRID_WIDTH = 600.0
_GRID_COLS  = 12
_GRID_ROWS  = 14

_NEAR_CLIP  = 1.0
_MAX_TRAIL  = 120


# ---------------------------------------- #
# Camera / projection helpers               #
# ---------------------------------------- #


def _build_camera(rocket_alt: float) -> tuple[float, float, float, float, float]:
    """
    Return (cam_y, tilt_cos, tilt_sin, look_y).

    Camera sits at (0, _CAM_HEIGHT, -_CAM_DIST).
    It looks toward (0, rocket_alt * _VERT_BIAS, 0).
    Pitch angle is the angle between forward (+Z) and the look-vector.
    """
    look_y = rocket_alt * _VERT_BIAS
    # Vector from camera to target.
    dy = look_y - _CAM_HEIGHT
    dz = _CAM_DIST   # target is at z=0, camera at z=-_CAM_DIST → dz = +CAM_DIST
    pitch = math.atan2(dy, dz)   # positive = look up
    return _CAM_HEIGHT, math.cos(pitch), math.sin(pitch), look_y, pitch


def _project(
    wx: float, wy: float, wz: float,
    cam_height: float,
    tilt_cos: float, tilt_sin: float,
    f: float, cx: float, cy: float,
) -> tuple[float, float] | None:
    """
    Project world point (wx, wy, wz) with camera at (0, cam_height, -CAM_DIST).

    Translate to camera space, apply pitch rotation, then pin-hole project.
    """
    # Translate: camera at (0, cam_height, -_CAM_DIST)
    rx = wx
    ry = wy - cam_height
    rz = wz + _CAM_DIST   # camera is at z = -_CAM_DIST

    # Pitch rotation around X (positive pitch = look up):
    #   z_cam =  rz * cos + ry * sin
    #   y_cam = -rz * sin + ry * cos
    z_cam =  rz * tilt_cos + ry * tilt_sin
    y_cam = -rz * tilt_sin + ry * tilt_cos
    x_cam =  rx

    if z_cam < _NEAR_CLIP:
        return None

    sx = f * x_cam / z_cam + cx
    sy = -f * y_cam / z_cam + cy
    return sx, sy


# ---------------------------------------- #
# SimPanel                                  #
# ---------------------------------------- #


class SimPanel(QtWidgets.QWidget):
    """
    Flat infinite-plane 3-D position view.

    Camera follows the rocket's altitude, pitching up to keep it in frame.
    Renders a forward-looking perspective ground grid, a 3-D altitude stem
    with rocket dot, an optional GPS trail, and an altitude label.

    Call update_telemetry() to push live data; clear() to reset.
    """

    def __init__(self, parent=None):
        """Initialise with no telemetry data."""
        super().__init__(parent)
        self._telemetry: Telemetry | None = None
        self._trail: list[tuple[float, float, float]] = []
        self._origin: tuple[float, float] | None = None
        self.setMinimumSize(200, 150)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )

    # ---------------------------------------- #

    def update_telemetry(self, t: Telemetry) -> None:
        """Store latest telemetry and append to the GPS trail if fix is valid."""
        self._telemetry = t
        if t["gps_fix"]:
            if self._origin is None:
                self._origin = (t["lat"], t["lon"])
            olat, olon = self._origin
            dx_m = (t["lon"] - olon) * 111_000.0
            dz_m = (t["lat"] - olat) * 111_000.0
            self._trail.append((dx_m, dz_m, t["alt_m"]))
            if len(self._trail) > _MAX_TRAIL:
                self._trail.pop(0)
        self.update()

    # ---------------------------------------- #

    def clear(self) -> None:
        """Reset to idle state."""
        self._telemetry = None
        self._trail.clear()
        self._origin = None
        self.update()

    # ---------------------------------------- #

    def paintEvent(self, _e: QtGui.QPaintEvent) -> None:  # noqa: N802
        """Render the ground-plane grid and rocket position."""
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)

        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.5

        # --- Sky gradient -------------------------------------------------------
        grad = QtGui.QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0, _SKY_TOP)
        grad.setColorAt(1.0, _SKY_HORIZ)
        p.fillRect(self.rect(), grad)

        # --- Camera setup -------------------------------------------------------
        alt_m = self._telemetry["alt_m"] if self._telemetry else 0.0
        cam_height, tilt_cos, tilt_sin, look_y, pitch = _build_camera(alt_m)
        f = (w * 0.5) / math.tan(math.radians(_FOV_DEG * 0.5))

        def proj(wx: float, wy: float, wz: float) -> tuple[float, float] | None:
            return _project(wx, wy, wz, cam_height, tilt_cos, tilt_sin, f, cx, cy)

        # --- Horizon line -------------------------------------------------------
        horiz_pt = proj(0.0, 0.0, _GRID_DEPTH * 10)
        if horiz_pt:
            p.setPen(QtGui.QPen(_HORIZ_LINE, 1))
            p.drawLine(0, int(horiz_pt[1]), w, int(horiz_pt[1]))

        # --- Ground grid — Z rails ---------------------------------------------
        x_step = _GRID_WIDTH / _GRID_COLS
        half_cols = _GRID_COLS // 2
        z_near = 1.0
        z_far  = _GRID_DEPTH

        for ci in range(_GRID_COLS + 1):
            gx = -_GRID_WIDTH * 0.5 + ci * x_step
            edge_frac = abs(ci - half_cols) / half_cols if half_cols else 0.0
            alpha = int(130 * (1.0 - edge_frac * 0.55) + 20)
            col = QtGui.QColor(_GRID_BASE)
            col.setAlpha(alpha)
            p.setPen(QtGui.QPen(col, 1))
            pt_near = proj(gx, 0.0, z_near)
            pt_far  = proj(gx, 0.0, z_far)
            if pt_near and pt_far:
                p.drawLine(
                    QtCore.QPointF(pt_near[0], pt_near[1]),
                    QtCore.QPointF(pt_far[0],  pt_far[1]),
                )

        # --- Ground grid — cross lines -----------------------------------------
        for ri in range(1, _GRID_ROWS + 1):
            t_frac = ri / _GRID_ROWS
            gz = z_near + (z_far - z_near) * (t_frac ** 1.5)
            alpha = int(150 * (1.0 - t_frac * 0.65) + 20)
            col = QtGui.QColor(_GRID_BASE)
            col.setAlpha(alpha)
            p.setPen(QtGui.QPen(col, 1))
            pt_l = proj(-_GRID_WIDTH * 0.5, 0.0, gz)
            pt_r = proj( _GRID_WIDTH * 0.5, 0.0, gz)
            if pt_l and pt_r:
                p.drawLine(
                    QtCore.QPointF(pt_l[0], pt_l[1]),
                    QtCore.QPointF(pt_r[0], pt_r[1]),
                )

        # --- Nadir crosshair (ground directly below rocket) --------------------
        nadir_pt = proj(0.0, 0.0, 0.0)
        if nadir_pt:
            nx, ny = nadir_pt
            cross = 5
            p.setPen(QtGui.QPen(_LABEL_COL, 1))
            p.drawLine(int(nx - cross), int(ny), int(nx + cross), int(ny))
            p.drawLine(int(nx), int(ny - cross), int(nx), int(ny + cross))

            # --- Altitude stem: 3-D line from ground to rocket altitude --------
            rocket_pt = proj(0.0, alt_m, 0.0)
            if rocket_pt:
                rx, ry = rocket_pt

                # Dashed stem
                p.setPen(QtGui.QPen(_STEM_COL, 1, QtCore.Qt.PenStyle.DashLine))
                p.drawLine(int(nx), int(ny), int(rx), int(ry))

                # Rocket dot
                p.setPen(QtGui.QPen(_DOT_COL, 5))
                p.drawPoint(int(rx), int(ry))

                # Altitude label
                p.setPen(_TEXT_COL)
                p.setFont(QtGui.QFont("monospace", 8))
                label = f"{alt_m:.0f} m" if self._telemetry else "-- m"
                p.drawText(int(rx) + 6, int(ry) - 3, label)

        # --- GPS trail ---------------------------------------------------------
        if len(self._trail) >= 2:
            for i in range(1, len(self._trail)):
                dx0, dz0, a0 = self._trail[i - 1]
                dx1, dz1, a1 = self._trail[i]
                # Trail fades from old (dim) to new (bright)
                frac = i / len(self._trail)
                alpha = int(60 + frac * 160)
                col = QtGui.QColor(_TRAIL_COL)
                col.setAlpha(alpha)
                p.setPen(QtGui.QPen(col, 2))
                pt0 = proj(dx0, a0, dz0)
                pt1 = proj(dx1, a1, dz1)
                if pt0 and pt1:
                    p.drawLine(
                        QtCore.QPointF(pt0[0], pt0[1]),
                        QtCore.QPointF(pt1[0], pt1[1]),
                    )

        # --- "NO SIM DATA" label -----------------------------------------------
        if self._telemetry is None:
            p.setPen(_LABEL_COL)
            p.setFont(QtGui.QFont("monospace", 9))
            p.drawText(
                self.rect(),
                QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignTop,
                "NO SIM DATA",
            )
