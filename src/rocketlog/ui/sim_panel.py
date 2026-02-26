import math

from PySide6 import QtCore, QtGui, QtWidgets

from rocketlog.telemetry.types import Telemetry


# ---------------------------------------- #
# Flat-plane perspective simulation panel   #
# ---------------------------------------- #
#
# The ground is an infinite flat plane viewed from a fixed camera height.
# Grid lines are projected with a standard pin-hole model:
#
#   sx = f * x / z + cx
#   sy = f * y / z + cy
#
# Coordinate system
# -----------------
# Camera is at (0, _CAM_HEIGHT, 0) looking in the +Z direction (forward)
# and slightly downward by _TILT_DEG. +X is right, +Y is up.
# The ground plane is y = 0; the camera is above it.
#
# Grid
# ----
# Z rails run from near-clip to _GRID_DEPTH.
# X cross-lines are spaced evenly across _GRID_WIDTH at fixed Z intervals.

_SKY_TOP    = QtGui.QColor("#060d06")
_SKY_HORIZ  = QtGui.QColor("#0d200d")
_GRID_BASE  = QtGui.QColor(0x1a, 0x4a, 0x1a)   # dim green
_HORIZ_LINE = QtGui.QColor(0x2a, 0x7a, 0x2a)   # brighter horizon line
_TRAIL_COL  = QtGui.QColor("#33ff55")
_DOT_COL    = QtGui.QColor("#88ffaa")
_TEXT_COL   = QtGui.QColor("#4dcc66")
_LABEL_COL  = QtGui.QColor("#2a7a3a")

# Camera
_CAM_HEIGHT = 10.0      # metres above ground plane
_TILT_DEG   = 20.0      # degrees below horizon the camera points

# Grid extents (metres in world space)
_GRID_DEPTH = 600.0     # how far forward the grid extends
_GRID_WIDTH = 400.0     # total lateral width of the grid
_GRID_COLS  = 10        # number of Z-rail lines (left/right of centre)
_GRID_ROWS  = 12        # number of cross-lines (depth slices)

_NEAR_CLIP  = 0.5       # minimum z_cam before clipping
_MAX_TRAIL  = 80


# ---------------------------------------- #
# Projection helpers                        #
# ---------------------------------------- #

def _project_point(
    wx: float, wy: float, wz: float,
    cam_height: float, tilt_cos: float, tilt_sin: float,
    f: float, cx: float, cy: float,
) -> tuple[float, float] | None:
    """
    Project a world point (wx, wy, wz) onto the screen.

    Camera is at (0, cam_height, 0). The view rotates by tilt around X so
    that the camera looks forward-and-down.

        rx = wx
        ry = wy - cam_height
        rz = wz  (world forward = camera forward before tilt)

    Tilt rotation (pitch down):
        z_cam =  rz * tilt_cos + ry * tilt_sin
        y_cam = -rz * tilt_sin + ry * tilt_cos

    Returns (sx, sy) or None if behind the camera.
    """
    rx = wx
    ry = wy - cam_height
    rz = wz

    z_cam =  rz * tilt_cos + ry * tilt_sin
    y_cam = -rz * tilt_sin + ry * tilt_cos
    x_cam =  rx

    if z_cam < _NEAR_CLIP:
        return None

    sx = f * x_cam / z_cam + cx
    sy = -f * y_cam / z_cam + cy   # screen Y inverted
    return sx, sy


# ---------------------------------------- #
# SimPanel                                  #
# ---------------------------------------- #


class SimPanel(QtWidgets.QWidget):
    """
    Flat infinite-plane 3-D position view.

    Renders a forward-looking perspective grid over a flat ground plane,
    a nadir crosshair, a dashed altitude stem with rocket dot, and an
    optional GPS trail.

    Call update_telemetry() to push live data; clear() to reset.
    """

    def __init__(self, parent=None):
        """Initialise the panel with no telemetry data."""
        super().__init__(parent)
        self._telemetry: Telemetry | None = None
        # Trail: list of (dx_m, dz_m, alt_m) offsets from the first GPS fix.
        self._trail: list[tuple[float, float, float]] = []
        self._origin: tuple[float, float] | None = None
        self.setMinimumSize(200, 150)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )

    # ---------------------------------------- #

    def update_telemetry(self, t: Telemetry) -> None:
        """Store the latest telemetry and append to the GPS trail if fix is valid."""
        self._telemetry = t
        if t["gps_fix"]:
            if self._origin is None:
                self._origin = (t["lat"], t["lon"])
            olat, olon = self._origin
            dx_m = (t["lon"] - olon) * 111_000.0   # easting
            dz_m = (t["lat"] - olat) * 111_000.0   # northing → world Z
            self._trail.append((dx_m, dz_m, t["alt_m"]))
            if len(self._trail) > _MAX_TRAIL:
                self._trail.pop(0)
        self.update()

    # ---------------------------------------- #

    def clear(self) -> None:
        """Reset to idle state (no telemetry, empty trail)."""
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

        # --- Sky gradient ---------------------------------------------------
        grad = QtGui.QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0, _SKY_TOP)
        grad.setColorAt(1.0, _SKY_HORIZ)
        p.fillRect(self.rect(), grad)

        # --- Camera setup ---------------------------------------------------
        tilt_rad  = math.radians(_TILT_DEG)
        tilt_cos  = math.cos(tilt_rad)
        tilt_sin  = math.sin(tilt_rad)
        f = (w * 0.5) / math.tan(math.radians(60.0 * 0.5))   # 60° horizontal FOV

        def proj(wx: float, wy: float, wz: float) -> tuple[float, float] | None:
            return _project_point(wx, wy, wz, _CAM_HEIGHT, tilt_cos, tilt_sin, f, cx, cy)

        # --- Horizon line ---------------------------------------------------
        # The horizon is where the ground plane meets the camera's view ray at
        # y=0. Find its screen Y by projecting a far point on y=0.
        horiz_pt = proj(0.0, 0.0, _GRID_DEPTH * 10)
        if horiz_pt:
            hy = horiz_pt[1]
            p.setPen(QtGui.QPen(_HORIZ_LINE, 1))
            p.drawLine(0, int(hy), w, int(hy))

        # --- Z-rail lines (running away from camera) ------------------------
        half_cols = _GRID_COLS // 2
        x_step    = _GRID_WIDTH / _GRID_COLS
        z_near    = _CAM_HEIGHT * 0.5   # start just in front of camera foot
        z_far     = _GRID_DEPTH

        for ci in range(_GRID_COLS + 1):
            gx = -_GRID_WIDTH * 0.5 + ci * x_step
            # Fade rails toward the edges.
            edge_frac = abs(ci - half_cols) / half_cols if half_cols else 0.0
            alpha = int(140 * (1.0 - edge_frac * 0.6) + 20)
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

        # --- Cross-lines (lateral, at increasing Z depths) ------------------
        for ri in range(1, _GRID_ROWS + 1):
            # Space rows with perspective-aware Z so they look evenly spaced.
            t_frac = ri / _GRID_ROWS
            gz = z_near + (z_far - z_near) * (t_frac ** 1.6)
            # Fade rows toward the horizon.
            alpha = int(160 * (1.0 - t_frac * 0.7) + 20)
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

        # --- Nadir crosshair + rocket stem + dot ----------------------------
        # Nadir on ground is directly below camera: world (0, 0, 0)… but the
        # camera is at z=0 looking forward, so project a point just ahead on
        # the ground to get the "foot" of the rocket.
        nadir_world_z = z_near * 2   # a short distance ahead on the ground
        nadir_pt = proj(0.0, 0.0, nadir_world_z)
        if nadir_pt:
            nx, ny = nadir_pt
            cross = 4
            p.setPen(QtGui.QPen(_LABEL_COL, 1))
            p.drawLine(int(nx - cross), int(ny), int(nx + cross), int(ny))
            p.drawLine(int(nx), int(ny - cross), int(nx), int(ny + cross))

            # Altitude maps to screen-space pixels upward from nadir foot.
            alt_m  = self._telemetry["alt_m"] if self._telemetry else 0.0
            alt_px = (alt_m / 2000.0) * h * 0.40
            rx, ry = nx, ny - alt_px

            p.setPen(QtGui.QPen(_LABEL_COL, 1, QtCore.Qt.PenStyle.DashLine))
            p.drawLine(int(nx), int(ny), int(rx), int(ry))

            p.setPen(QtGui.QPen(_DOT_COL, 4))
            p.drawPoint(int(rx), int(ry))

            p.setPen(_TEXT_COL)
            p.setFont(QtGui.QFont("monospace", 8))
            alt_label = f"{alt_m:.0f} m" if self._telemetry else "-- m"
            p.drawText(int(rx) + 5, int(ry) - 3, alt_label)

        # --- GPS trail ------------------------------------------------------
        if len(self._trail) >= 2:
            p.setPen(QtGui.QPen(_TRAIL_COL, 2))
            for i in range(1, len(self._trail)):
                dx0, dz0, a0 = self._trail[i - 1]
                dx1, dz1, a1 = self._trail[i]
                pt0 = proj(dx0, a0, dz0 + nadir_world_z)
                pt1 = proj(dx1, a1, dz1 + nadir_world_z)
                if pt0 and pt1:
                    p.drawLine(
                        QtCore.QPointF(pt0[0], pt0[1]),
                        QtCore.QPointF(pt1[0], pt1[1]),
                    )

        # --- "NO SIM DATA" label --------------------------------------------
        if self._telemetry is None:
            p.setPen(_LABEL_COL)
            p.setFont(QtGui.QFont("monospace", 9))
            p.drawText(
                self.rect(),
                QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignTop,
                "NO SIM DATA",
            )
