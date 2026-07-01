"""
3D rocket attitude display -- MATLAB-style software renderer.
No OpenGL required: uses numpy + QPainter.

Body frame: +Z = nose, +X = starboard, +Y = up-body.
Camera: az=0 deg, el=20 deg so Z projects straight up on screen.
"""

import math
import numpy as np

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore    import Qt, QPointF
from PyQt6.QtGui     import (
    QPainter, QColor, QPen, QBrush, QPolygonF, QFont
)

from core.packet_decoder import TelemetryData

# ── Palette ───────────────────────────────────────────────────────────────────
_BG     = QColor(15, 15, 16)
_GRID   = QColor(44, 44, 52, 160)
_BOX    = QColor(62, 62, 75, 200)
_TICK   = QColor(78, 78, 88, 200)
_AXIS_X = QColor(239, 68,  68)
_AXIS_Y = QColor(34,  197, 94)
_AXIS_Z = QColor(59,  130, 246)
_TEXT   = QColor(241, 245, 249)
_MUTED  = QColor(71,  85,  105)

# ── Scene box ─────────────────────────────────────────────────────────────────
_BX = 0.72
_ZL = -0.65
_ZH =  1.05

# ── Geometry parameters ───────────────────────────────────────────────────────
_N      = 16
_R_BODY = 0.10
_Z_TIP  = 0.95
_Z_TOP  = 0.50
_Z_BOT  = -0.45
_Z_NOZ  = -0.60
_R_NOZ  = 0.062
_FIN_R  = 0.36

# ── Camera ────────────────────────────────────────────────────────────────────
def _view_matrix(az_deg: float = 0.0, el_deg: float = 20.0) -> np.ndarray:
    az, el = math.radians(az_deg), math.radians(el_deg)
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)
    Ry = np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])
    Rx = np.array([[1, 0, 0], [0, ce, -se], [0, se, ce]])
    return Rx @ Ry

_VM = _view_matrix()

_LT = np.array([0.8, -0.3, 1.0])
_LT /= np.linalg.norm(_LT)


# ── Geometry ──────────────────────────────────────────────────────────────────
def _ring(r: float, z: float) -> np.ndarray:
    a = np.linspace(0.0, 2.0 * math.pi, _N, endpoint=False)
    return np.column_stack([r * np.cos(a), r * np.sin(a), np.full(_N, z)])


def _build_faces():
    top = _ring(_R_BODY, _Z_TOP)
    bot = _ring(_R_BODY, _Z_BOT)
    noz = _ring(_R_NOZ,  _Z_NOZ)
    tip = np.array([0.0, 0.0, _Z_TIP])
    nc  = np.array([0.0, 0.0, _Z_NOZ])

    BODY = QColor(232, 236, 245)   # near-white body
    NOSE = QColor(205, 210, 225)   # slightly darker nose cone
    BAND = QColor(50,  110, 200)   # blue accent band (top 18% of tube)
    NOZ  = QColor(72,  76,  95)    # dark metallic nozzle
    CAP  = QColor(50,  52,  68)    # nozzle cap
    FIN  = QColor(198, 203, 218)   # SILVER fins -- matches body family

    faces = []

    for i in range(_N):
        j = (i + 1) % _N
        band_z = _Z_TOP - (_Z_TOP - _Z_BOT) * 0.18
        c = BAND if (top[i][2] + bot[i][2]) / 2 > band_z else BODY
        faces.append((np.array([top[i], top[j], bot[j], bot[i]]), c))

    for i in range(_N):
        j = (i + 1) % _N
        faces.append((np.array([tip, top[j], top[i]]), NOSE))

    for i in range(_N):
        j = (i + 1) % _N
        faces.append((np.array([bot[i], bot[j], noz[j], noz[i]]), NOZ))

    for i in range(_N):
        j = (i + 1) % _N
        faces.append((np.array([nc, noz[i], noz[j]]), CAP))

    for fa in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
        ca, sa = math.cos(fa), math.sin(fa)
        v0 = np.array([_R_BODY * ca, _R_BODY * sa, _Z_TOP - 0.12])
        v1 = np.array([_R_BODY * ca, _R_BODY * sa, _Z_BOT])
        v2 = np.array([_FIN_R  * ca, _FIN_R  * sa, _Z_BOT])
        v3 = np.array([_FIN_R  * ca, _FIN_R  * sa, _Z_TOP - 0.32])
        faces.append((np.array([v0, v1, v2, v3]), FIN))

    return faces


_FACES = _build_faces()


# ── Helpers ───────────────────────────────────────────────────────────────────
def _qrot(w, x, y, z) -> np.ndarray:
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def _normal(v) -> np.ndarray:
    n = np.cross(v[1] - v[0], v[2] - v[0])
    d = np.linalg.norm(n)
    return n / d if d > 1e-9 else np.array([0.0, 0.0, 1.0])


def _shade(col: QColor, n_world: np.ndarray, ambient: float = 0.35) -> QColor:
    d = max(0.0, float(np.dot(n_world, _LT)))
    t = min(1.0, ambient + (1.0 - ambient) * d)
    return QColor(int(col.red()*t), int(col.green()*t), int(col.blue()*t))


def _wp(pt3d, sc: float, cx: float, cy: float) -> tuple[float, float]:
    vv = _VM @ np.asarray(pt3d, float)
    return cx + vv[0] * sc, cy - vv[2] * sc


def _proj(verts, sc, cx, cy):
    vv  = verts @ _VM.T
    sx  = vv[:, 0] * sc + cx
    sy  = -vv[:, 2] * sc + cy
    dep = float(np.mean(vv[:, 1]))
    return sx, sy, dep


# ── Widget ────────────────────────────────────────────────────────────────────
class RocketVisual(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 240)
        self._w, self._x, self._y, self._z = 1.0, 0.0, 0.0, 0.0
        self._state = 0

    def update_data(self, data: TelemetryData) -> None:
        self._w = data.quat_w
        self._x = data.quat_x
        self._y = data.quat_y
        self._z = data.quat_z
        self._state = data.state
        self.update()

    # ── Paint ─────────────────────────────────────────────────────────────────
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()

        p.fillRect(0, 0, W, H, _BG)

        READ_W   = 82
        RENDER_W = W - READ_W

        cx = RENDER_W / 2
        cy = H / 2 + H * 0.05
        sc = min(RENDER_W, H) * 0.32

        self._draw_scene(p, sc, cx, cy)

        R = _qrot(self._w, self._x, self._y, self._z)
        draw = []
        for verts, col in _FACES:
            rv          = verts @ R.T
            sx, sy, dep = _proj(rv, sc, cx, cy)
            n           = _normal(rv)
            shade       = _shade(col, n)
            pts         = [QPointF(float(sx[i]), float(sy[i])) for i in range(len(sx))]
            draw.append((dep, pts, shade))

        draw.sort(key=lambda t: t[0])
        for _, pts, col in draw:
            p.setPen(QPen(col.darker(108), 0.5))
            p.setBrush(QBrush(col))
            p.drawPolygon(QPolygonF(pts))

        if self._state == 2:
            self._draw_flame(p, R, sc, cx, cy)

        # Static world-frame triad (axes never rotate)
        self._draw_triad(p, RENDER_W - 50, 50, 34)

        self._draw_readout(p, W - READ_W, 0, READ_W, H)

        p.end()

    # ── Scene frame with MATLAB-style tick marks ───────────────────────────────
    def _draw_scene(self, p: QPainter, sc: float, cx: float, cy: float) -> None:
        B, ZL, ZH = _BX, _ZL, _ZH

        corners = [
            [-B, -B, ZL], [B, -B, ZL], [B, B, ZL], [-B, B, ZL],
            [-B, -B, ZH], [B, -B, ZH], [B, B, ZH], [-B, B, ZH],
        ]
        sc_pts = [_wp(c, sc, cx, cy) for c in corners]

        p.setPen(QPen(_BOX, 1.0))
        for i, j in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
            p.drawLine(QPointF(*sc_pts[i]), QPointF(*sc_pts[j]))

        # Floor grid
        p.setPen(QPen(_GRID, 0.7))
        N = 6
        for k in range(N + 1):
            t = -B + 2 * B * k / N
            x1, y1 = _wp([-B, t, ZL], sc, cx, cy)
            x2, y2 = _wp([ B, t, ZL], sc, cx, cy)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            x1, y1 = _wp([t, -B, ZL], sc, cx, cy)
            x2, y2 = _wp([t,  B, ZL], sc, cx, cy)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Z axis tick marks with numbers (left-front vertical edge)
        p.setFont(QFont('JetBrains Mono', 7))
        p.setPen(_TICK)
        z_ticks = [(0, ZL), (50, ZL+(ZH-ZL)*0.25), (100, ZL+(ZH-ZL)*0.5),
                   (150, ZL+(ZH-ZL)*0.75), (200, ZH)]
        for lbl, z in z_ticks:
            tx, ty = _wp([-B - 0.06, -B, z], sc, cx, cy)
            p.drawText(QPointF(tx - 26, ty + 4), str(lbl))

        # X axis tick marks (front-bottom edge)
        for lbl, x in [(-50, -B), (0, 0.0), (50, B)]:
            tx, ty = _wp([x, -B - 0.02, ZL + 0.02], sc, cx, cy)
            p.drawText(QPointF(tx - 8, ty + 13), str(lbl))

        # Y axis tick marks (left-bottom edge)
        for lbl, y in [(-50, -B), (0, 0.0), (50, B)]:
            tx, ty = _wp([-B - 0.04, y, ZL + 0.02], sc, cx, cy)
            p.drawText(QPointF(tx - 26, ty + 10), str(lbl))

        # Axis labels
        p.setFont(QFont('JetBrains Mono', 8, QFont.Weight.Bold))
        p.setPen(_MUTED)
        zx, zy = _wp([-B - 0.16, -B, (ZL + ZH) / 2 + 0.1], sc, cx, cy)
        p.drawText(QPointF(zx - 18, zy), 'Z (m)')
        xx, xy = _wp([0, -B - 0.12, ZL - 0.10], sc, cx, cy)
        p.drawText(QPointF(xx - 12, xy), 'X (m)')
        yx, yy = _wp([-B - 0.04, 0, ZL - 0.14], sc, cx, cy)
        p.drawText(QPointF(yx - 24, yy), 'Y (m)')

    # ── Static world-frame axis triad ──────────────────────────────────────────
    def _draw_triad(self, p: QPainter, ax: float, ay: float, ln: float) -> None:
        axes = [
            ('Z', np.array([0.0, 0.0, 1.0]), _AXIS_Z),
            ('Y', np.array([0.0, 1.0, 0.0]), _AXIS_Y),
            ('X', np.array([1.0, 0.0, 0.0]), _AXIS_X),
        ]
        for label, unit_v, col in axes:
            vv = _VM @ unit_v   # fixed world frame -- no rocket rotation
            ex = ax + vv[0] * ln
            ey = ay - vv[2] * ln
            p.setPen(QPen(col, 2.0))
            p.drawLine(QPointF(ax, ay), QPointF(float(ex), float(ey)))
            p.setFont(QFont('Segoe UI', 8, QFont.Weight.Bold))
            p.setPen(col)
            p.drawText(QPointF(float(ex) + 2, float(ey) + 4), label)

    # ── Flame ─────────────────────────────────────────────────────────────────
    def _draw_flame(self, p: QPainter, R: np.ndarray,
                    sc: float, cx: float, cy: float) -> None:
        nw  = R @ np.array([0.0, 0.0, _Z_NOZ])
        nvv = _VM @ nw
        ox, oy = cx + nvv[0] * sc, cy - nvv[2] * sc

        ew  = R @ np.array([0.0, 0.0, -1.0])
        evv = _VM @ ew
        fl  = sc * 0.50
        fdx, fdy = evv[0] * fl, -evv[2] * fl
        tx, ty = ox + fdx, oy + fdy

        hyp = math.hypot(fdx, fdy) + 1e-9
        px, py = -fdy / hyp, fdx / hyp
        fw = sc * 0.055

        pts = QPolygonF([
            QPointF(ox + px*fw,       oy + py*fw),
            QPointF(ox - px*fw,       oy - py*fw),
            QPointF(tx - px*fw*0.18,  ty - py*fw*0.18),
            QPointF(tx,               ty),
            QPointF(tx + px*fw*0.18,  ty + py*fw*0.18),
        ])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(255, 148, 18, 210)))
        p.drawPolygon(pts)

    # ── Angle readout panel ───────────────────────────────────────────────────
    def _draw_readout(self, p: QPainter, rx: float, ry: float,
                      rw: float, rh: float) -> None:
        w, x, y, z = self._w, self._x, self._y, self._z
        sinp  = 2.0 * (w * y - z * x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        roll  = math.degrees(math.atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)))
        yaw   = math.degrees(math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z)))

        p.setPen(QPen(QColor(30, 31, 42), 1))
        p.drawLine(QPointF(rx, ry), QPointF(rx, ry + rh))

        labels = [('ROLL', roll), ('PITCH', pitch), ('YAW', yaw)]
        spacing = rh / (len(labels) + 1)

        for i, (label, val) in enumerate(labels):
            cy_v = ry + spacing * (i + 1)

            p.setFont(QFont('Segoe UI', 8, QFont.Weight.Bold))
            p.setPen(_MUTED)
            p.drawText(QPointF(rx + 10, cy_v - 8), label)

            p.setFont(QFont('JetBrains Mono', 12, QFont.Weight.Bold))
            p.setPen(_TEXT)
            p.drawText(QPointF(rx + 10, cy_v + 8), f'{val:+.1f}°')
