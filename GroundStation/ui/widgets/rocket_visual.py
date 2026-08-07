"""
3D rocket attitude display -- a hand-rolled software renderer.
No OpenGL required: uses numpy + QPainter.

Body frame: +Z = nose, +X = starboard, +Y = up-body.
Camera: az=0 deg, el=20 deg so Z projects straight up on screen.
"""

import math
import os
import struct
import time
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

# ── Real CAD model (optional) ────────────────────────────────────────────────
# If assets/rocket.stl exists, it's loaded, auto-oriented, and decimated to
# replace the hand-coded geometry below. Falls back to the procedural rocket
# (see _build_faces) if the file is missing or fails to parse, so a checkout
# without the (large, binary) STL asset still runs.
_STL_PATH          = os.path.join(os.path.dirname(__file__), '..', '..', 'assets', 'rocket.stl')
_STL_TARGET_LENGTH = _Z_TIP - _Z_NOZ   # normalize to the same nose-to-nozzle span as the placeholder
# A first pass decimated down to ~1800 faces (a ~15x cut) and caused visible
# terracing on the nose cone's curve -- grid-snap decimation quantizes vertex
# positions, which bands badly on tightly curved surfaces. A second pass
# skipped decimation entirely (renders the real triangle count, ~26.7k) to
# fix that, which fixed the curve but made per-frame rendering slow enough to
# stall the whole GUI (Qt painting runs on the main thread -- a slow
# paintEvent freezes button clicks too, not just the 3D view). 6000 is a
# measured middle ground: comparing this project's actual STL's radius
# profile before/after decimation, 6000 faces keeps max deviation from full
# resolution to ~1/4 of what the visibly-stepped 1800-face version had.
_STL_TARGET_FACES  = 6000
_STL_COLOR         = QColor(228, 232, 242)

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


# ── STL import ────────────────────────────────────────────────────────────────
def _read_stl_triangles(path: str) -> np.ndarray:
    """Parse a binary or ASCII STL into an (N, 3, 3) array of triangle corners."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError('file too small to be an STL')

    # Binary STL: 80-byte header + uint32 triangle count + 50 bytes/triangle.
    # Checking the file size against that formula is more reliable than
    # sniffing for a "solid" header, since binary files can coincidentally
    # start with that word too.
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) == 84 + count * 50:
        body  = data[84:84 + count * 50]
        raw   = np.frombuffer(body, dtype=np.uint8).reshape(count, 50)
        block = raw[:, 0:48].copy().view(np.float32).reshape(count, 4, 3)
        return block[:, 1:4, :].astype(np.float64)   # drop the stored facet normal, keep the 3 verts

    # Fall back to ASCII STL ("solid ... facet normal ... vertex x y z ...")
    verts, cur = [], []
    for line in data.decode('ascii', errors='ignore').splitlines():
        line = line.strip()
        if line.startswith('vertex'):
            parts = line.split()
            cur.append([float(parts[1]), float(parts[2]), float(parts[3])])
            if len(cur) == 3:
                verts.append(cur)
                cur = []
    if not verts:
        raise ValueError('no triangles found -- not a valid STL')
    return np.array(verts, dtype=np.float64)


def _orient_and_normalize(tris: np.ndarray, target_length: float) -> np.ndarray:
    """
    Auto-detect the rocket's roll axis and nose direction, then rotate/center/
    scale into this renderer's body frame (+Z = nose, origin at the model's
    bounding-box center). Assumes the CAD export was modeled along one axis of
    revolution (true for essentially any nose-cone/body-tube assembly) --
    that axis is just whichever bounding-box dimension is longest. Which END
    of it is the nose is auto-detected by comparing the cross-sectional
    radius near each end: the nose tapers to a point, so it has the smaller
    radius. Re-run this if a future STL re-export uses a different axis --
    no code changes needed, it re-detects every load.
    """
    pts    = tris.reshape(-1, 3)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    extent = mx - mn
    long_axis   = int(np.argmax(extent))
    other_axes  = [a for a in range(3) if a != long_axis]

    axis_vals = pts[:, long_axis]
    radius    = np.linalg.norm(pts[:, other_axes] - pts[:, other_axes].mean(axis=0), axis=1)
    lo_cut    = mn[long_axis] + extent[long_axis] * 0.05
    hi_cut    = mx[long_axis] - extent[long_axis] * 0.05
    lo_mask, hi_mask = axis_vals <= lo_cut, axis_vals >= hi_cut
    r_lo = np.median(radius[lo_mask]) if lo_mask.any() else float('inf')
    r_hi = np.median(radius[hi_mask]) if hi_mask.any() else float('inf')
    nose_at_hi = r_hi < r_lo

    # Permutation mapping (other0, other1, long_axis) -> (X, Y, Z), i.e. long
    # axis becomes Z. If the nose ended up at -Z, flip Z and X together (not
    # just Z) so the result stays a proper rotation (det +1) instead of a
    # mirror, which would turn every face inside-out under the shading model.
    perm = other_axes + [long_axis]
    M = np.zeros((3, 3))
    for new_axis, old_axis in enumerate(perm):
        M[new_axis, old_axis] = 1.0
    if not nose_at_hi:
        M = np.diag([-1.0, 1.0, -1.0]) @ M

    rotated = tris @ M.T
    rpts    = rotated.reshape(-1, 3)
    rmn, rmx = rpts.min(axis=0), rpts.max(axis=0)
    scale   = target_length / (rmx[2] - rmn[2])
    center  = (rmn + rmx) / 2.0
    return (rotated - center) * scale


def _decimate(tris: np.ndarray, target_faces: int, max_iters: int = 14) -> np.ndarray:
    """
    Vertex-clustering decimation: snap all corners to a grid, drop triangles
    that collapsed to a line/point, dedupe. A CAD export easily carries tens
    of thousands of triangles -- redrawing that many individual QPainter
    polygons every telemetry update would be a visible stutter, so this trims
    it down to roughly target_faces while keeping the overall silhouette
    (verified against this project's actual STL: shape held up fine down to
    ~1800 faces from 26k). Not a substitute for a real simplification
    algorithm, but the body/nose/fins here are simple enough surfaces that
    grid-snapping alone looks fine, and it needs no extra dependency.
    """
    if len(tris) <= target_faces:
        return tris

    pts  = tris.reshape(-1, 3)
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    lo, hi = diag / 4000.0, diag / 4.0
    kept = tris
    for _ in range(max_iters):
        cell = math.sqrt(lo * hi)
        snapped = np.round(tris / cell) * cell
        v0, v1, v2 = snapped[:, 0], snapped[:, 1], snapped[:, 2]
        degenerate = (np.all(v0 == v1, axis=1) | np.all(v1 == v2, axis=1) | np.all(v0 == v2, axis=1))
        candidate  = snapped[~degenerate]
        if len(candidate) == 0:
            hi = cell
            continue
        _, unique_idx = np.unique(candidate.reshape(len(candidate), 9), axis=0, return_index=True)
        candidate = candidate[unique_idx]
        kept = candidate
        if abs(len(candidate) - target_faces) <= max(target_faces * 0.15, 50):
            break
        lo, hi = (cell, hi) if len(candidate) > target_faces else (lo, cell)

    # Slivers can survive the pairwise-equality check above if all 3 corners
    # snapped to distinct-but-collinear points -- drop by area instead.
    areas = 0.5 * np.linalg.norm(np.cross(kept[:, 1] - kept[:, 0], kept[:, 2] - kept[:, 0]), axis=1)
    return kept[areas > (diag ** 2) * 1e-7]


def _load_stl_faces(path: str):
    """Returns a _FACES-shaped list (verts, color) built from the STL at
    path, or None if the file is missing/unreadable -- caller falls back to
    the procedural model rather than crashing the whole ground station over
    a missing or corrupt asset."""
    if not os.path.isfile(path):
        return None
    try:
        tris = _read_stl_triangles(path)
        tris = _orient_and_normalize(tris, _STL_TARGET_LENGTH)
        tris = _decimate(tris, _STL_TARGET_FACES)
        return [(tri, _STL_COLOR) for tri in tris]
    except Exception as e:
        print(f'[ROCKET_VIS] Failed to load {path}: {e} -- using procedural placeholder model.')
        return None


_FACES = _load_stl_faces(_STL_PATH) or _build_faces()

# If every face is a triangle sharing one base color (true for the STL path,
# never for the procedural fallback's mixed quads/tris and per-part colors),
# precompute a single stacked array so paintEvent can transform/shade all
# faces in a few batched numpy calls instead of ~6 small numpy calls run
# individually per face. At a few thousand faces that per-face Python/numpy
# call overhead was the dominant cost -- see _STL_TARGET_FACES above for the
# other half of this fix (fewer faces to begin with).
_FACES_TRIS     = None   # (N, 3, 3) or None
_FACES_BASE_RGB = None   # (3,) or None
if _FACES and all(v.shape == (3, 3) for v, _ in _FACES):
    _distinct_colors = {c.getRgb() for _, c in _FACES}
    if len(_distinct_colors) == 1:
        _FACES_TRIS     = np.stack([v for v, _ in _FACES])
        _FACES_BASE_RGB = np.array(_FACES[0][1].getRgb()[:3], dtype=np.float64)


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


def _transform_batch(faces_tris: np.ndarray, base_rgb: np.ndarray, R: np.ndarray,
                      sc: float, cx: float, cy: float, ambient: float = 0.35):
    """
    Same math as _proj/_normal/_shade, run once across all N faces via numpy
    instead of once per face in a Python loop. Returns (sx, sy, dep, rgb) --
    each an (N,3) or (N,) array -- for the caller to sort by depth and draw.
    """
    rv = faces_tris @ R.T                    # (N,3,3) body -> world
    cv = rv @ _VM.T                          # (N,3,3) world -> camera

    sx  = cv[:, :, 0] * sc + cx              # (N,3)
    sy  = -cv[:, :, 2] * sc + cy             # (N,3)
    dep = cv[:, :, 1].mean(axis=1)           # (N,)

    v0, v1, v2 = rv[:, 0, :], rv[:, 1, :], rv[:, 2, :]
    raw_n  = np.cross(v1 - v0, v2 - v0)
    n_len  = np.linalg.norm(raw_n, axis=1)
    valid  = n_len > 1e-9
    n_all  = np.tile(np.array([0.0, 0.0, 1.0]), (len(raw_n), 1))
    n_all[valid] = raw_n[valid] / n_len[valid, None]

    d = np.clip(n_all @ _LT, 0.0, None)
    t = np.minimum(1.0, ambient + (1.0 - ambient) * d)
    rgb = np.clip(base_rgb[None, :] * t[:, None], 0, 255)

    return sx, sy, dep, rgb


# ── Widget ────────────────────────────────────────────────────────────────────
class RocketVisual(QWidget):
    # Caps actual redraws, independent of how often update_data() is called.
    # Telemetry can now arrive at up to 20Hz over a direct USB link (see
    # USB_TELEMETRY_INTERVAL_MS in the firmware) -- but this is the one
    # widget in the app redrawing a full CAD mesh (tens of thousands of
    # triangles) via plain QPainter, not GPU-accelerated. 15fps is well past
    # what's visually distinguishable on an attitude indicator anyway, and
    # decoupling it here means a fast feed can't stutter the rest of the UI.
    # The latest orientation is still stored on every call, so nothing is
    # lost -- only how often it's actually painted is capped.
    _MIN_REPAINT_INTERVAL_S = 1.0 / 15.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 240)
        self._w, self._x, self._y, self._z = 1.0, 0.0, 0.0, 0.0
        self._state = 0
        self._last_repaint = 0.0

    def update_data(self, data: TelemetryData) -> None:
        self._w = data.quat_w
        self._x = data.quat_x
        self._y = data.quat_y
        self._z = data.quat_z
        self._state = data.state

        now = time.monotonic()
        if now - self._last_repaint >= self._MIN_REPAINT_INTERVAL_S:
            self._last_repaint = now
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

        if _FACES_TRIS is not None:
            # Fast path: all faces transformed/shaded in a handful of batched
            # numpy calls instead of several per face -- see _transform_batch.
            sx, sy, dep, rgb = _transform_batch(_FACES_TRIS, _FACES_BASE_RGB, R, sc, cx, cy)
            order = np.argsort(dep)
            for i in order:
                col = QColor(int(rgb[i, 0]), int(rgb[i, 1]), int(rgb[i, 2]))
                p.setPen(QPen(col.darker(108), 0.5))
                p.setBrush(QBrush(col))
                pts = QPolygonF([QPointF(float(sx[i, j]), float(sy[i, j])) for j in range(3)])
                p.drawPolygon(pts)
        else:
            # Procedural fallback: few dozen faces, mixed tri/quad, per-part
            # colors -- not worth the batching machinery above.
            draw = []
            for verts, col in _FACES:
                rv          = verts @ R.T
                sxf, syf, d = _proj(rv, sc, cx, cy)
                n           = _normal(rv)
                shade       = _shade(col, n)
                pts         = [QPointF(float(sxf[i]), float(syf[i])) for i in range(len(sxf))]
                draw.append((d, pts, shade))

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

    # ── Scene frame with axis tick marks ───────────────────────────────────────
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
