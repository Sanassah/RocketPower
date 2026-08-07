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

# Raw-quaternion correction: the BNO085 reports orientation in its OWN chip-
# frame axes, which were never verified to actually line up with this
# renderer's assumed body frame (+Z=nose, +X=starboard, +Y=up-body -- see
# the module docstring). Same category of problem as ATTITUDE_ROLL_RATE/etc
# in the firmware's config.h, just on the ground-station rendering side
# instead of the firmware control side.
#
# Bench-confirmed: pitch and yaw are correct as raw data comes in. Roll
# comes in backwards (rolling right reads negative and the mesh visibly
# rolls left) -- but negating a single raw component to fix it (an earlier
# version of this function did that) turned out to be invalid: it's not a
# real rotation/reflection of 3D space, just an ad-hoc tweak that happens to
# look right for an isolated roll-from-identity test and breaks under
# composition -- confirmed by it reintroducing pitch/roll cross-talk after
# calibration. There IS no single linear operation on the quaternion that
# flips one Euler angle's sign while leaving the other two alone, for every
# orientation -- Euler angles aren't independent coordinates the way X/Y/Z
# are. See _flip_roll_sign() below for the fix that actually holds up under
# composition: decompose to (roll, pitch, yaw), negate roll, recompose.
def _correct_quat(w: float, x: float, y: float, z: float) -> tuple[float, float, float, float]:
    return (w, x, y, z)


# ── Quaternion helpers (Hamilton product / conjugate) ────────────────────────
def _qconj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _qmul(q1: tuple[float, float, float, float],
          q2: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


# Standard aerospace ZYX Tait-Bryan decomposition -- bench-confirmed against
# the mesh: the formula's asin term reads as PITCH on this renderer's model
# and its atan2(x,...) term reads as ROLL, opposite of their usual aerospace
# names, so swapped here to match.
def _euler_deg_from_quat(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinp  = 2.0 * (w * y - z * x)
    angle_asin  = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    angle_atan2 = math.degrees(math.atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)))
    yaw         = math.degrees(math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z)))
    roll, pitch = angle_asin, angle_atan2
    return roll, pitch, yaw


# Inverse of _euler_deg_from_quat -- composition order verified (numerically,
# round-tripped through the extraction above) to match: yaw outermost, then
# roll (the asin/Y-linked term), then pitch (the atan2/X-linked term)
# innermost.
def _quat_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    r, p, y = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2, math.radians(yaw_deg) / 2
    qz = (math.cos(y), 0.0, 0.0, math.sin(y))
    qroll  = (math.cos(r), 0.0, math.sin(r), 0.0)
    qpitch = (math.cos(p), math.sin(p), 0.0, 0.0)
    return _qmul(_qmul(qz, qroll), qpitch)


# Roll comes in backwards from the sensor (rolling right reads negative and
# the mesh visibly rolls left) -- but there is NO single linear operation on
# a quaternion (no fixed reflection or rotation) that flips one Euler angle's
# sign while leaving the other two alone, for every orientation. Euler
# angles aren't independent coordinates the way X/Y/Z axes are, so the only
# construction that actually holds up under composition (verified: doesn't
# reintroduce cross-talk after calibration, unlike an earlier version of
# this fix that just negated a raw quaternion component) is to decompose,
# negate roll, and recompose.
def _flip_roll_sign(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = _euler_deg_from_quat(*q)
    return _quat_from_euler_deg(-roll, pitch, yaw)


# ── Palette ───────────────────────────────────────────────────────────────────
_BG     = QColor(15, 15, 16)
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
# el_deg=20 (the previous default) put a real cosine-shaped asymmetry into
# how visible PITCH is on screen: the projected nose position follows
# cos(pitch + el_deg), which has a near-zero-sensitivity "blind spot" at
# pitch = -el_deg, and reaches a fully edge-on/foreshortened look at
# pitch = +(90-el_deg) forward but only at pitch = -(90+el_deg) backward --
# with el_deg=20 that's +70 vs -110, which is why -90 still looked "angled"
# instead of flat. Smaller el_deg shrinks both the blind spot and the
# forward/backward gap while still giving the resting pose some 3D depth.
def _view_matrix(az_deg: float = 0.0, el_deg: float = 8.0) -> np.ndarray:
    az, el = math.radians(az_deg), math.radians(el_deg)
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)
    Ry = np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])
    Rx = np.array([[1, 0, 0], [0, ce, -se], [0, se, ce]])
    return Rx @ Ry

_VM = _view_matrix()

# Projected extent, at unit scale, of the bounding volume the rocket mesh
# is normalized into (see _BX/_ZL/_ZH) -- used purely to size the mesh to
# fit the widget below; nothing is actually drawn at these bounds anymore.
# Precomputed once here since it depends only on the fixed camera/bounding
# constants above, never on rocket orientation or widget size.
def _box_extent_unit() -> tuple[float, float]:
    B = _BX
    corners = np.array([
        [-B, -B, _ZL], [B, -B, _ZL], [B, B, _ZL], [-B, B, _ZL],
        [-B, -B, _ZH], [B, -B, _ZH], [B, B, _ZH], [-B, B, _ZH],
    ])
    vv = corners @ _VM.T
    width_unit  = float(vv[:, 0].max() - vv[:, 0].min())
    height_unit = float(vv[:, 2].max() - vv[:, 2].min())
    return width_unit, height_unit

_BOX_W_UNIT, _BOX_H_UNIT = _box_extent_unit()

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

        # Session-only YAW-FRAME reference, rebuilt by zero_yaw() -- NOT a
        # scalar number offset, and NOT a full-pose latch. This is a PURE
        # yaw rotation quaternion (zero pitch/roll content by construction),
        # so subtracting it via proper quaternion math can only ever
        # RE-ORIENT which raw axis reads as pitch vs roll to match the
        # current heading -- it can never cancel real tilt, because there's
        # no tilt in the reference to cancel with. That's the key difference
        # from a full-pose latch: a genuinely tilted rocket still reads a
        # genuinely nonzero pitch/roll after this, exactly as before, but
        # (unlike a naive scalar yaw-number offset, which only changes the
        # YAW NUMBER and does nothing about which physical direction gets
        # called "pitch" vs "roll") it actually fixes cross-axis coupling
        # caused by the IMU being mounted at some arbitrary HEADING relative
        # to the airframe's intended forward direction. Identity = no
        # calibration done yet this session.
        self._yaw_ref_quat = (1.0, 0.0, 0.0, 0.0)

        # The _correct_quat()-corrected reading from the most recent
        # update_data() call -- kept separately so zero_yaw() can extract
        # yaw from it (i.e. from the pre-yaw-frame-correction layer).
        self._last_corrected_quat = (1.0, 0.0, 0.0, 0.0)

    def zero_yaw(self) -> None:
        """Re-orient the pitch/roll axes to match the CURRENT heading --
        see the comment on _yaw_ref_quat above for why this is neither a
        simple number offset nor a full-pose latch, and what each of those
        alternatives gets wrong. The sensor's own gravity reference already
        tells it whether it's truly vertical, so there's nothing else to
        verify or calibrate here -- this is the only orientation calibration
        this widget needs."""
        w, x, y, z = self._last_corrected_quat
        yaw_rad = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
        self._yaw_ref_quat = (math.cos(yaw_rad/2), 0.0, 0.0, math.sin(yaw_rad/2))

    def _current_euler_deg(self) -> tuple[float, float, float]:
        return _euler_deg_from_quat(self._w, self._x, self._y, self._z)

    def update_data(self, data: TelemetryData) -> None:
        self._last_corrected_quat = _correct_quat(
            data.quat_w, data.quat_x, data.quat_y, data.quat_z
        )
        # Session yaw-frame correction (identity until zero_yaw() has been
        # run this session) -- see _yaw_ref_quat's comment for why this is a
        # quaternion-frame rotation, not a scalar number offset.
        yaw_corrected = _qmul(_qconj(self._yaw_ref_quat), self._last_corrected_quat)
        # Roll-sign fix -- see _flip_roll_sign() for why this has to be a
        # decompose/negate/recompose instead of a quaternion-component tweak.
        self._w, self._x, self._y, self._z = _flip_roll_sign(yaw_corrected)
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

        # Fit the mesh to whichever of width/height is actually the tighter
        # constraint, based on its real (non-square) aspect ratio -- rather
        # than a flat min(RENDER_W, H) * const, which caps the render at the
        # SMALLER raw dimension even when the mesh itself is short-and-wide
        # or tall-and-narrow, leaving the other dimension underused. A small
        # fixed margin so the mesh doesn't touch the widget edges.
        MARGIN_W, MARGIN_H = 16, 16
        sc = min(
            max(RENDER_W - MARGIN_W, 40) / _BOX_W_UNIT,
            max(H - MARGIN_H, 40) / _BOX_H_UNIT,
        )

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

        self._draw_readout(p, W - READ_W, 0, READ_W, H)

        p.end()

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
        # Already baked into self._w/x/y/z via _yaw_ref_quat (see
        # update_data()/zero_yaw()) -- no separate offset/wrap needed here.
        roll, pitch, yaw = self._current_euler_deg()

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
