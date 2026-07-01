"""
GPS satellite map — real imagery tiles, no WebEngine or API key needed.

Tile source: ESRI World Imagery (free, unlimited for non-commercial use).
Tiles are fetched in background threads and cached in memory (LRU, 512 tiles).

Controls:
  Scroll wheel  → zoom in / out (levels 1–19)
  Click & drag  → pan (disables auto-follow)
  Double-click  → re-enable auto-follow on the rocket
"""

import math
import queue
import threading
from collections import OrderedDict
from urllib.request import urlopen, Request

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore    import Qt, QTimer, QPoint, QPointF
from PyQt6.QtGui     import (
    QPainter, QPixmap, QColor, QPen, QBrush, QPolygonF, QFont
)

from core.packet_decoder import TelemetryData

# ---------------------------------------------------------------------------
# ESRI World Imagery — real satellite tiles, no key required
# URL format: tile/{zoom}/{row}/{col}   (note: row=Y, col=X)
# ---------------------------------------------------------------------------
TILE_URL  = ('https://server.arcgisonline.com/ArcGIS/rest/services/'
             'World_Imagery/MapServer/tile/{z}/{y}/{x}')
TILE_SIZE  = 256
CACHE_MAX  = 512
MAX_TRAIL  = 3000
_HEADERS   = {'User-Agent': 'RocketPowerGCS/1.0'}
_MAX_WORKERS = 8   # concurrent download threads


# ---------------------------------------------------------------------------
# Coordinate math
# ---------------------------------------------------------------------------

def _to_global_px(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Lat/lon → global pixel position on the full tile grid."""
    n    = 2.0 ** zoom
    lr   = math.radians(lat)
    px_x = (lon + 180.0) / 360.0 * n * TILE_SIZE
    px_y = (1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n * TILE_SIZE
    return px_x, px_y


def _from_global_px(px_x: float, px_y: float, zoom: int) -> tuple[float, float]:
    """Global pixel → lat/lon (inverse of above)."""
    n   = 2.0 ** zoom
    lon = px_x / (n * TILE_SIZE) * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * px_y / (n * TILE_SIZE)))))
    return lat, lon


# ---------------------------------------------------------------------------
# Background tile downloader
# Raw PNG bytes are put on a queue; QPixmap is created on the main thread.
# ---------------------------------------------------------------------------

class _TileDownloader:
    def __init__(self, result_queue: queue.Queue) -> None:
        self._queue   = result_queue
        self._pending: set[tuple[int, int, int]] = set()
        self._lock    = threading.Lock()
        self._sem     = threading.Semaphore(_MAX_WORKERS)

    def request(self, z: int, x: int, y: int) -> None:
        key = (z, x, y)
        with self._lock:
            if key in self._pending:
                return
            self._pending.add(key)
        threading.Thread(target=self._fetch, args=(z, x, y), daemon=True).start()

    def _fetch(self, z: int, x: int, y: int) -> None:
        with self._sem:
            url = TILE_URL.format(z=z, y=y, x=x)
            try:
                req  = Request(url, headers=_HEADERS)
                data = urlopen(req, timeout=10).read()
                self._queue.put((z, x, y, data))
            except Exception:
                pass            # network error: tile stays blank this session
            finally:
                with self._lock:
                    self._pending.discard((z, x, y))


# ---------------------------------------------------------------------------
# Widget
# ---------------------------------------------------------------------------

class GPSMap(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)

        # Map view state — start at a world overview so tiles show immediately
        self._zoom        = 3
        self._center_lat  = 30.0    # rough centre of the world (Europe/Africa area)
        self._center_lon  = 10.0
        self._launch_lat: float | None = None
        self._launch_lon: float | None = None
        self._trail: list[tuple[float, float]] = []
        self._auto_follow = True
        self._has_fix     = False
        self._last_sats   = 0
        self._last_gps_alt: float | None = None

        # LRU tile cache  {(z, x, y): QPixmap}
        self._cache: OrderedDict[tuple[int, int, int], QPixmap] = OrderedDict()

        # Tile download pipeline
        self._tile_queue: queue.Queue = queue.Queue()
        self._downloader = _TileDownloader(self._tile_queue)

        # Drain finished tiles into the cache on the main thread
        self._poll = QTimer(self)
        self._poll.setInterval(50)          # 20 Hz
        self._poll.timeout.connect(self._drain_queue)
        self._poll.start()

        # Mouse drag state
        self._drag_start: QPoint | None          = None
        self._drag_origin_px: tuple[float, float] = (0.0, 0.0)

    # -- Data API ----------------------------------------------------------

    def update_data(self, data: TelemetryData) -> None:
        self._last_sats = data.gps_sats

        if not data.has_gps_fix:
            self.update()
            return

        lat, lon = data.lat, data.lon

        if self._launch_lat is None:        # first fix ever → snap to rocket
            self._launch_lat = lat
            self._launch_lon = lon
            self._center_lat = lat
            self._center_lon = lon
            self._zoom        = 17          # zoom in from world overview

        if not self._has_fix:
            self._has_fix = True

        self._last_gps_alt = data.gps_alt_m
        self._trail.append((lat, lon))
        if len(self._trail) > MAX_TRAIL:
            self._trail = self._trail[-MAX_TRAIL:]

        if self._auto_follow:
            self._center_lat = lat
            self._center_lon = lon
        self.update()

    def reset(self) -> None:
        self._zoom        = 3
        self._center_lat  = 30.0
        self._center_lon  = 10.0
        self._launch_lat  = None
        self._launch_lon  = None
        self._trail.clear()
        self._auto_follow = True
        self._has_fix     = False
        self._last_sats   = 0
        self.update()

    # -- Tile cache --------------------------------------------------------

    def _drain_queue(self) -> None:
        """Called on the main thread: turn downloaded bytes into QPixmaps."""
        changed = False
        while True:
            try:
                z, x, y, data = self._tile_queue.get_nowait()
            except queue.Empty:
                break
            px = QPixmap()
            if px.loadFromData(data):
                key = (z, x, y)
                self._cache[key] = px
                self._cache.move_to_end(key)
                if len(self._cache) > CACHE_MAX:
                    self._cache.popitem(last=False)
                changed = True
        if changed:
            self.update()

    def _get_tile(self, z: int, x: int, y: int) -> QPixmap | None:
        key = (z, x, y)
        px  = self._cache.get(key)
        if px:
            self._cache.move_to_end(key)
            return px
        self._downloader.request(z, x, y)
        return None

    # -- Coordinate helpers ------------------------------------------------

    def _to_widget(self, lat: float, lon: float,
                   cx: float, cy: float) -> tuple[float, float]:
        gx, gy = _to_global_px(lat, lon, self._zoom)
        return gx - cx + self.width() / 2, gy - cy + self.height() / 2

    # -- Paint -------------------------------------------------------------

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        p.fillRect(0, 0, w, h, QColor(15, 15, 16))

        cx, cy = _to_global_px(self._center_lat, self._center_lon, self._zoom)

        # Visible tile range
        max_t = (1 << self._zoom) - 1
        tx_lo = max(0,     int((cx - w / 2) / TILE_SIZE) - 1)
        tx_hi = min(max_t, int((cx + w / 2) / TILE_SIZE) + 1)
        ty_lo = max(0,     int((cy - h / 2) / TILE_SIZE) - 1)
        ty_hi = min(max_t, int((cy + h / 2) / TILE_SIZE) + 1)

        # Draw satellite tiles
        for tx in range(tx_lo, tx_hi + 1):
            for ty in range(ty_lo, ty_hi + 1):
                tile = self._get_tile(self._zoom, tx, ty)
                if tile:
                    dx = int(tx * TILE_SIZE - cx + w / 2)
                    dy = int(ty * TILE_SIZE - cy + h / 2)
                    p.drawPixmap(dx, dy, tile)

        # GPS trail -- blue like reference
        if len(self._trail) >= 2:
            pen = QPen(QColor('#60A5FA'), 2.5)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            p.setPen(pen)
            pts = [QPointF(*self._to_widget(lat, lon, cx, cy))
                   for lat, lon in self._trail]
            for i in range(1, len(pts)):
                p.drawLine(pts[i - 1], pts[i])

        # Launch site — red triangle
        if self._launch_lat is not None:
            lx, ly = self._to_widget(self._launch_lat, self._launch_lon, cx, cy)
            p.setPen(QPen(QColor('#EF4444'), 2))
            p.setBrush(QBrush(QColor('#EF4444')))
            p.drawPolygon(QPolygonF([
                QPointF(lx,      ly - 11),
                QPointF(lx - 8,  ly + 6),
                QPointF(lx + 8,  ly + 6),
            ]))

        # Current rocket position -- blue/white dot
        if self._trail:
            lat, lon = self._trail[-1]
            rx, ry = self._to_widget(lat, lon, cx, cy)
            p.setPen(QPen(QColor('#FFFFFF'), 2))
            p.setBrush(QBrush(QColor('#60A5FA')))
            p.drawEllipse(QPointF(rx, ry), 7.0, 7.0)

        # Lat/lon/alt overlay (top-left)
        if self._trail:
            last_lat, last_lon = self._trail[-1]
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor(20, 20, 22, 200)))
            p.drawRoundedRect(10, 10, 160, 56, 6, 6)
            p.setFont(QFont('JetBrains Mono', 10))
            p.setPen(QColor('#94A3B8'))
            p.drawText(18, 26, 'LAT')
            p.drawText(18, 41, 'LON')
            p.drawText(18, 56, 'ALT')
            alt_str = f'{self._last_gps_alt:.0f} m' if self._last_gps_alt is not None else '--'
            p.setPen(QColor('#F1F5F9'))
            p.drawText(46, 26, f'{last_lat:.6f} deg')
            p.drawText(46, 41, f'{last_lon:.6f} deg')
            p.drawText(46, 56, alt_str)

        # Info bar at bottom
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(12, 12, 14, 210)))
        p.drawRect(0, h - 24, w, 24)
        p.setFont(QFont('JetBrains Mono', 9))

        if self._has_fix:
            gps_color = '#22C55E'
            gps_txt   = f'GPS FIX  {self._last_sats} SATS'
        else:
            gps_color = '#F59E0B'
            sats = self._last_sats
            gps_txt = f'NO FIX  {sats} SAT{"S" if sats != 1 else ""}  —  NEEDS OPEN SKY'

        p.setPen(QColor(gps_color))
        p.drawText(8, h - 8, gps_txt)

        follow_txt = 'FOLLOWING' if self._auto_follow else 'MANUAL  DBL-CLICK TO FOLLOW'
        p.setPen(QColor('#64748B'))
        p.drawText(w - 260, h - 8, f'Z={self._zoom}  SCROLL=ZOOM  DRAG=PAN  {follow_txt}')

        p.end()

    # -- Mouse interaction -------------------------------------------------

    def wheelEvent(self, event) -> None:
        if event.angleDelta().y() > 0 and self._zoom < 19:
            self._zoom += 1
        elif event.angleDelta().y() < 0 and self._zoom > 1:
            self._zoom -= 1
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self._center_lat is not None:
            self._drag_start     = event.position().toPoint()
            self._drag_origin_px = _to_global_px(
                self._center_lat, self._center_lon, self._zoom
            )
            self._auto_follow = False

    def mouseMoveEvent(self, event) -> None:
        if self._drag_start is None:
            return
        delta = event.position().toPoint() - self._drag_start
        ox, oy = self._drag_origin_px
        self._center_lat, self._center_lon = _from_global_px(
            ox - delta.x(), oy - delta.y(), self._zoom
        )
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = None

    def mouseDoubleClickEvent(self, _event) -> None:
        self._auto_follow = True
        if self._trail:
            self._center_lat, self._center_lon = self._trail[-1]
        self.update()
