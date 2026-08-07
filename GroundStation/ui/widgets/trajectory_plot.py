"""
2D trajectory side-view: altitude (Y) vs horizontal downrange distance (X).
When GPS fix is available, X is the haversine distance from the first-fix point.
Without GPS, X stays 0, so the chart degrades to a vertical altitude-only column.
"""

import math
from collections import deque

import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_BG     = '#080808'
_CARD   = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_GREEN  = '#22C55E'
_ORANGE = '#F59E0B'
_RED    = '#EF4444'
_PURPLE = '#A78BFA'

_MAX_POINTS = 2000


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 6_371_000.0 * 2.0 * math.asin(math.sqrt(max(0.0, a)))


def _stat_widget(title: str) -> tuple[QWidget, QLabel]:
    """Returns (container, value_label)."""
    w = QWidget()
    w.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
    w.setStyleSheet('background:transparent;')
    lay = QVBoxLayout(w)
    lay.setContentsMargins(0, 4, 0, 4)
    lay.setSpacing(1)

    t = QLabel(title)
    t.setStyleSheet(
        f'color:{_MUTED};font-size:13px;font-weight:700;letter-spacing:0.6px;'
        f'border:none;background:transparent;'
    )
    v = QLabel('--')
    v.setFont(QFont('JetBrains Mono', 15, QFont.Weight.Bold))
    v.setStyleSheet(
        f'color:{_TEXT};font-size:15px;font-weight:700;border:none;background:transparent;'
    )
    lay.addWidget(t)
    lay.addWidget(v)
    return w, v


class TrajectoryPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_CARD};border:1px solid {_BORDER};border-radius:8px;'
        )

        self._x: deque[float] = deque(maxlen=_MAX_POINTS)
        self._y: deque[float] = deque(maxlen=_MAX_POINTS)
        self._launch_lat: float | None = None
        self._launch_lon: float | None = None
        self._max_alt  = 0.0
        self._apogee_x = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 10)
        root.setSpacing(4)

        # Header
        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('~~')
        sym.setStyleSheet(
            f'color:{_TEXT};font-size:14px;background:transparent;border:none;'
        )
        ttl = QLabel('FLIGHT PATH  (2D)')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)

        # Plot
        self._plot = pg.PlotWidget(background=_BG)
        p = self._plot
        lbl = {'color': _MUTED, 'font-size': '12px'}
        p.setLabel('left',   'Altitude (m)',  **lbl)
        p.setLabel('bottom', 'Downrange (m)', **lbl)
        p.showGrid(x=True, y=True, alpha=0.12)
        for ax in ('left', 'bottom'):
            p.getAxis(ax).setPen(pg.mkPen(_BORDER))
            p.getAxis(ax).setTextPen(pg.mkPen(_MUTED))
        p.setStyleSheet('border:none;background:transparent;')
        p.setLimits(yMin=0)          # rocket can't go underground
        p.setYRange(0, 500, padding=0)

        self._launch_dot = pg.ScatterPlotItem(
            [0], [0], symbol='t', size=14,
            pen=pg.mkPen(_RED, width=2), brush=pg.mkBrush(_RED),
        )
        p.addItem(self._launch_dot)

        self._curve = p.plot(pen=pg.mkPen(_GREEN, width=2))

        self._head = pg.ScatterPlotItem(
            [], [], symbol='o', size=10,
            pen=pg.mkPen('#FFFFFF', width=1.5), brush=pg.mkBrush(_GREEN),
        )
        p.addItem(self._head)

        self._apogee_dot = pg.ScatterPlotItem(
            [], [], symbol='o', size=10,
            pen=pg.mkPen(_PURPLE, width=2), brush=pg.mkBrush(_PURPLE),
        )
        p.addItem(self._apogee_dot)

        self._max_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen(_ORANGE, width=1, style=Qt.PenStyle.DashLine),
            label='MAX {value:.0f} m',
            labelOpts={'color': _ORANGE, 'position': 0.92, 'fill': pg.mkBrush(_BG)},
        )
        p.addItem(self._max_line)

        root.addWidget(self._plot)

        # Stats bar
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1)
        sep.setStyleSheet(f'background-color:{_BORDER};')
        root.addWidget(sep)

        stats_row = QHBoxLayout()
        stats_row.setSpacing(0)
        self._w_maxalt, self._v_maxalt = _stat_widget('MAX ALTITUDE')
        self._w_apogee, self._v_apogee = _stat_widget('APOGEE')
        self._w_drange, self._v_drange = _stat_widget('DOWNRANGE')
        self._w_impact, self._v_impact = _stat_widget('IMPACT (EST.)')

        for i, w in enumerate(
            [self._w_maxalt, self._w_apogee, self._w_drange, self._w_impact]
        ):
            stats_row.addWidget(w)
            if i < 3:
                div = QFrame()
                div.setFrameShape(QFrame.Shape.VLine)
                div.setFixedWidth(1)
                div.setStyleSheet(f'background-color:{_BORDER};')
                stats_row.addWidget(div)
                stats_row.addSpacing(12)

        root.addLayout(stats_row)

    def update_data(self, data: TelemetryData) -> None:
        alt = data.baro_alt_m

        if data.has_gps_fix:
            if self._launch_lat is None:
                self._launch_lat = data.lat
                self._launch_lon = data.lon
            horiz = _haversine_m(self._launch_lat, self._launch_lon, data.lat, data.lon)
        else:
            horiz = 0.0

        self._x.append(horiz)
        self._y.append(alt)

        if alt > self._max_alt:
            self._max_alt  = alt
            self._apogee_x = horiz
            self._max_line.setPos(alt)
            self._v_maxalt.setText(f'{alt:.0f} m')
            self._v_apogee.setText(f'{alt:.0f} m')
            self._apogee_dot.setData([horiz], [alt])

        self._v_drange.setText(f'{horiz:.0f} m')

        # Auto-scale Y from 0 to 15% above peak altitude
        self._plot.setYRange(0, max(self._max_alt * 1.15, 20), padding=0)

        xs = list(self._x)
        ys = list(self._y)
        self._curve.setData(xs, ys)
        if xs:
            self._head.setData([xs[-1]], [ys[-1]])

    def reset(self) -> None:
        self._x.clear()
        self._y.clear()
        self._launch_lat = None
        self._launch_lon = None
        self._max_alt    = 0.0
        self._apogee_x   = 0.0
        self._curve.setData([], [])
        self._head.setData([], [])
        self._apogee_dot.setData([], [])
        self._max_line.setPos(0)
        for v in (self._v_maxalt, self._v_apogee, self._v_drange, self._v_impact):
            v.setText('--')
