"""Scrolling barometric altitude plot — white card, Apple-blue curve, 60 s window."""

import numpy as np
from collections import deque

import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore    import Qt

from core.packet_decoder import TelemetryData

_WINDOW_S   = 60
_MAX_POINTS = 600


class AltitudePlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self._times   = deque(maxlen=_MAX_POINTS)
        self._alts    = deque(maxlen=_MAX_POINTS)
        self._max_alt = 0.0
        self._t0: float | None = None

        # ── Plot widget ──────────────────────────────────────────
        self._plot = pg.PlotWidget(background='#FFFFFF')

        axis_pen   = pg.mkPen(color='#E5E5EA', width=1)
        label_args = {'color': '#6E6E73', 'font-size': '10px'}

        self._plot.setLabel('left',   'Altitude (m)', **label_args)
        self._plot.setLabel('bottom', 'Time (s)',      **label_args)
        self._plot.showGrid(x=True, y=True, alpha=0.6)
        self._plot.getAxis('left').setPen(axis_pen)
        self._plot.getAxis('bottom').setPen(axis_pen)
        self._plot.getAxis('left').setTextPen(pg.mkPen('#6E6E73'))
        self._plot.getAxis('bottom').setTextPen(pg.mkPen('#6E6E73'))
        self._plot.getPlotItem().getViewBox().setBorder(None)

        # Remove the outer border from the plot widget itself
        self._plot.setStyleSheet('border: none;')

        # ── Altitude curve — Apple blue ──────────────────────────
        self._curve = self._plot.plot(
            pen=pg.mkPen(color='#0071E3', width=2.5),
        )

        # ── Max altitude dashed line — iOS red ───────────────────
        self._max_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen(color='#FF3B30', width=1.5, style=Qt.PenStyle.DashLine),
            label='MAX  {value:.0f} m',
            labelOpts={'color': '#FF3B30', 'position': 0.92,
                       'fill': pg.mkBrush('#FFFFFF')},
        )
        self._plot.addItem(self._max_line)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._plot)

    def update_data(self, data: TelemetryData) -> None:
        if self._t0 is None:
            self._t0 = data.rx_time

        t = data.rx_time - self._t0
        self._times.append(t)
        self._alts.append(data.baro_alt_m)

        if data.baro_alt_m > self._max_alt:
            self._max_alt = data.baro_alt_m
            self._max_line.setPos(self._max_alt)

        t_arr = np.array(self._times)
        a_arr = np.array(self._alts)
        t_min = t - _WINDOW_S
        mask  = t_arr >= t_min
        self._curve.setData(t_arr[mask], a_arr[mask])
        self._plot.setXRange(max(0.0, t_min), t, padding=0.02)

    def reset(self) -> None:
        self._times.clear()
        self._alts.clear()
        self._max_alt = 0.0
        self._t0      = None
        self._curve.setData([], [])
        self._max_line.setPos(0)
