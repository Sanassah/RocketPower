"""Scrolling acceleration magnitude plot — white card, iOS-orange curve, 60 s window."""

import numpy as np
from collections import deque

import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore    import Qt

from core.packet_decoder import TelemetryData

_WINDOW_S   = 60
_MAX_POINTS = 600


class AccelPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self._times = deque(maxlen=_MAX_POINTS)
        self._mags  = deque(maxlen=_MAX_POINTS)
        self._t0: float | None = None

        # ── Plot widget ──────────────────────────────────────────
        self._plot = pg.PlotWidget(background='#FFFFFF')

        axis_pen   = pg.mkPen(color='#E5E5EA', width=1)
        label_args = {'color': '#6E6E73', 'font-size': '10px'}

        self._plot.setLabel('left',   'Linear Acceleration (g)', **label_args)
        self._plot.setLabel('bottom', 'Time (s)',          **label_args)
        self._plot.showGrid(x=True, y=True, alpha=0.6)
        self._plot.getAxis('left').setPen(axis_pen)
        self._plot.getAxis('bottom').setPen(axis_pen)
        self._plot.getAxis('left').setTextPen(pg.mkPen('#6E6E73'))
        self._plot.getAxis('bottom').setTextPen(pg.mkPen('#6E6E73'))
        self._plot.getPlotItem().getViewBox().setBorder(None)
        self._plot.setStyleSheet('border: none;')

        # ── Accel curve — iOS orange ─────────────────────────────
        self._curve = self._plot.plot(
            pen=pg.mkPen(color='#FF9F0A', width=2.5),
        )

        # ── 0 g reference (at rest) — iOS green ─────────────────
        self._zero_g = pg.InfiniteLine(
            pos=0.0, angle=0,
            pen=pg.mkPen(color='#34C759', width=1.5, style=Qt.PenStyle.DashLine),
            label='0 g  (rest)',
            labelOpts={'color': '#34C759', 'position': 0.06,
                       'fill': pg.mkBrush('#FFFFFF')},
        )
        self._plot.addItem(self._zero_g)

        # Fixed Y floor so noise at rest (±0.05 g) doesn't fill the whole axis
        self._plot.setYRange(-0.5, 5.0, padding=0.02)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._plot)

    def update_data(self, data: TelemetryData) -> None:
        if self._t0 is None:
            self._t0 = data.rx_time

        t = data.rx_time - self._t0
        self._times.append(t)
        self._mags.append(data.accel_mag_g)

        t_arr = np.array(self._times)
        m_arr = np.array(self._mags)
        t_min = t - _WINDOW_S
        mask  = t_arr >= t_min
        self._curve.setData(t_arr[mask], m_arr[mask])
        self._plot.setXRange(max(0.0, t_min), t, padding=0.02)

    def reset(self) -> None:
        self._times.clear()
        self._mags.clear()
        self._t0 = None
        self._curve.setData([], [])
