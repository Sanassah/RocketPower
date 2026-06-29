"""
2D rocket silhouette — light background, dark charcoal body, Apple-blue porthole.
Rotates based on BNO085 quaternion pitch. Flame renders during POWERED_ASCENT.
"""

import math

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore    import Qt, QPointF
from PyQt6.QtGui     import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, QLinearGradient
)

from core.packet_decoder import TelemetryData


def _quat_to_pitch_deg(w: float, x: float, y: float, z: float) -> float:
    z_world = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    x_world = 2.0 * (x * z + w * y)
    return math.degrees(math.atan2(x_world, max(z_world, 1e-9)))


class RocketVisual(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch_deg = 0.0
        self._state     = 0
        self.setMinimumHeight(150)

    def update_data(self, data: TelemetryData) -> None:
        self._pitch_deg = _quat_to_pitch_deg(
            data.quat_w, data.quat_x, data.quat_y, data.quat_z
        )
        self._state = data.state
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor('#F5F5F7'))

        # Pitch angle readout
        painter.setPen(QPen(QColor('#8E8E93')))
        painter.setFont(QFont('JetBrains Mono', 9))
        painter.drawText(8, 16, f'{self._pitch_deg:+.1f}°')

        # Centre and rotate
        painter.translate(w / 2, h / 2)
        painter.rotate(self._pitch_deg)

        scale = min(w, h) / 220.0
        painter.scale(scale, scale)

        painter.setPen(Qt.PenStyle.NoPen)

        # ── Fins ──────────────────────────────────────────────────
        painter.setBrush(QBrush(QColor('#3A3A3C')))
        painter.drawPolygon(QPolygonF([QPointF(-14, 60), QPointF(-30, 102), QPointF(-14, 88)]))
        painter.drawPolygon(QPolygonF([QPointF( 14, 60), QPointF( 30, 102), QPointF( 14, 88)]))

        # ── Body tube ─────────────────────────────────────────────
        painter.setBrush(QBrush(QColor('#2C2C2E')))
        painter.drawRoundedRect(-14, -60, 28, 130, 5, 5)

        # ── Nose cone ─────────────────────────────────────────────
        painter.setBrush(QBrush(QColor('#1D1D1F')))
        painter.drawPolygon(QPolygonF([QPointF(-14, -60), QPointF(14, -60), QPointF(0, -102)]))

        # ── Nozzle bell ───────────────────────────────────────────
        painter.setBrush(QBrush(QColor('#636366')))
        painter.drawPolygon(QPolygonF([
            QPointF(-10, 70), QPointF(10, 70),
            QPointF(8,   83), QPointF(-8, 83),
        ]))

        # ── Porthole — Apple blue ─────────────────────────────────
        painter.setBrush(QBrush(QColor('#CCE4FF')))
        painter.setPen(QPen(QColor('#0071E3'), 1.5))
        painter.drawEllipse(-7, -32, 14, 14)
        painter.setPen(Qt.PenStyle.NoPen)

        # ── Racing stripe ─────────────────────────────────────────
        painter.setBrush(QBrush(QColor('#0071E3')))
        painter.drawRect(-14, 10, 28, 6)

        # ── Flame (POWERED_ASCENT only) ───────────────────────────
        if self._state == 2:
            grad = QLinearGradient(0, 83, 0, 135)
            grad.setColorAt(0.0, QColor(255, 159, 10, 240))    # iOS orange
            grad.setColorAt(0.6, QColor(255, 59,  48, 200))    # iOS red
            grad.setColorAt(1.0, QColor(255, 59,  48,   0))    # fade out
            painter.setBrush(QBrush(grad))
            painter.drawPolygon(QPolygonF([
                QPointF(-8, 83), QPointF(8, 83),
                QPointF(5, 108), QPointF(0, 132), QPointF(-5, 108),
            ]))

        painter.end()
