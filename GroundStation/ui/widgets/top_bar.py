"""Dark top bar: title row + telemetry stats strip."""

import datetime

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel
from PyQt6.QtCore    import Qt, QTimer, QRectF
from PyQt6.QtGui     import QFont, QColor, QPainter, QPen, QBrush

from core.packet_decoder import TelemetryData
from core.serial_worker  import LinkStats

# The radio is stuck at a fixed factory-default air data rate -- see
# TELEMETRY_INTERVAL_MS in FlightComputer/src/config.h for the full story.
# This is what RX+TX bandwidth is actually being measured against.
_LORA_AIR_RATE_BPS = 2400

_BG     = '#0F0F10'
_CARD   = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_GREEN  = '#22C55E'
_ORANGE = '#F59E0B'
_RED    = '#EF4444'
_BLUE   = '#3B82F6'

_VMIN, _VMAX = 6.0, 8.4

_STATE_COLOR = {
    'IDLE':           '#94A3B8',
    'ARMED':          _BLUE,
    'POWERED_ASCENT': _ORANGE,
    'COAST':          _GREEN,
    'APOGEE':         '#A78BFA',
    'DESCENT':        '#67E8F9',
    'LANDED':         _GREEN,
}


def _card_css() -> str:
    return f'background-color:{_CARD};border:none;border-radius:9px;'


def _lbl_css(color: str, size: int = 12, weight: int = 600) -> str:
    return (
        f'color:{color};font-size:{size}px;font-weight:{weight};'
        f'letter-spacing:0.9px;border:none;background:transparent;'
    )


# ── Standard stat card ────────────────────────────────────────────────────────
class _Stat(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(_card_css())
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(2)
        lbl = QLabel(title)
        lbl.setStyleSheet(_lbl_css(_MUTED))
        self._val = QLabel('--')
        self._val.setFont(QFont('Segoe UI', 16, QFont.Weight.Bold))
        self._val.setStyleSheet(_lbl_css(_TEXT, 16, 700))
        lay.addWidget(lbl)
        lay.addWidget(self._val)

    def set(self, text: str, color: str = _TEXT) -> None:
        self._val.setText(text)
        self._val.setStyleSheet(_lbl_css(color, 16, 700))


# ── State badge card (replaces Vehicle ID) ────────────────────────────────────
class _StatStateBadge(QWidget):
    """Shows the current flight state as a large colored rounded badge."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(_card_css())

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._badge = QLabel('IDLE')
        self._badge.setFont(QFont('Segoe UI', 15, QFont.Weight.ExtraBold))
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setStyleSheet(
            f'color:#0A0A0B;background:{_STATE_COLOR["IDLE"]};'
            f'border-radius:10px;padding:7px 16px;'
            f'font-size:15px;font-weight:800;letter-spacing:2px;border:none;'
        )

        lay.addStretch()
        lay.addWidget(self._badge)
        lay.addStretch()

    def set_state(self, state_name: str) -> None:
        color = _STATE_COLOR.get(state_name, _MUTED)
        self._badge.setText(state_name.replace('_', ' '))
        self._badge.setStyleSheet(
            f'color:#0A0A0B;background:{color};'
            f'border-radius:10px;padding:7px 16px;'
            f'font-size:15px;font-weight:800;letter-spacing:2px;border:none;'
        )


# ── Battery icon (QPainter) ───────────────────────────────────────────────────
class _BatteryIcon(QWidget):
    """Classic battery cell with fill level, drawn with QPainter."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pct   = 0.0
        self._color = _MUTED
        self.setFixedSize(64, 30)

    def set(self, pct: float, color: str) -> None:
        self._pct   = pct
        self._color = color
        self.update()

    def paintEvent(self, _) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        W, H = self.width(), self.height()
        NUB_W = 5
        NUB_H = int(H * 0.42)

        bx, by   = 1, 2
        body_w   = W - NUB_W - 3
        body_h   = H - 4

        col      = QColor(self._color)
        no_data  = self._pct <= 0
        outline  = QColor(70, 70, 75) if no_data else col

        # Body outline
        p.setPen(QPen(outline, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(QRectF(bx, by, body_w, body_h), 3.5, 3.5)

        # Terminal nub
        nub_x = bx + body_w + 1
        nub_y = by + (body_h - NUB_H) / 2
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(outline))
        p.drawRoundedRect(QRectF(nub_x, nub_y, NUB_W - 1, NUB_H), 1.5, 1.5)

        # Charge fill
        if not no_data:
            pad      = 3
            max_fill = body_w - 2 * pad
            fill_w   = max(2.0, max_fill * self._pct / 100.0)
            p.setBrush(QBrush(col))
            p.drawRoundedRect(
                QRectF(bx + pad, by + pad, fill_w, body_h - 2 * pad),
                2, 2
            )

        p.end()


# ── Battery stat card with icon ───────────────────────────────────────────────
class _StatBattery(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(_card_css())

        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(2)

        ttl = QLabel('BATTERY')
        ttl.setStyleSheet(_lbl_css(_MUTED))
        lay.addWidget(ttl)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(10)
        row.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self._val = QLabel('-- %')
        self._val.setFont(QFont('Segoe UI', 16, QFont.Weight.Bold))
        self._val.setStyleSheet(_lbl_css(_MUTED, 16, 700))

        self._icon = _BatteryIcon()

        row.addWidget(self._val)
        row.addStretch()
        row.addWidget(self._icon)
        lay.addLayout(row)

    def set_pct(self, pct: float, color: str) -> None:
        self._val.setText(f'{pct:.0f} %')
        self._val.setStyleSheet(_lbl_css(color, 16, 700))
        self._icon.set(pct, color)


# ── Telemetry card with signal dot ────────────────────────────────────────────
class _StatTelem(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(_card_css())
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(2)

        ttl = QLabel('TELEMETRY')
        ttl.setStyleSheet(_lbl_css(_MUTED))
        lay.addWidget(ttl)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self._val = QLabel('NO LINK')
        self._val.setFont(QFont('Segoe UI', 16, QFont.Weight.Bold))
        self._val.setStyleSheet(_lbl_css(_RED, 16, 700))

        self._dot = QLabel('●')
        self._dot.setFont(QFont('Segoe UI', 13))
        self._dot.setStyleSheet(f'color:{_RED};background:transparent;border:none;')
        self._dot.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        row.addWidget(self._val)
        row.addWidget(self._dot)
        row.addStretch()
        lay.addLayout(row)

        self._bw_lbl = QLabel('RX -- / TX --')
        self._bw_lbl.setStyleSheet(_lbl_css(_MUTED, 11, 600))
        self._bw_lbl.setToolTip('Live throughput vs. the LoRa link\'s fixed air data rate.')
        lay.addWidget(self._bw_lbl)

    def set(self, text: str, color: str) -> None:
        self._val.setText(text)
        self._val.setStyleSheet(_lbl_css(color, 16, 700))
        self._dot.setStyleSheet(f'color:{color};background:transparent;border:none;')

    def set_bandwidth(self, stats: LinkStats) -> None:
        used_bps = stats.rx_bps + stats.tx_bps
        pct = used_bps / _LORA_AIR_RATE_BPS * 100.0

        # Real signal-integrity proxy -- see LinkStats.rx_bad_packets. The
        # E22 radio can't report RSSI in transparent mode (LoRa.h always
        # returns -1), so a rising corrupted-packet rate is what actually
        # tells you the link is degrading here.
        good_and_bad = stats.rx_pkt_per_sec + stats.rx_bad_pkt_per_sec
        err_pct = (stats.rx_bad_pkt_per_sec / good_and_bad * 100.0) if good_and_bad > 0 else 0.0

        color = _RED if (pct >= 90 or err_pct >= 10) else _ORANGE if (pct >= 60 or err_pct >= 2) else _MUTED
        self._bw_lbl.setStyleSheet(_lbl_css(color, 11, 600))
        self._bw_lbl.setText(
            f'RX {stats.rx_pkt_per_sec:.1f}/s {stats.rx_bps:.0f}bps  '
            f'TX {stats.tx_pkt_per_sec:.1f}/s {stats.tx_bps:.0f}bps  '
            f'({pct:.0f}% of {_LORA_AIR_RATE_BPS}bps)  ·  {err_pct:.0f}% err'
        )
        self._bw_lbl.setToolTip(
            f'RX total: {stats.rx_packets} packets\n'
            f'TX total: {stats.tx_packets} packets\n'
            f'Combined throughput: {used_bps:.0f} bps of the LoRa link\'s '
            f'{_LORA_AIR_RATE_BPS} bps fixed air rate ({pct:.0f}%).\n\n'
            f'Corrupted packets: {stats.rx_bad_packets} total, '
            f'{err_pct:.1f}% of traffic this window -- the real link-quality '
            f'signal here, since the E22 radio can\'t report RSSI in '
            f'transparent mode (see LoRa.h).'
        )

    def clear_bandwidth(self) -> None:
        self._bw_lbl.setStyleSheet(_lbl_css(_MUTED, 11, 600))
        self._bw_lbl.setText('RX -- / TX --')


# ── Top bar ───────────────────────────────────────────────────────────────────
class TopBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f'background-color:{_BG};border-bottom:1px solid {_BORDER};')
        self.setFixedHeight(130)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 8, 16, 8)
        root.setSpacing(6)

        # Title row
        row1 = QHBoxLayout()
        row1.setSpacing(8)

        logo = QLabel('/ /  ROCKETPOWER')
        logo.setFont(QFont('Segoe UI', 14, QFont.Weight.Bold))
        logo.setStyleSheet(
            f'color:{_TEXT};letter-spacing:3px;background:transparent;border:none;'
        )
        dash = QLabel('DASHBOARD')
        dash.setFont(QFont('Segoe UI', 14))
        dash.setStyleSheet(
            f'color:{_MUTED};letter-spacing:3px;background:transparent;border:none;'
        )
        self._clock = QLabel()
        self._clock.setFont(QFont('JetBrains Mono', 12))
        self._clock.setStyleSheet(f'color:{_MUTED};background:transparent;border:none;')

        row1.addWidget(logo)
        row1.addWidget(dash)
        row1.addStretch()
        row1.addWidget(self._clock)
        root.addLayout(row1)

        # Stats row
        row2 = QHBoxLayout()
        row2.setSpacing(10)

        self._s_state_badge = _StatStateBadge()
        self._s_met         = _Stat('MISSION TIME')
        self._s_state       = _Stat('FLIGHT STATE')
        self._s_alt         = _Stat('ALTITUDE  (MSL)')
        self._s_vel         = _Stat('VELOCITY')
        self._s_batt        = _StatBattery()
        self._s_telem       = _StatTelem()

        self._s_met.set('00:00:00', _MUTED)
        self._s_state.set('PRE-LAUNCH', _MUTED)
        self._s_alt.set('-- m', _MUTED)
        self._s_vel.set('-- m/s', _MUTED)

        for w in (self._s_state_badge, self._s_met, self._s_state,
                  self._s_alt, self._s_vel, self._s_batt, self._s_telem):
            row2.addWidget(w)

        root.addLayout(row2)

        self._utc_timer = QTimer(self)
        self._utc_timer.setInterval(1000)
        self._utc_timer.timeout.connect(self._tick)
        self._utc_timer.start()
        self._tick()

    def _tick(self) -> None:
        now = datetime.datetime.utcnow()
        self._clock.setText(now.strftime('UTC   %Y-%m-%d   %H:%M:%S'))

    def update_data(self, data: TelemetryData) -> None:
        sn = data.state_name
        self._s_state_badge.set_state(sn)
        self._s_state.set(sn.replace('_', ' '), _STATE_COLOR.get(sn, _MUTED))

        ms = data.timestamp_ms
        h  = ms // 3_600_000
        m  = (ms % 3_600_000) // 60_000
        s  = (ms % 60_000) // 1000
        self._s_met.set(f'{h:02d}:{m:02d}:{s:02d}', _TEXT)

        self._s_alt.set(f'{data.baro_alt_m:.0f} m', _TEXT)
        self._s_vel.set(f'{data.vert_vel_ms:+.1f} m/s', _TEXT)

        pct = max(0.0, min(100.0, (data.voltage_v - _VMIN) / (_VMAX - _VMIN) * 100.0))
        bc  = _GREEN if pct > 40 else _ORANGE if pct > 20 else _RED
        self._s_batt.set_pct(pct, bc)

    def update_stats(self, stats: LinkStats) -> None:
        # Nominal downlink rate is 1 Hz -- see TELEMETRY_INTERVAL_MS in
        # FlightComputer/src/config.h (hard-won: faster rates starved the
        # half-duplex radio of listening time for inbound commands).
        rx_hz = stats.rx_pkt_per_sec
        if rx_hz >= 0.8:
            color = _GREEN
        elif rx_hz >= 0.3:
            color = _ORANGE
        else:
            color = _RED
        self._s_telem.set(f'{rx_hz:.1f} Hz', color)
        self._s_telem.set_bandwidth(stats)

    def set_connected(self, connected: bool) -> None:
        if not connected:
            self._s_state_badge.set_state('IDLE')
            self._s_state.set('PRE-LAUNCH', _MUTED)
            self._s_alt.set('-- m', _MUTED)
            self._s_vel.set('-- m/s', _MUTED)
            self._s_batt.set_pct(0, _MUTED)
            self._s_telem.set('NO LINK', _RED)
            self._s_telem.clear_bandwidth()
            self._s_met.set('00:00:00', _MUTED)
