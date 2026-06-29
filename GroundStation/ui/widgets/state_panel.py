"""
Flight state display — filled pill badge with iOS-style state colours.
ARMED flashes at 1 Hz (solid blue ↔ light blue).
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore    import Qt, QTimer
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

# iOS / Apple HIG-aligned palette, one colour per state
_STATE_COLORS = {
    'IDLE':           '#8E8E93',   # system grey
    'ARMED':          '#0071E3',   # Apple blue
    'POWERED_ASCENT': '#FF9F0A',   # iOS orange
    'COAST':          '#32ADE6',   # teal
    'APOGEE':         '#BF5AF2',   # purple
    'DESCENT':        '#5AC8FA',   # light blue / cyan
    'LANDED':         '#34C759',   # iOS green
}
_ARMED_DIM = '#4DA6FF'   # lighter blue for the "off" flash frame


def _badge_css(color: str, alpha_bg: float = 0.12) -> str:
    """Solid-fill pill: coloured background + white text."""
    return (
        f'background-color: {color};'
        f'color: #FFFFFF;'
        f'border-radius: 22px;'
        f'font-family: "Segoe UI", Arial, sans-serif;'
        f'font-size: 20px;'
        f'font-weight: 800;'
        f'letter-spacing: 0.5px;'
        f'padding: 14px 20px;'
    )


class StatePanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('FLIGHT STATE', parent)
        self._flash_on      = True
        self._current_state = 'IDLE'

        # ── Badge ────────────────────────────────────────────────
        self._badge = QLabel('IDLE')
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setFont(QFont('Segoe UI', 20, QFont.Weight.ExtraBold))
        self._badge.setMinimumHeight(70)

        # ── Divider ──────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet('color: #E5E5EA; margin: 0 4px;')

        # ── Metrics row ──────────────────────────────────────────
        mono = QFont('JetBrains Mono', 10) if True else QFont('Consolas', 10)
        self._seq_lbl = QLabel('SEQ  —')
        self._met_lbl = QLabel('MET  —')
        for lbl in (self._seq_lbl, self._met_lbl):
            lbl.setFont(mono)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet('color: #8E8E93;')

        metrics = QHBoxLayout()
        metrics.addWidget(self._seq_lbl)
        metrics.addWidget(self._met_lbl)

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.addWidget(self._badge)
        layout.addWidget(sep)
        layout.addLayout(metrics)

        # ── Flash timer (ARMED only) ─────────────────────────────
        self._flash_timer = QTimer(self)
        self._flash_timer.setInterval(500)
        self._flash_timer.timeout.connect(self._do_flash)

        self._apply_state('IDLE')

    # ── Public ───────────────────────────────────────────────────

    def update_data(self, data: TelemetryData) -> None:
        name = data.state_name
        self._badge.setText(name.replace('_', ' '))
        self._seq_lbl.setText(f'SEQ  {data.seq}')
        met_s = data.timestamp_ms // 1000
        self._met_lbl.setText(f'MET  {met_s // 60:02d}:{met_s % 60:02d}')

        if name != self._current_state:
            self._current_state = name
            if name == 'ARMED':
                self._flash_on = True
                self._flash_timer.start()
                self._badge.setStyleSheet(_badge_css(_STATE_COLORS['ARMED']))
            else:
                self._flash_timer.stop()
                self._apply_state(name)

    # ── Private ──────────────────────────────────────────────────

    def _do_flash(self) -> None:
        self._flash_on = not self._flash_on
        self._badge.setStyleSheet(
            _badge_css(_STATE_COLORS['ARMED'] if self._flash_on else _ARMED_DIM)
        )

    def _apply_state(self, name: str) -> None:
        color = _STATE_COLORS.get(name, '#8E8E93')
        self._badge.setText(name.replace('_', ' '))
        self._badge.setStyleSheet(_badge_css(color))
