"""
Pyro channel status — iOS-style pill badges: SAFE (grey) / ARMED (orange) / FIRED (red).
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QWidget
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_PYRO_NAMES  = {1: 'CH 1  —  Drogue', 2: 'CH 2  —  Main', 3: 'CH 3  —  Aux'}
PYRO_CHANNELS = 3

# (text, text-colour, background, border)
_SAFE  = ('SAFE',  '#8E8E93', '#F2F2F7', '#D2D2D7')
_ARMED = ('ARMED', '#FF9F0A', '#FFF4E0', '#FF9F0A')
_FIRED = ('FIRED', '#FF3B30', '#FFE5E4', '#FF3B30')


class _ChannelRow(QWidget):
    def __init__(self, ch: int, parent=None):
        super().__init__(parent)
        self._status = ''

        row = QHBoxLayout(self)
        row.setContentsMargins(4, 4, 4, 4)

        lbl = QLabel(_PYRO_NAMES[ch])
        lbl.setStyleSheet('color: #1D1D1F; font-size: 12px;')
        row.addWidget(lbl)
        row.addStretch()

        self._badge = QLabel('SAFE')
        self._badge.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._badge.setFixedWidth(62)
        self._badge.setFixedHeight(24)
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        row.addWidget(self._badge)

        self.set_status('SAFE')

    def set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        cfg = _FIRED if status == 'FIRED' else _ARMED if status == 'ARMED' else _SAFE
        text, fg, bg, border = cfg
        self._badge.setText(text)
        self._badge.setStyleSheet(
            f'color: {fg}; background-color: {bg};'
            f'border: 1px solid {border}; border-radius: 12px;'
            f'font-size: 10px; font-weight: 700; letter-spacing: 0.3px;'
        )


class PyroPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('PYRO CHANNELS', parent)
        self._rows: dict[int, _ChannelRow] = {}
        self._fired: set[int] = set()

        layout = QVBoxLayout(self)
        layout.setSpacing(2)
        layout.setContentsMargins(12, 8, 12, 12)

        for ch in range(1, PYRO_CHANNELS + 1):
            row = _ChannelRow(ch)
            self._rows[ch] = row
            layout.addWidget(row)

            if ch < PYRO_CHANNELS:
                div = QLabel()
                div.setFixedHeight(1)
                div.setStyleSheet('background-color: #F2F2F7;')
                layout.addWidget(div)

        layout.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        armed = data.state >= 1

        for ch, row in self._rows.items():
            if ch in self._fired:
                row.set_status('FIRED')
            elif armed:
                row.set_status('ARMED')
            else:
                row.set_status('SAFE')

        if data.state >= 5 and 1 not in self._fired:
            self._fired.add(1)
        if data.state >= 6 and 2 not in self._fired:
            self._fired.add(2)

        if data.state == 0:
            self._fired.clear()

    def mark_fired(self, channel: int) -> None:
        self._fired.add(channel)
        if channel in self._rows:
            self._rows[channel].set_status('FIRED')
