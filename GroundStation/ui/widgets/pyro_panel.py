"""Pyro channel status card."""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'   # lighter muted so text is readable
_DIM    = '#1E1E20'
_GREEN  = '#22C55E'
_ORANGE = '#F59E0B'
_RED    = '#EF4444'

_PYRO_NAMES = {1: 'Ignition', 2: 'Parachute', 3: 'Backup'}
PYRO_CHANNELS = 3

_IDLE  = ('IDLE',  '#94A3B8', '#222226', '#3A3A3E')
_ARMED = ('ARMED', _ORANGE,   '#251C00', _ORANGE)
_FIRED = ('FIRED', _RED,      '#200800', _RED)


class _ChannelRow(QWidget):
    def __init__(self, ch: int, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet('background:transparent;')
        self._status = ''

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 14, 0, 14)   # generous vertical padding
        row.setSpacing(10)

        # Continuity dot
        self._cont_dot = QLabel('●')
        self._cont_dot.setFont(QFont('Segoe UI', 15))
        self._cont_dot.setFixedWidth(22)
        self._cont_dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._cont_dot.setStyleSheet(f'color:{_MUTED};border:none;background:transparent;')

        # Channel number label ("PYRO 1")
        ch_lbl = QLabel(f'PYRO {ch}')
        ch_lbl.setFont(QFont('Segoe UI', 12, QFont.Weight.Bold))
        ch_lbl.setStyleSheet(
            f'color:{_TEXT};font-size:12px;font-weight:700;letter-spacing:0.5px;'
            f'border:none;background:transparent;min-width:60px;'
        )

        # Function name ("Ignition" / "Parachute" / "Backup")
        name_lbl = QLabel(_PYRO_NAMES[ch])
        name_lbl.setFont(QFont('Segoe UI', 15))
        name_lbl.setStyleSheet(
            f'color:{_TEXT};font-size:15px;border:none;background:transparent;'
        )

        # Status badge
        self._badge = QLabel('IDLE')
        self._badge.setFixedWidth(74)
        self._badge.setFixedHeight(27)
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setFont(QFont('Segoe UI', 12, QFont.Weight.Bold))

        row.addWidget(self._cont_dot)
        row.addWidget(ch_lbl)
        row.addWidget(name_lbl)
        row.addStretch()
        row.addWidget(self._badge)

        self.set_status('IDLE')

    def set_continuity(self, ok: bool) -> None:
        color = _GREEN if ok else _RED
        self._cont_dot.setStyleSheet(
            f'color:{color};border:none;background:transparent;'
        )
        self._cont_dot.setToolTip('Continuity OK' if ok else 'Open circuit')

    def set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        text, fg, bg, border = (
            _FIRED if status == 'FIRED' else
            _ARMED if status == 'ARMED' else
            _IDLE
        )
        self._badge.setText(text)
        self._badge.setStyleSheet(
            f'color:{fg};background-color:{bg};border:1px solid {border};'
            f'border-radius:13px;font-size:12px;font-weight:700;letter-spacing:0.5px;'
        )


class PyroPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:none;border-radius:10px;'
        )
        self._rows: dict[int, _ChannelRow] = {}
        self._fired: set[int] = set()

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 16)
        root.setSpacing(0)

        # Header
        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('/\\')
        sym.setStyleSheet(f'color:{_TEXT};font-size:14px;background:transparent;border:none;')
        ttl = QLabel('PYRO CHANNELS')
        ttl.setFont(QFont('Segoe UI', 13, QFont.Weight.Bold))
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1)
        sep.setStyleSheet('background-color:rgba(255,255,255,18);margin:6px 0;')
        root.addWidget(sep)

        for ch in range(1, PYRO_CHANNELS + 1):
            row = _ChannelRow(ch)
            self._rows[ch] = row
            root.addWidget(row)
            if ch < PYRO_CHANNELS:
                d = QFrame(); d.setFrameShape(QFrame.Shape.HLine)
                d.setFixedHeight(1)
                d.setStyleSheet('background-color:rgba(255,255,255,18);')
                root.addWidget(d)

        root.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        armed = data.state >= 1
        for ch, row in self._rows.items():
            row.set_continuity(data.pyro_continuity[ch - 1])
            if ch in self._fired:
                row.set_status('FIRED')
            elif armed:
                row.set_status('ARMED')
            else:
                row.set_status('IDLE')

        if data.state >= 2 and 1 not in self._fired:
            self._fired.add(1)
        if data.state >= 5 and 2 not in self._fired:
            self._fired.add(2)
        if data.state == 0:
            self._fired.clear()

    def mark_fired(self, channel: int) -> None:
        self._fired.add(channel)
        if channel in self._rows:
            self._rows[channel].set_status('FIRED')
