"""CONTINUITY STATUS card -- pyro circuit health."""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'   # lighter so labels are readable
_GREEN  = '#22C55E'
_RED    = '#EF4444'

_CH_NAMES = [
    ('PYRO 1', 'PARACHUTE'),
    ('PYRO 2', 'RESERVED'),
    ('PYRO 3', 'BACKUP'),
]


class _CircuitRow(QWidget):
    def __init__(self, ch_label: str, func_label: str, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet('background:transparent;')

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 14, 0, 14)   # generous vertical padding
        row.setSpacing(10)

        # Status circle icon
        self._icon = QLabel('●')
        self._icon.setFont(QFont('Segoe UI', 16))
        self._icon.setFixedWidth(24)
        self._icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._icon.setStyleSheet(f'color:{_MUTED};border:none;background:transparent;')

        # Channel number ("PYRO 1")
        ch_lbl = QLabel(ch_label)
        ch_lbl.setFont(QFont('Segoe UI', 12, QFont.Weight.Bold))
        ch_lbl.setStyleSheet(
            f'color:{_TEXT};font-size:12px;font-weight:700;letter-spacing:0.5px;'
            f'border:none;background:transparent;min-width:58px;'
        )

        # Function name ("PARACHUTE")
        fn_lbl = QLabel(func_label)
        fn_lbl.setFont(QFont('Segoe UI', 15))
        fn_lbl.setStyleSheet(
            f'color:{_TEXT};font-size:15px;border:none;background:transparent;'
        )

        # Status text ("CONTINUOUS" / "OPEN CIRCUIT")
        self._status = QLabel('--')
        self._status.setFont(QFont('Segoe UI', 14, QFont.Weight.Bold))
        self._status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._status.setStyleSheet(
            f'color:{_MUTED};font-size:14px;font-weight:700;border:none;background:transparent;'
        )

        row.addWidget(self._icon)
        row.addWidget(ch_lbl)
        row.addWidget(fn_lbl)
        row.addStretch()
        row.addWidget(self._status)

    def set_ok(self, ok: bool) -> None:
        color = _GREEN if ok else _RED
        self._icon.setStyleSheet(f'color:{color};border:none;background:transparent;')
        self._status.setText('CONTINUOUS' if ok else 'OPEN CIRCUIT')
        self._status.setStyleSheet(
            f'color:{color};font-size:14px;font-weight:700;border:none;background:transparent;'
        )


class ContinuityPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:none;border-radius:10px;'
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 16)
        root.setSpacing(0)

        # Header
        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('()')
        sym.setStyleSheet(f'color:{_TEXT};font-size:14px;background:transparent;border:none;')
        ttl = QLabel('CONTINUITY STATUS')
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

        self._rows: list[_CircuitRow] = []
        for i, (ch_lbl, fn_lbl) in enumerate(_CH_NAMES):
            row = _CircuitRow(ch_lbl, fn_lbl)
            self._rows.append(row)
            root.addWidget(row)
            if i < len(_CH_NAMES) - 1:
                d = QFrame(); d.setFrameShape(QFrame.Shape.HLine)
                d.setFixedHeight(1)
                d.setStyleSheet('background-color:rgba(255,255,255,18);')
                root.addWidget(d)

        root.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        for i, row in enumerate(self._rows):
            row.set_ok(data.pyro_continuity[i])
