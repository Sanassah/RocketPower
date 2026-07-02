"""Scrolling event log -- dark terminal style."""

from datetime import datetime

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'

_MAX_LINES = 500

_COLORS = {
    'info':    '#64748B',
    'ok':      '#22C55E',
    'warn':    '#F59E0B',
    'error':   '#EF4444',
    'command': '#3B82F6',
    'state':   '#A78BFA',
}


class EventLog(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        self._last_state: int | None = None
        self._line_count = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(8)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('--')
        sym.setStyleSheet(f'color:{_TEXT};font-size:12px;background:transparent;border:none;')
        ttl = QLabel('EVENT LOG')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)
        self.hdr_layout = hdr   # allows main_window to inject action buttons

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont('JetBrains Mono', 11))
        self._text.setStyleSheet(f"""
            QTextEdit {{
                background-color: #0F0F10;
                color: #F1F5F9;
                border: 1px solid {_BORDER};
                border-radius: 6px;
                padding: 6px 10px;
                font-size: 11px;
            }}
        """)
        root.addWidget(self._text)

    def log(self, message: str, level: str = 'info') -> None:
        color = _COLORS.get(level, _COLORS['info'])
        ts    = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line  = (
            f'<span style="color:#64748B;font-family:monospace;">{ts}</span>'
            f'&nbsp;&nbsp;'
            f'<span style="color:{color};">{message}</span>'
        )
        self._text.append(line)
        self._line_count += 1

        if self._line_count > _MAX_LINES:
            cursor = self._text.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

        self._text.verticalScrollBar().setValue(
            self._text.verticalScrollBar().maximum()
        )

    def update_data(self, data: TelemetryData) -> None:
        if data.state != self._last_state:
            self._last_state = data.state
            self.log(f'State -> {data.state_name}  (seq {data.seq})', level='state')
