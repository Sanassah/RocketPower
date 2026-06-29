"""
Scrolling event log — clean white background, monospace timestamps,
iOS-palette level colours, subtle alternating row tints.
"""

from datetime import datetime

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QTextEdit
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QFont

from core.packet_decoder import TelemetryData

_MAX_LINES = 500

# iOS / Apple HIG event colours
_COLORS = {
    'info':    '#6E6E73',   # grey
    'ok':      '#34C759',   # green
    'warn':    '#FF9F0A',   # orange
    'error':   '#FF3B30',   # red
    'command': '#0071E3',   # blue
    'state':   '#BF5AF2',   # purple
}

# Alternating row tints (very subtle)
_ROW_A = '#FFFFFF'
_ROW_B = '#FAFAFA'


class EventLog(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('EVENT LOG', parent)
        self._last_state: int | None = None
        self._line_count = 0

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont('JetBrains Mono', 10))
        self._text.setStyleSheet("""
            QTextEdit {
                background-color: #FFFFFF;
                color: #1D1D1F;
                border: none;
                padding: 4px 8px;
                font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
                font-size: 10px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.addWidget(self._text)

    def log(self, message: str, level: str = 'info') -> None:
        color  = _COLORS.get(level, _COLORS['info'])
        ts     = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        row_bg = _ROW_B if self._line_count % 2 else _ROW_A

        line = (
            f'<div style="background-color:{row_bg}; padding:2px 8px;">'
            f'<span style="color:#C7C7CC; font-family:monospace;">{ts}</span>'
            f'&nbsp;&nbsp;'
            f'<span style="color:{color};">{message}</span>'
            f'</div>'
        )
        self._text.append(line)
        self._line_count += 1

        # Trim oldest entry when over limit
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
            self.log(f'State → {data.state_name}  (seq {data.seq})', level='state')
