"""
Top navigation bar — minimal, Apple-style light header.
White background, thin bottom border, clean controls.
"""

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QPushButton, QComboBox, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui  import QFont

from core.serial_worker import list_serial_ports

_BAUD_RATES   = ['115200', '9600', '57600', '38400', '19200']
_DEFAULT_BAUD = '115200'

_BTN_CONNECT = """
    QPushButton {
        background-color: #0071E3;
        color: #FFFFFF;
        border: none;
        border-radius: 8px;
        font-size: 12px;
        font-weight: 700;
        padding: 6px 18px;
        letter-spacing: 0.3px;
    }
    QPushButton:hover   { background-color: #0077ED; }
    QPushButton:pressed { background-color: #005BBB; }
"""

_BTN_DISCONNECT = """
    QPushButton {
        background-color: transparent;
        color: #FF3B30;
        border: 1.5px solid #FF3B30;
        border-radius: 8px;
        font-size: 12px;
        font-weight: 700;
        padding: 6px 18px;
        letter-spacing: 0.3px;
    }
    QPushButton:hover   { background-color: #FFF0EF; }
    QPushButton:pressed { background-color: #FFE0DE; }
"""

_BTN_REFRESH = """
    QPushButton {
        background-color: #F2F2F7;
        color: #6E6E73;
        border: 1px solid #D2D2D7;
        border-radius: 6px;
        font-size: 13px;
        padding: 4px 8px;
        min-width: 0;
    }
    QPushButton:hover   { background-color: #E5E5EA; border-color: #8E8E93; }
    QPushButton:pressed { background-color: #D8D8DC; }
"""


def _vsep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.VLine)
    f.setFixedWidth(1)
    f.setFixedHeight(22)
    f.setStyleSheet('background-color: #E5E5EA;')
    return f


def _micro_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet('color: #8E8E93; font-size: 10px; font-weight: 600; letter-spacing: 0.5px;')
    return lbl


class ConnectionPanel(QWidget):
    connect_requested    = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self.setFixedHeight(56)
        self.setStyleSheet("""
            ConnectionPanel {
                background-color: #FFFFFF;
                border-bottom: 1px solid #E5E5EA;
            }
        """)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(12)

        # ── App title ──────────────────────────────────────────────
        title = QLabel('RocketPower GCS')
        title.setFont(QFont('Segoe UI', 15, QFont.Weight.Bold))
        title.setStyleSheet('color: #1D1D1F; letter-spacing: -0.3px; background: transparent;')
        layout.addWidget(title)

        layout.addSpacing(8)
        layout.addWidget(_vsep())
        layout.addSpacing(8)

        # ── Port ───────────────────────────────────────────────────
        layout.addWidget(_micro_label('PORT'))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(100)
        self._port_combo.setMaximumWidth(130)
        self._port_combo.setFixedHeight(32)
        layout.addWidget(self._port_combo)

        refresh_btn = QPushButton('↻')
        refresh_btn.setFixedSize(32, 32)
        refresh_btn.setStyleSheet(_BTN_REFRESH)
        refresh_btn.setToolTip('Refresh port list')
        refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addWidget(_vsep())

        # ── Baud ───────────────────────────────────────────────────
        layout.addWidget(_micro_label('BAUD'))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(_BAUD_RATES)
        self._baud_combo.setCurrentText(_DEFAULT_BAUD)
        self._baud_combo.setMinimumWidth(80)
        self._baud_combo.setMaximumWidth(95)
        self._baud_combo.setFixedHeight(32)
        layout.addWidget(self._baud_combo)

        layout.addWidget(_vsep())

        # ── Connect / Disconnect ───────────────────────────────────
        self._conn_btn = QPushButton('Connect')
        self._conn_btn.setFixedHeight(32)
        self._conn_btn.setMinimumWidth(100)
        self._conn_btn.setStyleSheet(_BTN_CONNECT)
        self._conn_btn.clicked.connect(self._on_button)
        layout.addWidget(self._conn_btn)

        # ── Status indicator ───────────────────────────────────────
        self._dot = QLabel('●')
        self._dot.setFont(QFont('Arial', 10))
        self._dot.setStyleSheet('color: #D2D2D7; background: transparent;')
        layout.addWidget(self._dot)

        self._status_lbl = QLabel('Not connected')
        self._status_lbl.setStyleSheet(
            'color: #8E8E93; font-size: 11px; background: transparent;'
        )
        layout.addWidget(self._status_lbl)

        layout.addStretch()

        # ── Stats ──────────────────────────────────────────────────
        self._stats_lbl = QLabel('0 pkts · 0.0 Hz')
        self._stats_lbl.setFont(QFont('JetBrains Mono', 10))
        self._stats_lbl.setStyleSheet('color: #C7C7CC; background: transparent;')
        layout.addWidget(self._stats_lbl)

        self._refresh_ports()

    # ── Public ────────────────────────────────────────────────────

    def set_connected(self, connected: bool, message: str) -> None:
        self._connected = connected
        if connected:
            self._dot.setStyleSheet('color: #34C759; background: transparent;')
            self._status_lbl.setStyleSheet('color: #34C759; font-size: 11px; background: transparent;')
            self._conn_btn.setText('Disconnect')
            self._conn_btn.setStyleSheet(_BTN_DISCONNECT)
            self._port_combo.setEnabled(False)
            self._baud_combo.setEnabled(False)
        else:
            self._dot.setStyleSheet('color: #FF3B30; background: transparent;')
            self._status_lbl.setStyleSheet('color: #6E6E73; font-size: 11px; background: transparent;')
            self._conn_btn.setText('Connect')
            self._conn_btn.setStyleSheet(_BTN_CONNECT)
            self._port_combo.setEnabled(True)
            self._baud_combo.setEnabled(True)
        self._status_lbl.setText(message)

    def update_stats(self, total: int, rate: float) -> None:
        self._stats_lbl.setText(f'{total} pkts · {rate:.1f} Hz')
        self._stats_lbl.setStyleSheet('color: #8E8E93; font-size: 10px; background: transparent;')

    # ── Private ───────────────────────────────────────────────────

    def _on_button(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
        else:
            port = self._port_combo.currentText()
            baud = int(self._baud_combo.currentText())
            if port:
                self.connect_requested.emit(port, baud)

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = list_serial_ports()
        self._port_combo.addItems(ports)
        if current in ports:
            self._port_combo.setCurrentText(current)
