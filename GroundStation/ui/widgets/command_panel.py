"""
ARM / DISARM / FIRE PYRO command buttons — Apple-style solid fills.
ARM = solid blue, DISARM = outlined blue, FIRE = solid red.
Confirmation dialogs unchanged.
"""

from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLabel, QFrame
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui  import QFont

from core.packet_decoder import TelemetryData

PYRO_CHANNELS = 3

_FIRE_NAMES = ['CH 1  Drogue', 'CH 2  Main', 'CH 3  Aux']


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet('background-color: #F2F2F7; margin: 4px 0;')
    return f


class CommandPanel(QGroupBox):
    arm_requested       = pyqtSignal()
    disarm_requested    = pyqtSignal()
    fire_pyro_requested = pyqtSignal(int)
    ping_requested      = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__('COMMANDS', parent)
        self._state = 0

        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 8, 12, 12)

        # ── ARM / DISARM ──────────────────────────────────────────
        arm_row = QHBoxLayout()
        arm_row.setSpacing(8)

        self._arm_btn = QPushButton('ARM')
        self._arm_btn.setObjectName('armButton')
        self._arm_btn.setFixedHeight(38)
        self._arm_btn.setFont(QFont('Segoe UI', 12, QFont.Weight.Bold))
        self._arm_btn.clicked.connect(self._on_arm)

        self._disarm_btn = QPushButton('DISARM')
        self._disarm_btn.setObjectName('disarmButton')
        self._disarm_btn.setFixedHeight(38)
        self._disarm_btn.setFont(QFont('Segoe UI', 12, QFont.Weight.Bold))
        self._disarm_btn.clicked.connect(self._on_disarm)

        arm_row.addWidget(self._arm_btn)
        arm_row.addWidget(self._disarm_btn)
        layout.addLayout(arm_row)

        layout.addWidget(_divider())

        # ── Fire section header ───────────────────────────────────
        fire_hdr = QLabel('IGNITION CONTROL')
        fire_hdr.setStyleSheet(
            'color: #8E8E93; font-size: 10px; font-weight: 700; letter-spacing: 0.5px;'
        )
        layout.addWidget(fire_hdr)

        fire_note = QLabel('Available from ARMED through DESCENT')
        fire_note.setStyleSheet('color: #C7C7CC; font-size: 10px;')
        layout.addWidget(fire_note)

        # ── FIRE buttons ──────────────────────────────────────────
        self._fire_btns: list[QPushButton] = []
        for ch in range(1, PYRO_CHANNELS + 1):
            btn = QPushButton(f'Fire  {_FIRE_NAMES[ch - 1]}')
            btn.setObjectName('fireButton')
            btn.setFixedHeight(34)
            btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
            btn.setEnabled(False)
            btn.clicked.connect(lambda checked, c=ch: self._on_fire(c))
            self._fire_btns.append(btn)
            layout.addWidget(btn)

        layout.addWidget(_divider())

        # ── Ping ──────────────────────────────────────────────────
        ping_btn = QPushButton('Ping')
        ping_btn.setFixedHeight(30)
        ping_btn.setFont(QFont('Segoe UI', 11))
        ping_btn.clicked.connect(self.ping_requested.emit)
        layout.addWidget(ping_btn)

        layout.addStretch()
        self._update_button_states()

    def update_data(self, data: TelemetryData) -> None:
        if data.state != self._state:
            self._state = data.state
            self._update_button_states()

    def _update_button_states(self) -> None:
        # Allow manual pyro override from ARMED through DESCENT (states 1–5).
        # IDLE and LANDED are the only safe states where firing must be blocked.
        can_fire = self._state in {1, 2, 3, 4, 5}
        for btn in self._fire_btns:
            btn.setEnabled(can_fire)

    def _on_arm(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm ARM',
            'Send ARM command to the flight computer?\n\nThis will arm the pyro channels.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.arm_requested.emit()

    def _on_disarm(self) -> None:
        self.disarm_requested.emit()

    def _on_fire(self, channel: int) -> None:
        names = {1: 'Drogue', 2: 'Main', 3: 'Aux'}
        reply = QMessageBox.warning(
            self, f'Confirm Fire CH{channel}',
            f'Fire pyro channel {channel} ({names.get(channel, "")})?\n\nThis is IRREVERSIBLE.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.fire_pyro_requested.emit(channel)
