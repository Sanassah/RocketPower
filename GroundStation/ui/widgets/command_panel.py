"""ARM / FIRE / UTILITY commands -- dark dashboard style."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QMessageBox, QLabel, QFrame
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui  import QFont

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_CARD2  = '#222224'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_GREEN  = '#22C55E'
_ORANGE = '#F59E0B'
_RED    = '#EF4444'
_BLUE   = '#3B82F6'

PYRO_CHANNELS = 3
_FIRE_NAMES = ['CH 1  --  Ignition', 'CH 2  --  Parachute', 'CH 3  --  Backup']


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet(f'background-color:{_BORDER};margin:4px 0;')
    return f


def _section(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:0.8px;'
        f'padding-top:4px;border:none;background:transparent;'
    )
    return lbl


class CommandPanel(QWidget):
    arm_requested       = pyqtSignal()
    disarm_requested    = pyqtSignal()
    fire_pyro_requested = pyqtSignal(int)
    ping_requested      = pyqtSignal()
    calibrate_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        self._state = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 14)
        root.setSpacing(6)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('>_')
        sym.setStyleSheet(
            f'color:{_TEXT};font-size:12px;background:transparent;border:none;'
        )
        ttl = QLabel('COMMANDS')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)
        root.addWidget(_divider())

        # ARM / DISARM
        root.addWidget(_section('ARM CONTROL'))
        arm_row = QHBoxLayout()
        arm_row.setSpacing(6)

        self._arm_btn = QPushButton('ARM')
        self._arm_btn.setFixedHeight(36)
        self._arm_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._arm_btn.clicked.connect(self._on_arm)
        self._arm_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_BLUE};color:#FFF;border:none;
                border-radius:6px;font-weight:700;letter-spacing:0.5px;
            }}
            QPushButton:hover {{ background-color:#60A5FA; }}
            QPushButton:pressed {{ background-color:#2563EB; }}
        """)

        self._disarm_btn = QPushButton('DISARM')
        self._disarm_btn.setFixedHeight(36)
        self._disarm_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._disarm_btn.clicked.connect(self._on_disarm)
        self._disarm_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:transparent;color:{_RED};
                border:1px solid {_RED};border-radius:6px;
                font-weight:700;letter-spacing:0.5px;
            }}
            QPushButton:hover {{ background-color:#200A0A; }}
        """)

        arm_row.addWidget(self._arm_btn)
        arm_row.addWidget(self._disarm_btn)
        root.addLayout(arm_row)
        root.addWidget(_divider())

        # Pyro fire
        root.addWidget(_section('IGNITION CONTROL'))
        note = QLabel('Available from ARMED through DESCENT')
        note.setStyleSheet(
            f'color:{_MUTED};font-size:11px;border:none;background:transparent;'
        )
        root.addWidget(note)

        self._fire_btns: list[QPushButton] = []
        for ch in range(1, PYRO_CHANNELS + 1):
            btn = QPushButton(f'Fire  {_FIRE_NAMES[ch - 1]}')
            btn.setFixedHeight(32)
            btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
            btn.setEnabled(False)
            btn.clicked.connect(lambda checked, c=ch: self._on_fire(c))
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color:{_CARD2};color:{_RED};
                    border:1px solid {_RED};border-radius:6px;font-weight:700;
                    letter-spacing:0.3px;
                }}
                QPushButton:hover   {{ background-color:#200A0A; }}
                QPushButton:pressed {{ background-color:#1A0505; }}
                QPushButton:disabled {{
                    background-color:{_CARD2};color:{_BORDER};
                    border-color:{_BORDER};
                }}
            """)
            self._fire_btns.append(btn)
            root.addWidget(btn)

        root.addWidget(_divider())

        # Utilities
        root.addWidget(_section('UTILITIES'))
        util_row = QHBoxLayout()
        util_row.setSpacing(6)

        for label, slot in [('Calibrate Baro', self._on_calibrate),
                            ('Ping', self.ping_requested.emit)]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.setFont(QFont('Segoe UI', 10))
            btn.clicked.connect(slot)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color:{_CARD2};color:{_TEXT};
                    border:1px solid {_BORDER};border-radius:6px;
                }}
                QPushButton:hover {{ background-color:#222436; }}
            """)
            util_row.addWidget(btn)

        root.addLayout(util_row)
        root.addStretch()
        self._update_button_states()

    def update_data(self, data: TelemetryData) -> None:
        if data.state != self._state:
            self._state = data.state
            self._update_button_states()

    def _update_button_states(self) -> None:
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

    def _on_calibrate(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm Calibrate',
            'Re-zero the barometer at the current altitude?\n\nDo this only when the rocket is on the ground.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.calibrate_requested.emit()

    def _on_fire(self, channel: int) -> None:
        names = {1: 'Ignition', 2: 'Parachute', 3: 'Backup'}
        reply = QMessageBox.warning(
            self, f'Confirm Fire CH{channel}',
            f'Fire pyro channel {channel} ({names.get(channel, "")})?\n\nThis is IRREVERSIBLE.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.fire_pyro_requested.emit(channel)
