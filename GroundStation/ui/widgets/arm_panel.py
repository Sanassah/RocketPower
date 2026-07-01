"""ARM / SAFETY STATUS card."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
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

_STATE_COLOR = {
    'IDLE':           '#64748B',
    'ARMED':          _BLUE,
    'POWERED_ASCENT': _ORANGE,
    'COAST':          _GREEN,
    'APOGEE':         '#A78BFA',
    'DESCENT':        '#67E8F9',
    'LANDED':         _GREEN,
}

_STATE_SUB = {
    'IDLE':           'ROCKET IS SAFE',
    'ARMED':          'PYROS HOT  --  STAND CLEAR',
    'POWERED_ASCENT': 'MOTOR BURNING',
    'COAST':          'COASTING TO APOGEE',
    'APOGEE':         'AT APOGEE',
    'DESCENT':        'DESCENDING',
    'LANDED':         'SAFE TO APPROACH',
}


def _card_header(symbol: str, title: str) -> QHBoxLayout:
    h = QHBoxLayout(); h.setSpacing(8)
    sym = QLabel(symbol)
    sym.setStyleSheet(f'color:{_TEXT};font-size:12px;background:transparent;border:none;')
    lbl = QLabel(title)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:1px;'
        f'background:transparent;border:none;'
    )
    h.addWidget(sym); h.addWidget(lbl); h.addStretch()
    return h


class ArmPanel(QWidget):
    arm_requested    = pyqtSignal()
    disarm_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        self._state_name = 'IDLE'
        self._flash_on   = True

        self._flash_timer = QTimer(self)
        self._flash_timer.setInterval(500)
        self._flash_timer.timeout.connect(self._do_flash)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 16)
        root.setSpacing(0)

        root.addLayout(_card_header('[]', 'ARM / SAFETY STATUS'))

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1); sep.setStyleSheet(f'background:{_BORDER};margin:8px 0;')
        root.addWidget(sep)

        # Shield symbol + state
        centre = QVBoxLayout()
        centre.setSpacing(4)
        centre.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._shield = QLabel('[ ]')
        self._shield.setFont(QFont('Segoe UI', 28, QFont.Weight.Light))
        self._shield.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._shield.setStyleSheet(f'color:{_MUTED};background:transparent;border:none;')

        self._state_lbl = QLabel('IDLE')
        self._state_lbl.setFont(QFont('Segoe UI', 26, QFont.Weight.ExtraBold))
        self._state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state_lbl.setStyleSheet(
            f'color:{_MUTED};letter-spacing:3px;background:transparent;border:none;'
        )

        self._sub_lbl = QLabel('ROCKET IS SAFE')
        self._sub_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._sub_lbl.setStyleSheet(
            f'color:{_MUTED};font-size:12px;letter-spacing:1px;background:transparent;border:none;'
        )

        centre.addStretch()
        centre.addWidget(self._shield)
        centre.addWidget(self._state_lbl)
        centre.addWidget(self._sub_lbl)
        centre.addStretch()
        root.addLayout(centre)

        # ARM button (full width, slide-to-arm style)
        self._arm_btn = QPushButton('ARM ROCKET   >')
        self._arm_btn.setFixedHeight(40)
        self._arm_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._arm_btn.clicked.connect(self._on_arm)
        self._arm_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};
                color:{_MUTED};
                border:1px solid {_BORDER};
                border-radius:6px;
                font-weight:700;
                letter-spacing:1px;
            }}
            QPushButton:enabled {{
                background-color:{_CARD2};
                color:{_TEXT};
                border:1px solid {_BLUE};
            }}
            QPushButton:enabled:hover {{ background-color:#1E2535; }}
            QPushButton:enabled:pressed {{ background-color:#14151E; }}
            QPushButton:disabled {{ color:{_MUTED}; border-color:{_BORDER}; }}
        """)

        self._disarm_btn = QPushButton('DISARM')
        self._disarm_btn.setFixedHeight(40)
        self._disarm_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._disarm_btn.setEnabled(False)
        self._disarm_btn.clicked.connect(self.disarm_requested.emit)
        self._disarm_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:transparent;
                color:{_RED};
                border:1px solid {_RED};
                border-radius:6px;
                font-weight:700;
                letter-spacing:1px;
            }}
            QPushButton:hover {{ background-color:#200A0A; }}
            QPushButton:disabled {{ color:{_BORDER}; border-color:{_BORDER}; }}
        """)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        btn_row.addWidget(self._arm_btn)
        btn_row.addWidget(self._disarm_btn)
        root.addLayout(btn_row)

    def update_data(self, data: TelemetryData) -> None:
        sn = data.state_name
        if sn == self._state_name:
            return
        self._state_name = sn
        color = _STATE_COLOR.get(sn, _MUTED)

        self._state_lbl.setText(sn.replace('_', ' '))
        self._state_lbl.setStyleSheet(
            f'color:{color};font-size:26px;font-weight:800;letter-spacing:3px;'
            f'background:transparent;border:none;'
        )
        self._shield.setStyleSheet(
            f'color:{color};font-size:28px;background:transparent;border:none;'
        )
        self._sub_lbl.setText(_STATE_SUB.get(sn, ''))

        is_idle  = sn == 'IDLE'
        is_armed = sn == 'ARMED'
        self._arm_btn.setEnabled(is_idle)
        self._disarm_btn.setEnabled(is_armed)

        if is_armed:
            self._flash_on = True
            self._flash_timer.start()
        else:
            self._flash_timer.stop()

    def _do_flash(self) -> None:
        self._flash_on = not self._flash_on
        c = _BLUE if self._flash_on else _MUTED
        self._state_lbl.setStyleSheet(
            f'color:{c};font-size:26px;font-weight:800;letter-spacing:3px;'
            f'background:transparent;border:none;'
        )

    def _on_arm(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm ARM',
            'Send ARM command to the flight computer?\n\nThis will arm the pyro channels.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.arm_requested.emit()
