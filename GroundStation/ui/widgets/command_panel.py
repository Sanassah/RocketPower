"""ARM / FIRE / UTILITY commands -- dark dashboard style, two-column layout."""

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

FIN_CHANNELS = 4
_COMPASS = {1: 'S', 2: 'E', 3: 'N', 4: 'W'}


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet(f'background-color:{_BORDER};margin:2px 0;')
    return f


def _vdivider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.VLine)
    f.setFixedWidth(1)
    f.setStyleSheet(f'background-color:{_BORDER};margin:0 2px;')
    return f


def _section(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:0.8px;'
        f'padding-top:1px;border:none;background:transparent;'
    )
    return lbl


def _flat_btn(text: str, height: int = 26, width: int = 0, font_size: int = 10) -> QPushButton:
    b = QPushButton(text)
    b.setFixedHeight(height)
    if width:
        b.setFixedWidth(width)
    b.setFont(QFont('Segoe UI', font_size, QFont.Weight.Bold))
    b.setStyleSheet(f"""
        QPushButton {{
            background-color:{_CARD2};color:{_TEXT};
            border:1px solid {_BORDER};border-radius:5px;
        }}
        QPushButton:hover {{ background-color:#222436; }}
    """)
    return b


class CommandPanel(QWidget):
    arm_requested             = pyqtSignal()
    disarm_requested          = pyqtSignal()
    fire_pyro_requested       = pyqtSignal(int)
    ping_requested            = pyqtSignal()
    calibrate_requested       = pyqtSignal()
    servo_test_requested      = pyqtSignal(int)
    cam_toggle_requested      = pyqtSignal()
    servo_nudge_requested     = pyqtSignal(int, bool)   # (channel, positive)
    servo_save_cal_requested  = pyqtSignal()
    servo_center_requested    = pyqtSignal()
    servo_preflight_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        self._state = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 8, 14, 10)
        root.setSpacing(4)

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

        columns = QHBoxLayout()
        columns.setSpacing(10)
        col_a = QVBoxLayout(); col_a.setSpacing(4)
        col_b = QVBoxLayout(); col_b.setSpacing(4)
        columns.addLayout(col_a, 1)
        columns.addWidget(_vdivider())
        columns.addLayout(col_b, 1)
        root.addLayout(columns)

        # ==== Column A: flight commands (ARM / IGNITION / UTILITIES) ====

        col_a.addWidget(_section('ARM CONTROL'))
        arm_row = QHBoxLayout()
        arm_row.setSpacing(6)

        self._arm_btn = QPushButton('ARM')
        self._arm_btn.setFixedHeight(32)
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
        self._disarm_btn.setFixedHeight(32)
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
        col_a.addLayout(arm_row)
        col_a.addWidget(_divider())

        # Pyro fire
        col_a.addWidget(_section('IGNITION CONTROL'))
        note = QLabel('Available from ARMED through DESCENT')
        note.setStyleSheet(
            f'color:{_MUTED};font-size:11px;border:none;background:transparent;'
        )
        col_a.addWidget(note)

        self._fire_btns: list[QPushButton] = []
        for ch in range(1, PYRO_CHANNELS + 1):
            btn = QPushButton(f'Fire  {_FIRE_NAMES[ch - 1]}')
            btn.setFixedHeight(28)
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
            col_a.addWidget(btn)

        col_a.addWidget(_divider())

        # Utilities
        col_a.addWidget(_section('UTILITIES'))
        util_row = QHBoxLayout()
        util_row.setSpacing(6)

        for label, slot in [('Calibrate Baro', self._on_calibrate),
                            ('Ping', self.ping_requested.emit)]:
            btn = _flat_btn(label, height=26)
            btn.clicked.connect(slot)
            util_row.addWidget(btn)

        col_a.addLayout(util_row)
        col_a.addStretch()

        # ==== Column B: bench-test commands (FIN SERVOS / CAMERA) ====

        col_b.addWidget(_section('FIN SERVOS  (BENCH TEST)'))

        preflight_btn = QPushButton('▶  PREFLIGHT  (S→E→N→W, ~6s)')
        preflight_btn.setFixedHeight(28)
        preflight_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        preflight_btn.clicked.connect(self.servo_preflight_requested.emit)
        preflight_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_BLUE};
                border:1px solid {_BLUE};border-radius:6px;font-weight:700;
            }}
            QPushButton:hover {{ background-color:#0A1830; }}
        """)
        col_b.addWidget(preflight_btn)

        fin_note = QLabel('Test = sweep.  −/+ = nudge trim against your jig.')
        fin_note.setStyleSheet(
            f'color:{_MUTED};font-size:11px;border:none;background:transparent;'
        )
        col_b.addWidget(fin_note)

        for ch in range(1, FIN_CHANNELS + 1):
            ch_row = QHBoxLayout()
            ch_row.setSpacing(4)

            lbl = QLabel(f'CH{ch} ({_COMPASS[ch]})')
            lbl.setFixedWidth(52)
            lbl.setStyleSheet(
                f'color:{_MUTED};font-size:10px;font-weight:700;'
                f'border:none;background:transparent;'
            )
            test_btn  = _flat_btn('Test', height=23)
            minus_btn = _flat_btn('−', height=23, width=28)
            plus_btn  = _flat_btn('+', height=23, width=28)
            test_btn.clicked.connect(lambda checked, c=ch: self.servo_test_requested.emit(c))
            minus_btn.clicked.connect(lambda checked, c=ch: self.servo_nudge_requested.emit(c, False))
            plus_btn.clicked.connect(lambda checked, c=ch: self.servo_nudge_requested.emit(c, True))

            ch_row.addWidget(lbl)
            ch_row.addWidget(test_btn)
            ch_row.addWidget(minus_btn)
            ch_row.addWidget(plus_btn)
            col_b.addLayout(ch_row)

        cal_row = QHBoxLayout()
        cal_row.setSpacing(6)

        center_btn = _flat_btn('Center All', height=26)
        center_btn.setToolTip('Drive all 4 to raw center -- for mounting fins')
        center_btn.clicked.connect(self.servo_center_requested.emit)

        save_cal_btn = QPushButton('Save Calibration')
        save_cal_btn.setFixedHeight(26)
        save_cal_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        save_cal_btn.clicked.connect(self._on_save_calibration)
        save_cal_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_GREEN};
                border:1px solid {_GREEN};border-radius:6px;font-weight:700;
            }}
            QPushButton:hover {{ background-color:#0A2010; }}
        """)

        cal_row.addWidget(center_btn)
        cal_row.addWidget(save_cal_btn)
        col_b.addLayout(cal_row)
        col_b.addWidget(_divider())

        # Camera -- the camera only exposes a toggle (see CameraController),
        # so one button is all there is: it sends CAM_TOGGLE and its look is
        # kept in sync with the FC's reported cam_recording state on every
        # telemetry packet. That state is the FC's own belief, not a
        # camera-confirmed one -- there's no ack on that link.
        col_b.addWidget(_section('CAMERA  (BENCH TEST)'))
        self._cam_btn = QPushButton('⏺  Toggle Recording')
        self._cam_btn.setFixedHeight(28)
        self._cam_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._cam_btn.setCheckable(True)
        self._cam_btn.clicked.connect(self.cam_toggle_requested.emit)
        self._cam_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_TEXT};
                border:1px solid {_BORDER};border-radius:6px;font-weight:700;
            }}
            QPushButton:hover {{ background-color:#222436; }}
            QPushButton:checked {{
                background-color:#200A0A;color:{_RED};border:1px solid {_RED};
            }}
        """)
        col_b.addWidget(self._cam_btn)
        col_b.addStretch()

        self._update_button_states()

    def update_data(self, data: TelemetryData) -> None:
        if data.state != self._state:
            self._state = data.state
            self._update_button_states()
        self._cam_btn.setChecked(data.is_recording)
        self._cam_btn.setText('⏺  Recording -- Tap to Stop' if data.is_recording else '⏺  Toggle Recording')

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

    def _on_save_calibration(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm Save Calibration',
            'Save the current fin positions as the new zero for all 4 channels?\n\n'
            'This overwrites the previous calibration and takes effect on the next boot.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.servo_save_cal_requested.emit()

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
