"""FIRE / ATTITUDE / UTILITY commands -- dark dashboard style, two-column
layout. ARM/DISARM lives in the OVERVIEW page's ArmPanel, not here."""

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
            padding:0;
        }}
        QPushButton:hover {{ background-color:#222436; }}
    """)
    return b


class CommandPanel(QWidget):
    fire_pyro_requested       = pyqtSignal(int)
    calibrate_requested       = pyqtSignal()
    servo_test_requested      = pyqtSignal(int)
    cam_toggle_requested      = pyqtSignal()
    servo_nudge_requested     = pyqtSignal(int, bool)   # (channel, positive)
    servo_save_cal_requested  = pyqtSignal()
    servo_center_requested    = pyqtSignal()
    servo_preflight_requested = pyqtSignal()
    sd_start_requested        = pyqtSignal()
    sd_stop_requested         = pyqtSignal()
    attitude_control_enable_requested  = pyqtSignal()
    attitude_control_disable_requested = pyqtSignal()
    attitude_demo_enable_requested     = pyqtSignal()
    attitude_demo_disable_requested    = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        self._state       = 0
        self._sd_present  = False
        self._sd_recording = False
        self._attitude_control_on = False
        self._attitude_demo_on    = False

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

        # ==== Column A: flight commands (IGNITION / ATTITUDE / UTILITIES) ====
        # ARM/DISARM deliberately not duplicated here -- it already lives in
        # the OVERVIEW page's ArmPanel.

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

        # Attitude control -- runtime mode toggles, ground-commanded (not
        # compile-time flags -- see AttitudeController). REAL engages fin
        # correction during POWERED_ASCENT/COAST; DEMO does the same but only
        # while ARMED, for hand-rotating the airframe on the bench and
        # watching the fins react. Both default off at boot and reset off on
        # DISARM. Live confirmed state shows in the sensor panel's STATUS
        # section, not here -- these are just the action buttons.
        col_a.addWidget(_section('ATTITUDE CONTROL'))

        real_row = QHBoxLayout(); real_row.setSpacing(6)
        real_lbl = QLabel('Real (in-flight)')
        real_lbl.setFixedWidth(96)
        real_lbl.setStyleSheet(f'color:{_MUTED};font-size:11px;border:none;background:transparent;')
        self._att_ctrl_btn = QPushButton('OFF')
        self._att_ctrl_btn.setFixedHeight(24)
        self._att_ctrl_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._att_ctrl_btn.setCheckable(True)
        self._att_ctrl_btn.clicked.connect(self._on_attitude_control_toggled)
        self._att_ctrl_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_TEXT};
                border:1px solid {_BORDER};border-radius:5px;
            }}
            QPushButton:hover {{ background-color:#222436; }}
            QPushButton:checked {{
                background-color:#2A0A0A;color:{_RED};border:1px solid {_RED};
            }}
        """)
        real_row.addWidget(real_lbl)
        real_row.addWidget(self._att_ctrl_btn)
        col_a.addLayout(real_row)

        demo_row = QHBoxLayout(); demo_row.setSpacing(6)
        demo_lbl = QLabel('Demo (armed only)')
        demo_lbl.setFixedWidth(96)
        demo_lbl.setStyleSheet(f'color:{_MUTED};font-size:11px;border:none;background:transparent;')
        self._att_demo_btn = QPushButton('OFF')
        self._att_demo_btn.setFixedHeight(24)
        self._att_demo_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._att_demo_btn.setCheckable(True)
        self._att_demo_btn.clicked.connect(self._on_attitude_demo_toggled)
        self._att_demo_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_TEXT};
                border:1px solid {_BORDER};border-radius:5px;
            }}
            QPushButton:hover {{ background-color:#222436; }}
            QPushButton:checked {{
                background-color:#0A2010;color:{_GREEN};border:1px solid {_GREEN};
            }}
        """)
        demo_row.addWidget(demo_lbl)
        demo_row.addWidget(self._att_demo_btn)
        col_a.addLayout(demo_row)
        col_a.addWidget(_divider())

        # Utilities -- also re-zeros the YAW readout on the 3D view, not just
        # the barometer, so the name can't just say "Baro" anymore.
        col_a.addWidget(_section('UTILITIES'))
        calibrate_btn = _flat_btn('Calibrate', height=26)
        calibrate_btn.clicked.connect(self._on_calibrate)
        col_a.addWidget(calibrate_btn)
        col_a.addWidget(_divider())

        # SD card -- the FC auto-starts logging at boot and auto-stops on
        # LANDED, so this is a manual override, not something you need to
        # remember to press before flight. Useful for starting a fresh log or
        # a card swapped in on the bench. Live PRESENT/RECORDING status
        # itself lives in the sensor panel's STATUS section, not here.
        col_a.addWidget(_section('SD CARD  (FLIGHT LOG)'))
        self._sd_btn = QPushButton('Start Log')
        self._sd_btn.setFixedHeight(28)
        self._sd_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._sd_btn.setCheckable(True)
        self._sd_btn.clicked.connect(self._on_sd_toggled)
        self._sd_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_TEXT};
                border:1px solid {_BORDER};border-radius:6px;font-weight:700;
            }}
            QPushButton:hover {{ background-color:#222436; }}
            QPushButton:checked {{
                background-color:#200A0A;color:{_RED};border:1px solid {_RED};
            }}
        """)
        col_a.addWidget(self._sd_btn)
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

        fin_note = QLabel('Test = sweep.  -/+ = nudge trim against your jig.')
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
            minus_btn = _flat_btn('-', height=23, width=28, font_size=13)
            plus_btn  = _flat_btn('+', height=23, width=28, font_size=13)
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

        if data.sd_present != self._sd_present or data.sd_recording != self._sd_recording:
            self._sd_present   = data.sd_present
            self._sd_recording = data.sd_recording
            self._update_button_states()

        if (data.attitude_control_on != self._attitude_control_on or
                data.attitude_demo_on != self._attitude_demo_on):
            self._attitude_control_on = data.attitude_control_on
            self._attitude_demo_on    = data.attitude_demo_on
            self._update_button_states()

    def _update_button_states(self) -> None:
        can_fire = self._state in {1, 2, 3, 4, 5}
        for btn in self._fire_btns:
            btn.setEnabled(can_fire)

        # Not gated on self._sd_present: the FC no longer polls for a card in
        # the background (that was a plausible cause of telemetry stalls --
        # see DataLogger::cardPresent() on the firmware side), so pressing
        # this while unchecked is also the retry button -- e.g. after
        # inserting a card that wasn't there at boot. It just re-attempts
        # and reports the result.
        self._sd_btn.setChecked(self._sd_recording)
        self._sd_btn.setText('Recording -- Tap to Stop' if self._sd_recording else 'Start Log')

        # Toggle buttons stay clickable in both directions -- setChecked()
        # here is a programmatic sync from telemetry, not a click, so it
        # does not re-trigger the toggled handlers/confirmation dialogs.
        self._att_ctrl_btn.setChecked(self._attitude_control_on)
        self._att_ctrl_btn.setText('ON' if self._attitude_control_on else 'OFF')
        self._att_demo_btn.setChecked(self._attitude_demo_on)
        self._att_demo_btn.setText('ON' if self._attitude_demo_on else 'OFF')

    def _on_sd_toggled(self, checked: bool) -> None:
        if checked:
            self.sd_start_requested.emit()
            return
        reply = QMessageBox.question(
            self, 'Confirm Stop Log',
            'Stop the flight computer\'s onboard SD recording?\n\n'
            'The FC auto-stops on LANDED already -- only do this manually '
            'if you specifically want to end the current log early.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.sd_stop_requested.emit()
        else:
            self._sd_btn.setChecked(True)   # revert -- still recording, nothing changed

    def _on_attitude_control_toggled(self, checked: bool) -> None:
        if not checked:
            self.attitude_control_disable_requested.emit()
            return
        reply = QMessageBox.warning(
            self, 'Confirm Enable Real Attitude Control',
            'Enable REAL in-flight fin control?\n\n'
            'This engages active fin correction during POWERED_ASCENT/COAST on the '
            'next flight. Only do this once the gyro-axis mapping and gains have '
            'been bench-validated (see config.h / ATTITUDE_DEMO for that check).',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.attitude_control_enable_requested.emit()
        else:
            self._att_ctrl_btn.setChecked(False)   # revert -- nothing was actually sent

    def _on_attitude_demo_toggled(self, checked: bool) -> None:
        if checked:
            self.attitude_demo_enable_requested.emit()
        else:
            self.attitude_demo_disable_requested.emit()

    def _on_calibrate(self) -> None:
        reply = QMessageBox.question(
            self, 'Confirm Calibrate',
            'Re-zero the barometer at the current altitude, and the YAW readout '
            'to the current heading?\n\nDo this only when the rocket is on the ground.',
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
