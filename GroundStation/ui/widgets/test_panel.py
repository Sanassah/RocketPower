"""TEMPORARY bench-test panel: axis-mapping/allocation-sign calibration data,
live, replacing the need to watch a plain-text serial monitor alongside
GroundStation. Reads the TEMPORARY bench fields TelemetryData carries (see
core/packet_decoder.py's docstring) -- delete this whole file (and its nav
entry/page in main_window.py) once config.h's ATTITUDE_*_RATE/ANGLE_ERR
mapping is confirmed and those fields come back out of the wire packet.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGridLayout, QFrame, QHBoxLayout
from PyQt6.QtCore import Qt

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_GREEN  = '#22C55E'
_RED    = '#EF4444'
_AMBER  = '#F59E0B'

_MONO = 'JetBrains Mono", "Cascadia Code", "Consolas'


def _section(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:0.9px;'
        f'padding-top:8px;border:none;background:transparent;'
    )
    return lbl


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet('background-color:rgba(255,255,255,18);margin:6px 0;')
    return f


class _Row:
    def __init__(self, grid: QGridLayout, row: int, label: str):
        lbl = QLabel(label)
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:14px;border:none;background:transparent;'
        )
        self._val = QLabel('--')
        self._val.setStyleSheet(
            f'color:{_TEXT};font-family:"{_MONO}",monospace;'
            f'font-size:16px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(lbl,       row, 0)
        grid.addWidget(self._val, row, 1)

    def set(self, text: str, warn: bool = False, ok: bool = False) -> None:
        c = _RED if warn else _GREEN if ok else _TEXT
        self._val.setStyleSheet(
            f'color:{c};font-family:"{_MONO}",monospace;'
            f'font-size:16px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setText(text)


class TestPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f'background-color:{_BG};border:none;border-radius:10px;')

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 18)
        root.setSpacing(2)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('<<')
        sym.setStyleSheet(f'color:{_TEXT};font-size:14px;background:transparent;border:none;')
        ttl = QLabel('AXIS CAL BENCH TEST  (temporary)')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        self._r_mode = QLabel('OFF')
        self._r_mode.setStyleSheet(
            f'color:{_MUTED};font-size:12px;font-weight:800;letter-spacing:0.5px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(self._r_mode)
        root.addLayout(hdr)
        root.addWidget(_divider())

        note = QLabel(
            'Tilt the airframe by hand toward one compass direction at a time and watch '
            'which numbers move. Enable ATTITUDE DEMO first (Telemetry page).'
        )
        note.setWordWrap(True)
        note.setStyleSheet(f'color:{_MUTED};font-size:12px;background:transparent;border:none;padding-bottom:6px;')
        root.addWidget(note)

        columns = QHBoxLayout()
        columns.setSpacing(32)
        col_a = QVBoxLayout(); col_a.setSpacing(2)
        col_b = QVBoxLayout(); col_b.setSpacing(2)
        columns.addLayout(col_a, 1)
        columns.addLayout(col_b, 1)
        root.addLayout(columns)

        # Column A: raw gyro rate + absolute tilt
        col_a.addWidget(_section('GYRO (rad/s, raw sensor axis)'))
        gyro_grid = QGridLayout()
        gyro_grid.setColumnStretch(1, 1)
        gyro_grid.setVerticalSpacing(4)
        self._r_gx = _Row(gyro_grid, 0, 'x')
        self._r_gy = _Row(gyro_grid, 1, 'y')
        self._r_gz = _Row(gyro_grid, 2, 'z')
        col_a.addLayout(gyro_grid)
        col_a.addWidget(_divider())

        # Real Euler extraction from the absolute quaternion (tilt from
        # level, via gravity) -- NOT AttitudeController's reference-latched
        # small-angle error, which can't be trusted for this (see the
        # exchange that led to this tab). Labeled by raw axis on purpose --
        # which axis is which compass direction is what this test finds out.
        col_a.addWidget(_section('TILT (deg, absolute from level)'))
        tilt_grid = QGridLayout()
        tilt_grid.setColumnStretch(1, 1)
        tilt_grid.setVerticalSpacing(4)
        self._r_tx   = _Row(tilt_grid, 0, 'X')
        self._r_ty   = _Row(tilt_grid, 1, 'Y')
        self._r_spin = _Row(tilt_grid, 2, 'spin (Z)')
        col_a.addLayout(tilt_grid)
        col_a.addStretch()

        # Column B: fin response
        col_b.addWidget(_section('FIN CORRECTION (deg, live commanded)'))
        fin_grid = QGridLayout()
        fin_grid.setColumnStretch(1, 1)
        fin_grid.setVerticalSpacing(4)
        self._r_fs = _Row(fin_grid, 0, 'S  (CH1)')
        self._r_fe = _Row(fin_grid, 1, 'E  (CH2)')
        self._r_fn = _Row(fin_grid, 2, 'N  (CH3)')
        self._r_fw = _Row(fin_grid, 3, 'W  (CH4)')
        col_b.addLayout(fin_grid)
        col_b.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        if data.attitude_control_on:
            self._r_mode.setText('REAL CONTROL')
            self._r_mode.setStyleSheet(
                f'color:{_RED};font-size:12px;font-weight:800;letter-spacing:0.5px;'
                f'background:transparent;border:none;'
            )
        elif data.attitude_demo_on:
            self._r_mode.setText('DEMO ACTIVE')
            self._r_mode.setStyleSheet(
                f'color:{_GREEN};font-size:12px;font-weight:800;letter-spacing:0.5px;'
                f'background:transparent;border:none;'
            )
        else:
            self._r_mode.setText('OFF -- enable ATTITUDE DEMO to see live data')
            self._r_mode.setStyleSheet(
                f'color:{_AMBER};font-size:12px;font-weight:800;letter-spacing:0.5px;'
                f'background:transparent;border:none;'
            )

        self._r_gx.set(f'{data.gyro_x:+.3f}')
        self._r_gy.set(f'{data.gyro_y:+.3f}')
        self._r_gz.set(f'{data.gyro_z:+.3f}')

        self._r_tx.set(f'{data.tilt_x_deg:+.1f}')
        self._r_ty.set(f'{data.tilt_y_deg:+.1f}')
        self._r_spin.set(f'{data.spin_z_deg:+.1f}')

        s, e, n, w = data.fin_deg
        self._r_fs.set(f'{s:+.1f}')
        self._r_fe.set(f'{e:+.1f}')
        self._r_fn.set(f'{n:+.1f}')
        self._r_fw.set(f'{w:+.1f}')
