"""Live sensor readouts -- dark dashboard style."""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGridLayout, QFrame, QHBoxLayout
from PyQt6.QtCore import Qt
from PyQt6.QtGui  import QFont

from core.packet_decoder import TelemetryData

_BG     = '#1A1A1B'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_GREEN  = '#22C55E'
_RED    = '#EF4444'

_MONO = 'JetBrains Mono", "Cascadia Code", "Consolas'

LOW_VOLTAGE_V = 7.0


def _section(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:0.9px;'
        f'padding-top:8px;border:none;background:transparent;'
    )
    return lbl


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet(f'background-color:{_BORDER};margin:3px 0;')
    return f


class _Row:
    def __init__(self, grid: QGridLayout, row: int, label: str):
        lbl = QLabel(label)
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:12px;border:none;background:transparent;'
        )
        self._val = QLabel('--')
        self._val.setStyleSheet(
            f'color:{_TEXT};font-family:"{_MONO}",monospace;'
            f'font-size:13px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(lbl,       row, 0)
        grid.addWidget(self._val, row, 1)

    def set(self, text: str, warn: bool = False, ok: bool = False) -> None:
        c = _RED if warn else _GREEN if ok else _TEXT
        self._val.setStyleSheet(
            f'color:{c};font-family:"{_MONO}",monospace;'
            f'font-size:13px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setText(text)


class SensorPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 16)
        root.setSpacing(2)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('<<')
        sym.setStyleSheet(f'color:{_TEXT};font-size:12px;background:transparent;border:none;')
        ttl = QLabel('TELEMETRY')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)
        root.addWidget(_divider())

        root.addWidget(_section('GPS'))
        gps_grid = QGridLayout()
        gps_grid.setColumnStretch(1, 1)
        gps_grid.setVerticalSpacing(4)
        self._r_lat  = _Row(gps_grid, 0, 'Latitude')
        self._r_lon  = _Row(gps_grid, 1, 'Longitude')
        self._r_galt = _Row(gps_grid, 2, 'GPS Alt (m)')
        self._r_sats = _Row(gps_grid, 3, 'Satellites')
        self._r_fix  = _Row(gps_grid, 4, 'Fix')
        root.addLayout(gps_grid)
        root.addWidget(_divider())

        root.addWidget(_section('BARO / IMU'))
        baro_grid = QGridLayout()
        baro_grid.setColumnStretch(1, 1)
        baro_grid.setVerticalSpacing(4)
        self._r_balt = _Row(baro_grid, 0, 'Baro Alt (m)')
        self._r_vvel = _Row(baro_grid, 1, 'Vert Vel (m/s)')
        self._r_ax   = _Row(baro_grid, 2, 'Accel X (g)')
        self._r_ay   = _Row(baro_grid, 3, 'Accel Y (g)')
        self._r_az   = _Row(baro_grid, 4, 'Accel Z (g)')
        self._r_amag = _Row(baro_grid, 5, 'Lin. |accel| (g)')
        root.addLayout(baro_grid)
        root.addWidget(_divider())

        root.addWidget(_section('POWER'))
        pwr_grid = QGridLayout()
        pwr_grid.setColumnStretch(1, 1)
        pwr_grid.setVerticalSpacing(4)
        self._r_volt = _Row(pwr_grid, 0, 'Voltage (V)')
        self._r_curr = _Row(pwr_grid, 1, 'Current (mA)')
        self._r_rssi = _Row(pwr_grid, 2, 'RSSI (dBm)')
        root.addLayout(pwr_grid)
        root.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        has_fix = data.has_gps_fix
        self._r_lat.set(f'{data.lat:.6f} deg' if has_fix else '--')
        self._r_lon.set(f'{data.lon:.6f} deg' if has_fix else '--')
        self._r_galt.set(f'{data.gps_alt_m:.1f}' if has_fix else '--')
        self._r_sats.set(str(data.gps_sats), warn=data.gps_sats < 3)
        self._r_fix.set('FIX' if has_fix else 'NO FIX', warn=not has_fix, ok=has_fix)

        self._r_balt.set(f'{data.baro_alt_m:.1f}')
        self._r_vvel.set(f'{data.vert_vel_ms:+.2f}')
        self._r_ax.set(f'{data.accel_x_g:+.3f}')
        self._r_ay.set(f'{data.accel_y_g:+.3f}')
        self._r_az.set(f'{data.accel_z_g:+.3f}')
        self._r_amag.set(f'{data.accel_mag_g:.3f}')

        low_v = data.voltage_v < LOW_VOLTAGE_V
        self._r_volt.set(f'{data.voltage_v:.2f}', warn=low_v)
        self._r_curr.set(f'{data.current_ma:.0f}')
        self._r_rssi.set(f'{data.rssi}')
