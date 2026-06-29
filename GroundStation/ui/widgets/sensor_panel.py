"""Live sensor readouts: GPS info, baro/IMU, power."""

from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QLabel, QGridLayout, QFrame, QWidget, QHBoxLayout
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui  import QFont

from core.packet_decoder import TelemetryData

_MONO = 'JetBrains Mono", "Cascadia Code", "Consolas'

_LABEL_STYLE = 'color: #6E6E73; font-size: 11px;'
_VALUE_STYLE = f'color: #1D1D1F; font-family: "{_MONO}", monospace; font-size: 13px; font-weight: 600;'
_WARN_STYLE  = f'color: #FF3B30; font-family: "{_MONO}", monospace; font-size: 13px; font-weight: 700;'
_OK_STYLE    = f'color: #34C759; font-family: "{_MONO}", monospace; font-size: 13px; font-weight: 600;'

LOW_VOLTAGE_V = 7.0


def _section_header(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        'color: #8E8E93; font-size: 10px; font-weight: 700;'
        'letter-spacing: 0.6px; padding-top: 4px;'
    )
    return lbl


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet('background-color: #F2F2F7; max-height: 1px; margin: 4px 0;')
    return f


class _Row:
    """Label + monospace value pair in a QGridLayout."""
    def __init__(self, grid: QGridLayout, row: int, label: str):
        lbl = QLabel(label)
        lbl.setStyleSheet(_LABEL_STYLE)
        self._val = QLabel('—')
        self._val.setStyleSheet(_VALUE_STYLE)
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(lbl,       row, 0)
        grid.addWidget(self._val, row, 1)

    def set(self, text: str, warn: bool = False, ok: bool = False) -> None:
        self._val.setStyleSheet(_WARN_STYLE if warn else _OK_STYLE if ok else _VALUE_STYLE)
        self._val.setText(text)


class SensorPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('SENSORS', parent)
        layout = QVBoxLayout(self)
        layout.setSpacing(2)
        layout.setContentsMargins(12, 8, 12, 12)

        # ── GPS ───────────────────────────────────────────────────
        layout.addWidget(_section_header('GPS'))
        gps_grid = QGridLayout()
        gps_grid.setColumnStretch(1, 1)
        gps_grid.setVerticalSpacing(3)
        self._r_lat  = _Row(gps_grid, 0, 'Latitude')
        self._r_lon  = _Row(gps_grid, 1, 'Longitude')
        self._r_galt = _Row(gps_grid, 2, 'GPS Alt (m)')
        self._r_sats = _Row(gps_grid, 3, 'Satellites')
        self._r_fix  = _Row(gps_grid, 4, 'Fix')
        layout.addLayout(gps_grid)
        layout.addWidget(_divider())

        # ── Baro / IMU ────────────────────────────────────────────
        layout.addWidget(_section_header('BARO / IMU'))
        baro_grid = QGridLayout()
        baro_grid.setColumnStretch(1, 1)
        baro_grid.setVerticalSpacing(3)
        self._r_balt  = _Row(baro_grid, 0, 'Baro Alt (m)')
        self._r_vvel  = _Row(baro_grid, 1, 'Vert Vel (m/s)')
        self._r_ax    = _Row(baro_grid, 2, 'Accel X (g)')
        self._r_ay    = _Row(baro_grid, 3, 'Accel Y (g)')
        self._r_az    = _Row(baro_grid, 4, 'Accel Z (g)')
        self._r_amag  = _Row(baro_grid, 5, 'Lin. |accel| (g)')
        layout.addLayout(baro_grid)
        layout.addWidget(_divider())

        # ── Power ─────────────────────────────────────────────────
        layout.addWidget(_section_header('POWER'))
        pwr_grid = QGridLayout()
        pwr_grid.setColumnStretch(1, 1)
        pwr_grid.setVerticalSpacing(3)
        self._r_volt = _Row(pwr_grid, 0, 'Voltage (V)')
        self._r_curr = _Row(pwr_grid, 1, 'Current (mA)')
        self._r_rssi = _Row(pwr_grid, 2, 'RSSI (dBm)')
        layout.addLayout(pwr_grid)

        layout.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        has_fix = data.has_gps_fix
        self._r_lat.set(f'{data.lat:.6f}°' if has_fix else '—')
        self._r_lon.set(f'{data.lon:.6f}°' if has_fix else '—')
        self._r_galt.set(f'{data.gps_alt_m:.1f}' if has_fix else '—')
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
