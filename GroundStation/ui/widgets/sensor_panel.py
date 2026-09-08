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
        f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:0.9px;'
        f'padding-top:8px;border:none;background:transparent;'
    )
    return lbl


def _divider() -> QFrame:
    # Softer than a full _BORDER line -- section separation still helps
    # scanability in a data-dense panel like this, but doesn't need to be as
    # loud as an outer card outline.
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
            f'font-size:15px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(lbl,       row, 0)
        grid.addWidget(self._val, row, 1)

    def set(self, text: str, warn: bool = False, ok: bool = False) -> None:
        c = _RED if warn else _GREEN if ok else _TEXT
        self._val.setStyleSheet(
            f'color:{c};font-family:"{_MONO}",monospace;'
            f'font-size:15px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setText(text)


class SensorPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:none;border-radius:10px;'
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 18)
        root.setSpacing(2)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('<<')
        sym.setStyleSheet(f'color:{_TEXT};font-size:14px;background:transparent;border:none;')
        ttl = QLabel('TELEMETRY')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)
        root.addWidget(_divider())

        # Two side-by-side columns instead of one long stack -- this panel
        # now spans the full page width (see main_window.py), so splitting
        # it this way gives every row real horizontal room instead of a
        # cramped single column with everything squeezed to one side.
        columns = QHBoxLayout()
        columns.setSpacing(32)
        col_a = QVBoxLayout(); col_a.setSpacing(2)
        col_b = QVBoxLayout(); col_b.setSpacing(2)
        columns.addLayout(col_a, 1)
        columns.addLayout(col_b, 1)
        root.addLayout(columns)

        # Column A: STATUS + GPS
        # Live sensor health -- "responded within the last 500ms", not a
        # one-time boot check. The only place this ever surfaces once the
        # airframe is closed up and the status LEDs are out of sight.
        col_a.addWidget(_section('STATUS'))
        status_grid = QGridLayout()
        status_grid.setColumnStretch(1, 1)
        status_grid.setVerticalSpacing(4)
        self._r_imu_ok   = _Row(status_grid, 0, 'IMU')
        self._r_baro_ok  = _Row(status_grid, 1, 'Barometer')
        self._r_accel_ok = _Row(status_grid, 2, 'Accelerometer')
        self._r_gps_ok   = _Row(status_grid, 3, 'GPS Module')
        self._r_power_ok = _Row(status_grid, 4, 'Power Monitor')
        self._r_sd       = _Row(status_grid, 5, 'SD Card')
        self._r_attitude = _Row(status_grid, 6, 'Attitude Control')
        col_a.addLayout(status_grid)
        col_a.addWidget(_divider())

        col_a.addWidget(_section('GPS'))
        gps_grid = QGridLayout()
        gps_grid.setColumnStretch(1, 1)
        gps_grid.setVerticalSpacing(4)
        self._r_lat  = _Row(gps_grid, 0, 'Latitude')
        self._r_lon  = _Row(gps_grid, 1, 'Longitude')
        self._r_galt = _Row(gps_grid, 2, 'GPS Alt (m)')
        self._r_sats = _Row(gps_grid, 3, 'Satellites')
        self._r_fix  = _Row(gps_grid, 4, 'Fix')
        col_a.addLayout(gps_grid)
        col_a.addStretch()

        # Column B: BARO / IMU + POWER
        col_b.addWidget(_section('BARO / IMU'))
        baro_grid = QGridLayout()
        baro_grid.setColumnStretch(1, 1)
        baro_grid.setVerticalSpacing(4)
        self._r_balt = _Row(baro_grid, 0, 'Baro Alt (m)')
        self._r_vvel = _Row(baro_grid, 1, 'Vert Vel (m/s)')
        # TEMPORARY: sourced from the BNO085 (gravity-removed, ~0g at rest),
        # not the ADXL375 -- see FlightComputer's Packet.h.
        self._r_ax   = _Row(baro_grid, 2, 'Lin. Accel X (g)')
        self._r_ay   = _Row(baro_grid, 3, 'Lin. Accel Y (g)')
        self._r_az   = _Row(baro_grid, 4, 'Lin. Accel Z (g)')
        self._r_amag = _Row(baro_grid, 5, 'Lin. |accel| (g)')
        col_b.addLayout(baro_grid)
        col_b.addWidget(_divider())

        col_b.addWidget(_section('POWER'))
        pwr_grid = QGridLayout()
        pwr_grid.setColumnStretch(1, 1)
        pwr_grid.setVerticalSpacing(4)
        self._r_volt = _Row(pwr_grid, 0, 'Voltage (V)')
        self._r_curr = _Row(pwr_grid, 1, 'Current (mA)')
        col_b.addLayout(pwr_grid)
        col_b.addStretch()

    def update_data(self, data: TelemetryData) -> None:
        self._r_imu_ok.set('OK' if data.imu_ok else 'LOST', warn=not data.imu_ok, ok=data.imu_ok)
        self._r_baro_ok.set('OK' if data.baro_ok else 'LOST', warn=not data.baro_ok, ok=data.baro_ok)
        self._r_accel_ok.set('OK' if data.accel_ok else 'LOST', warn=not data.accel_ok, ok=data.accel_ok)
        self._r_gps_ok.set('OK' if data.gps_ok else 'LOST', warn=not data.gps_ok, ok=data.gps_ok)
        self._r_power_ok.set('OK' if data.power_ok else 'LOST', warn=not data.power_ok, ok=data.power_ok)

        # Flight computer's onboard SD card (not the ground station's own
        # local CSV log -- that's the separate "REC" button up top). Unlike
        # the sensor rows above, this isn't continuously re-checked while
        # idle -- it reflects the FC's last actual attempt (boot, a Start
        # Log command, or a write during active recording), not a live
        # poll. "NO CARD" can mean "definitely no card" or "haven't
        # rechecked since you inserted one" -- press Start Log to retry.
        if data.sd_recording:
            self._r_sd.set('RECORDING', ok=True)
        elif data.sd_present:
            self._r_sd.set('IDLE')
        else:
            self._r_sd.set('NO CARD', warn=True)

        # Last ground-commanded attitude-control mode -- see command panel's
        # ATTITUDE CONTROL section for the actual toggles. This is read-only,
        # reflecting what the FC confirms it's doing, not a boot-time check.
        if data.attitude_control_on:
            self._r_attitude.set('REAL CONTROL', warn=True)
        elif data.attitude_demo_on:
            self._r_attitude.set('DEMO', ok=True)
        else:
            self._r_attitude.set('OFF')

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
