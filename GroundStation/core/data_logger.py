"""
CSV data logger. Opens a timestamped file on connect, appends one row per
valid telemetry packet, and flushes regularly so data survives a crash.
"""

import csv
import os
from datetime import datetime
from typing import Optional

from core.packet_decoder import TelemetryData

_CSV_FIELDS = [
    'rx_time', 'seq', 'timestamp_ms', 'state',
    'lat', 'lon', 'gps_alt_m', 'gps_sats', 'gps_fix',
    'baro_alt_m', 'vert_vel_ms',
    'accel_x_g', 'accel_y_g', 'accel_z_g', 'accel_mag_g',
    'quat_w', 'quat_x', 'quat_y', 'quat_z',
    'voltage_v', 'current_ma', 'rssi',
]


class DataLogger:
    def __init__(self, log_dir: str = '.'):
        self._log_dir   = log_dir
        self._file      = None
        self._writer    = None
        self._path: Optional[str] = None
        self._row_count = 0

    @property
    def log_path(self) -> Optional[str]:
        return self._path

    def open(self) -> str:
        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._path = os.path.join(self._log_dir, f'log_{ts}.csv')
        self._file   = open(self._path, 'w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._file, fieldnames=_CSV_FIELDS)
        self._writer.writeheader()
        self._file.flush()
        self._row_count = 0
        return self._path

    def log(self, data: TelemetryData) -> None:
        if self._writer is None:
            return
        self._writer.writerow({
            'rx_time':      f'{data.rx_time:.3f}',
            'seq':          data.seq,
            'timestamp_ms': data.timestamp_ms,
            'state':        data.state,
            'lat':          f'{data.lat:.8f}',
            'lon':          f'{data.lon:.8f}',
            'gps_alt_m':    f'{data.gps_alt_m:.2f}',
            'gps_sats':     data.gps_sats,
            'gps_fix':      data.gps_fix,
            'baro_alt_m':   f'{data.baro_alt_m:.2f}',
            'vert_vel_ms':  f'{data.vert_vel_ms:.3f}',
            'accel_x_g':    f'{data.accel_x_g:.4f}',
            'accel_y_g':    f'{data.accel_y_g:.4f}',
            'accel_z_g':    f'{data.accel_z_g:.4f}',
            'accel_mag_g':  f'{data.accel_mag_g:.4f}',
            'quat_w':       f'{data.quat_w:.6f}',
            'quat_x':       f'{data.quat_x:.6f}',
            'quat_y':       f'{data.quat_y:.6f}',
            'quat_z':       f'{data.quat_z:.6f}',
            'voltage_v':    f'{data.voltage_v:.3f}',
            'current_ma':   f'{data.current_ma:.1f}',
            'rssi':         data.rssi,
        })
        self._row_count += 1
        # Flush every 50 rows so we don't lose too much data on crash
        if self._row_count % 50 == 0:
            self._file.flush()

    def close(self) -> None:
        if self._file:
            self._file.flush()
            self._file.close()
        self._file   = None
        self._writer = None
