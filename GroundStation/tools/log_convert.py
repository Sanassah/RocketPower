"""
Converts a flight computer .BIN log (FLTxxxxx.BIN, pulled off the SD card)
into .csv and/or .mat for analysis. The flight computer logs in a raw binary
format on purpose -- see DataLogger.cpp -- because formatting ~30 fields to
ASCII 100 times a second is real CPU/SD-write cost on an embedded MCU that's
already budgeted tight. This script does that conversion after the fact, on
a PC, where none of that matters.

File layout (see DataLogger.cpp):
  - An ASCII header block (field names, informational -- see below for why
    it's not enough on its own) ending in an 0x1E "record separator" byte.
  - Fixed-size 128-byte binary records after that, one per FlightData struct.

FlightData (SensorManager.h) is a plain, NOT #pragma-packed struct, so the
compiler inserts alignment padding between some fields. The byte offsets
below were extracted directly from the actual ARM/GCC build (verified via
offsetof()/sizeof() against the real compiler, not assumed) -- if
FlightData's fields are ever added/removed/reordered, this format string
must be regenerated the same way, or every field after the change will
silently decode as garbage.

Usage:
    python tools/log_convert.py FLT00001.BIN
    python tools/log_convert.py FLT00001.BIN --csv out.csv --mat out.mat
    python tools/log_convert.py FLT00001.BIN --no-mat   # csv only
"""

import argparse
import csv
import os
import struct
import sys

# '<' = little-endian, no implicit alignment (we specify every pad byte
# ourselves via 'x', matching the real compiler's layout exactly).
# 4x before the doubles: adding accel_mag_ms2 (2026-09) bumped the offset
# right before lat/lon from 80 to 84, which isn't 8-byte-aligned any more --
# the compiler inserts 4 bytes of padding there to align the doubles, on top
# of the 4 bytes accel_mag_ms2 itself adds. Re-verified via sizeof() against
# the real ARM/GCC build (136), not assumed -- see FlightComputer's
# SensorManager.h if this needs re-deriving again.
RECORD_FORMAT = '<IB3x19f4xddfBB2x3f5B7x'
RECORD_SIZE   = struct.calcsize(RECORD_FORMAT)  # 136, must equal sizeof(FlightData)

FIELDS = [
    'timestamp_ms', 'state',
    'quat_w', 'quat_x', 'quat_y', 'quat_z',
    'lin_accel_x', 'lin_accel_y', 'lin_accel_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'accel_mag_ms2',
    'pressure_hpa', 'temperature_c', 'baro_alt_m', 'vert_vel_ms',
    'highg_x_g', 'highg_y_g', 'highg_z_g', 'highg_mag_g',
    'lat', 'lon', 'gps_alt_m', 'gps_sats', 'gps_fix',
    'voltage_v', 'current_ma', 'power_mw',
    'imu_ok', 'baro_ok', 'accel_ok', 'gps_ok', 'power_ok',
]

BINARY_SENTINEL = 0x1E  # ASCII "record separator" -- marks end of the ASCII header

STATE_NAMES = {
    0: 'IDLE', 1: 'ARMED', 2: 'POWERED_ASCENT', 3: 'COAST',
    4: 'APOGEE', 5: 'DESCENT', 6: 'LANDED',
}


def parse_log(path: str) -> list[dict]:
    with open(path, 'rb') as f:
        raw = f.read()

    sentinel_idx = raw.find(bytes([BINARY_SENTINEL]))
    if sentinel_idx == -1:
        raise ValueError(
            f'No binary-section sentinel (0x1E) found in {path} -- '
            'this may not be a valid RocketFlightComputer log.'
        )

    header_text = raw[:sentinel_idx].decode('ascii', errors='replace')
    if '# END_HEADER' not in header_text:
        print('WARNING: header text found but missing "# END_HEADER" marker -- '
              'format may have changed since this script was written.', file=sys.stderr)

    body = raw[sentinel_idx + 1:]
    n_records, leftover = divmod(len(body), RECORD_SIZE)
    if leftover:
        print(f'WARNING: {leftover} trailing bytes are a partial record (probably '
              f'power loss mid-write) -- dropped, not included in output.', file=sys.stderr)

    records = []
    for i in range(n_records):
        chunk = body[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]
        vals = struct.unpack(RECORD_FORMAT, chunk)
        records.append(dict(zip(FIELDS, vals)))

    return records


def write_csv(records: list[dict], path: str) -> None:
    fieldnames = ['state_name'] + FIELDS
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for rec in records:
            row = dict(rec)
            row['state_name'] = STATE_NAMES.get(rec['state'], f'STATE_{rec["state"]}')
            writer.writerow(row)


def write_mat(records: list[dict], path: str) -> None:
    try:
        import numpy as np
        from scipy.io import savemat
    except ImportError:
        print('WARNING: scipy is not installed ("pip install scipy") -- skipped .mat export. '
              'CSV output (if requested) is unaffected.', file=sys.stderr)
        return

    columns = {name: np.array([rec[name] for rec in records]) for name in FIELDS}
    columns['state_name'] = np.array(
        [STATE_NAMES.get(rec['state'], f'STATE_{rec["state"]}') for rec in records],
        dtype=object,
    )
    # Nested under 'flight' so MATLAB sees one struct (flight.baro_alt_m, etc.)
    # instead of dozens of loose top-level variables.
    savemat(path, {'flight': columns})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='Path to a FLTxxxxx.BIN log file')
    ap.add_argument('--csv', help='Output CSV path (default: <input>.csv)')
    ap.add_argument('--mat', help='Output .mat path (default: <input>.mat)')
    ap.add_argument('--no-csv', action='store_true', help='Skip CSV output')
    ap.add_argument('--no-mat', action='store_true', help='Skip .mat output')
    args = ap.parse_args()

    records = parse_log(args.input)
    if not records:
        print('No complete records found -- nothing to write.', file=sys.stderr)
        sys.exit(1)

    base, _ = os.path.splitext(args.input)
    t0, t1 = records[0]['timestamp_ms'], records[-1]['timestamp_ms']
    print(f'Parsed {len(records)} records ({(t1 - t0) / 1000.0:.1f}s of flight time).')

    if not args.no_csv:
        csv_path = args.csv or base + '.csv'
        write_csv(records, csv_path)
        print(f'Wrote {csv_path}')

    if not args.no_mat:
        mat_path = args.mat or base + '.mat'
        write_mat(records, mat_path)
        if os.path.exists(mat_path):
            print(f'Wrote {mat_path}')


if __name__ == '__main__':
    main()
