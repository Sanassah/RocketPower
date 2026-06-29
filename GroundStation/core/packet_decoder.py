"""
Decodes binary TelemetryPacket from the flight computer.

Packet layout (packed, little-endian, 78 bytes total):
  Offset  Size  Type      Field
  0       1     uint8     magic[0]      = 0xAA
  1       1     uint8     magic[1]      = 0x55
  2       2     uint16    seq
  4       4     uint32    timestamp_ms
  8       1     uint8     state
  9       8     double    lat
  17      8     double    lon
  25      4     float     gps_alt_m
  29      1     uint8     gps_sats
  30      1     uint8     gps_fix
  31      4     float     baro_alt_m
  35      4     float     vert_vel_ms
  39      4     float     accel_x_g
  43      4     float     accel_y_g
  47      4     float     accel_z_g
  51      4     float     quat_w
  55      4     float     quat_x
  59      4     float     quat_y
  63      4     float     quat_z
  67      4     float     voltage_v
  71      4     float     current_ma
  75      1     int8      rssi
  76      2     uint16    checksum     (sum of bytes 0..75)
"""

import struct
import math
import time
from dataclasses import dataclass, field
from typing import Optional

TELEM_MAGIC_0 = 0xAA
TELEM_MAGIC_1 = 0x55

# 23 fields, 78 bytes total
TELEM_FORMAT = '<BBHIBddfBBfffffffffffbH'
TELEM_SIZE   = struct.calcsize(TELEM_FORMAT)  # 78

STATE_NAMES = {
    0: 'IDLE',
    1: 'ARMED',
    2: 'POWERED_ASCENT',
    3: 'COAST',
    4: 'APOGEE',
    5: 'DESCENT',
    6: 'LANDED',
}

STATE_COLORS = {
    'IDLE':           '#555577',
    'ARMED':          '#cc8800',
    'POWERED_ASCENT': '#ff5500',
    'COAST':          '#2255dd',
    'APOGEE':         '#8800cc',
    'DESCENT':        '#009999',
    'LANDED':         '#00cc66',
}


@dataclass
class TelemetryData:
    magic0:       int
    magic1:       int
    seq:          int
    timestamp_ms: int
    state:        int
    lat:          float
    lon:          float
    gps_alt_m:    float
    gps_sats:     int
    gps_fix:      int
    baro_alt_m:   float
    vert_vel_ms:  float
    accel_x_g:    float
    accel_y_g:    float
    accel_z_g:    float
    quat_w:       float
    quat_x:       float
    quat_y:       float
    quat_z:       float
    voltage_v:    float
    current_ma:   float
    rssi:         int
    checksum:     int
    # Derived — populated by decode_packet()
    accel_mag_g:  float = 0.0
    rx_time:      float = field(default_factory=time.time)

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f'STATE_{self.state}')

    @property
    def state_color(self) -> str:
        return STATE_COLORS.get(self.state_name, '#555577')

    @property
    def has_gps_fix(self) -> bool:
        return bool(self.gps_fix) and self.gps_sats >= 3


def decode_packet(raw: bytes) -> Optional[TelemetryData]:
    """
    Decode a raw 78-byte telemetry packet.
    Returns None if magic bytes are wrong or checksum fails.
    """
    if len(raw) < TELEM_SIZE:
        return None

    raw = raw[:TELEM_SIZE]

    if raw[0] != TELEM_MAGIC_0 or raw[1] != TELEM_MAGIC_1:
        return None

    # Checksum = 16-bit sum of bytes 0..75
    expected = sum(raw[:-2]) & 0xFFFF
    stored   = struct.unpack_from('<H', raw, TELEM_SIZE - 2)[0]
    if expected != stored:
        return None

    try:
        vals = struct.unpack(TELEM_FORMAT, raw)
    except struct.error:
        return None

    data = TelemetryData(*vals)
    data.accel_mag_g = math.sqrt(data.accel_x_g**2 + data.accel_y_g**2 + data.accel_z_g**2)
    data.rx_time     = time.time()
    return data


def find_packet_start(buf: bytes) -> int:
    """
    Scan buf for the telemetry magic bytes 0xAA 0x55.
    Returns the index of the magic or -1 if not found.
    """
    for i in range(len(buf) - 1):
        if buf[i] == TELEM_MAGIC_0 and buf[i + 1] == TELEM_MAGIC_1:
            return i
    return -1
