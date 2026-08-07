"""
Decodes binary TelemetryPacket from the flight computer.

Packet layout (packed, little-endian, 52 bytes total). Deliberately compact --
the LoRa link is stuck at a slow factory-default air data rate and packet size
is the only remaining lever to reduce airtime per packet, so most fields are
scaled fixed-point (int16) instead of float. Values are unscaled back to
normal engineering units in decode_packet() below; every other module only
ever sees the same TelemetryData fields/units as before this change.
  Offset  Size  Type      Field                 Scale
  0       1     uint8     magic[0]      = 0xAA
  1       1     uint8     magic[1]      = 0x55
  2       2     uint16    seq
  4       4     uint32    timestamp_ms
  8       1     uint8     state
  9       4     float     lat
  13      4     float     lon
  17      2     int16     gps_alt_dm            /10  -> meters
  19      1     uint8     gps_sats
  20      1     uint8     gps_fix
  21      2     int16     baro_alt_dm           /10  -> meters
  23      2     int16     vert_vel_cms          /100 -> m/s
  25      2     int16     accel_x_cg            /100 -> g
  27      2     int16     accel_y_cg            /100 -> g
  29      2     int16     accel_z_cg            /100 -> g
  31      2     int16     quat_w_i16            /32767
  33      2     int16     quat_x_i16            /32767
  35      2     int16     quat_y_i16            /32767
  37      2     int16     quat_z_i16            /32767
  39      2     int16     voltage_cv            /100 -> V
  41      2     int16     current_ma
  43      1     int8      rssi
  44      1     uint8     pyro_cont[0]  (CH1 ignition:  1=OK, 0=open)
  45      1     uint8     pyro_cont[1]  (CH2 parachute: 1=OK, 0=open)
  46      1     uint8     pyro_cont[2]  (CH3 backup:    1=OK, 0=open)
  47      1     uint8     cam_recording (1=recording, 0=stopped -- FC's belief, no camera ack)
  48      1     uint8     system_status (bit0 imu, bit1 baro, bit2 accel, bit3 gps, bit4 power ok --
                                          1=read successfully within the last 500ms, "alive right now";
                                          bit5 sd_present, bit6 sd_recording -- live, not boot-time)
  49      1     uint8     attitude_status (bit0 control_on, bit1 demo_on -- last ground-commanded
                                          attitude-control mode, not sensor health; see AttitudeController)
  50      2     uint16    checksum      (sum of bytes 0..49)
"""

import struct
import math
import time
from dataclasses import dataclass, field
from typing import Optional

TELEM_MAGIC_0 = 0xAA
TELEM_MAGIC_1 = 0x55
ACK_MAGIC_0   = 0xAC
ACK_MAGIC_1   = 0x4B

# Bits within system_status -- must mirror the #defines in Packet.h
SENSOR_HEALTH_IMU_OK   = 1 << 0
SENSOR_HEALTH_BARO_OK  = 1 << 1
SENSOR_HEALTH_ACCEL_OK = 1 << 2
SENSOR_HEALTH_GPS_OK   = 1 << 3
SENSOR_HEALTH_POWER_OK = 1 << 4
SD_STATUS_PRESENT      = 1 << 5
SD_STATUS_RECORDING    = 1 << 6

# Bits within attitude_status -- must mirror the #defines in Packet.h
ATTITUDE_STATUS_CONTROL_ON = 1 << 0
ATTITUDE_STATUS_DEMO_ON    = 1 << 1

# 29 fields, 52 bytes total
TELEM_FORMAT = '<BBHIBffhBBhhhhhhhhhhhbBBBBBBH'
TELEM_SIZE   = struct.calcsize(TELEM_FORMAT)  # 52

# AckPacket: magic0, magic1, cmdSeq, cmdType, checksum
ACK_FORMAT = '<BBBBH'
ACK_SIZE   = struct.calcsize(ACK_FORMAT)  # 6

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
    pyro_cont_0:  int          # CH1 continuity (1=OK, 0=open)
    pyro_cont_1:  int          # CH2 continuity
    pyro_cont_2:  int          # CH3 continuity
    cam_recording: int         # 1=recording, 0=stopped (FC's belief, no camera ack)
    system_status: int         # bitfield, see SENSOR_HEALTH_*_OK / SD_STATUS_* above
    attitude_status: int       # bitfield, see ATTITUDE_STATUS_* above
    checksum:     int
    # Derived — populated by decode_packet()
    accel_mag_g:  float = 0.0
    rx_time:      float = field(default_factory=time.time)

    @property
    def pyro_continuity(self) -> tuple[bool, bool, bool]:
        """True per channel if the ematch (igniter wire) is connected."""
        return (bool(self.pyro_cont_0), bool(self.pyro_cont_1), bool(self.pyro_cont_2))

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f'STATE_{self.state}')

    @property
    def state_color(self) -> str:
        return STATE_COLORS.get(self.state_name, '#555577')

    @property
    def is_recording(self) -> bool:
        return bool(self.cam_recording)

    @property
    def has_gps_fix(self) -> bool:
        # gps_fix comes from TinyGPSPlus location.isValid(); satellite count
        # arrives in a separate NMEA sentence so don't gate on it here.
        return bool(self.gps_fix)

    # Live sensor health -- "read successfully within the last 500ms", not a
    # one-time boot check. A False here mid-flight means that sensor just
    # dropped out, not that it never worked.
    @property
    def imu_ok(self) -> bool:
        return bool(self.system_status & SENSOR_HEALTH_IMU_OK)

    @property
    def baro_ok(self) -> bool:
        return bool(self.system_status & SENSOR_HEALTH_BARO_OK)

    @property
    def accel_ok(self) -> bool:
        return bool(self.system_status & SENSOR_HEALTH_ACCEL_OK)

    @property
    def gps_ok(self) -> bool:
        return bool(self.system_status & SENSOR_HEALTH_GPS_OK)

    @property
    def power_ok(self) -> bool:
        return bool(self.system_status & SENSOR_HEALTH_POWER_OK)

    # Flight computer's onboard SD card -- live, not a boot-time check. Not to
    # be confused with the ground station's own local CSV log (see the "REC"
    # button / core/data_logger.py), which is a separate, unrelated file.
    @property
    def sd_present(self) -> bool:
        return bool(self.system_status & SD_STATUS_PRESENT)

    @property
    def sd_recording(self) -> bool:
        return bool(self.system_status & SD_STATUS_RECORDING)

    # Last ground-commanded attitude-control mode -- see ATTITUDE_STATUS_*
    # above. Reflects what the FC was TOLD to do, not sensor/hardware health.
    @property
    def attitude_control_on(self) -> bool:
        return bool(self.attitude_status & ATTITUDE_STATUS_CONTROL_ON)

    @property
    def attitude_demo_on(self) -> bool:
        return bool(self.attitude_status & ATTITUDE_STATUS_DEMO_ON)


def decode_packet(raw: bytes) -> Optional[TelemetryData]:
    """
    Decode a raw telemetry packet.
    Returns None if magic bytes are wrong or checksum fails.
    """
    if len(raw) < TELEM_SIZE:
        return None

    raw = raw[:TELEM_SIZE]

    if raw[0] != TELEM_MAGIC_0 or raw[1] != TELEM_MAGIC_1:
        return None

    # Checksum = 16-bit sum of all bytes except the last 2 (the checksum itself)
    expected = sum(raw[:-2]) & 0xFFFF
    stored   = struct.unpack_from('<H', raw, TELEM_SIZE - 2)[0]
    if expected != stored:
        return None

    try:
        vals = struct.unpack(TELEM_FORMAT, raw)
    except struct.error:
        return None

    (magic0, magic1, seq, timestamp_ms, state,
     lat, lon, gps_alt_dm, gps_sats, gps_fix,
     baro_alt_dm, vert_vel_cms,
     accel_x_cg, accel_y_cg, accel_z_cg,
     quat_w_i16, quat_x_i16, quat_y_i16, quat_z_i16,
     voltage_cv, current_ma,
     rssi, pyro_cont_0, pyro_cont_1, pyro_cont_2, cam_recording,
     system_status, attitude_status, checksum) = vals

    # Unscale wire fixed-point values back to normal engineering units --
    # everything downstream of this function sees the same units as before.
    data = TelemetryData(
        magic0=magic0, magic1=magic1, seq=seq, timestamp_ms=timestamp_ms, state=state,
        lat=lat, lon=lon,
        gps_alt_m=gps_alt_dm / 10.0,
        gps_sats=gps_sats, gps_fix=gps_fix,
        baro_alt_m=baro_alt_dm / 10.0,
        vert_vel_ms=vert_vel_cms / 100.0,
        accel_x_g=accel_x_cg / 100.0,
        accel_y_g=accel_y_cg / 100.0,
        accel_z_g=accel_z_cg / 100.0,
        quat_w=quat_w_i16 / 32767.0,
        quat_x=quat_x_i16 / 32767.0,
        quat_y=quat_y_i16 / 32767.0,
        quat_z=quat_z_i16 / 32767.0,
        voltage_v=voltage_cv / 100.0,
        current_ma=float(current_ma),
        rssi=rssi,
        pyro_cont_0=pyro_cont_0, pyro_cont_1=pyro_cont_1, pyro_cont_2=pyro_cont_2,
        cam_recording=cam_recording,
        system_status=system_status,
        attitude_status=attitude_status,
        checksum=checksum,
    )
    data.accel_mag_g = math.sqrt(data.accel_x_g**2 + data.accel_y_g**2 + data.accel_z_g**2)
    data.rx_time     = time.time()
    return data


def find_packet_start(buf: bytes, magic0: int = TELEM_MAGIC_0, magic1: int = TELEM_MAGIC_1) -> int:
    """
    Scan buf for a given magic byte pair (defaults to the telemetry magic).
    Returns the index of the magic or -1 if not found.
    """
    for i in range(len(buf) - 1):
        if buf[i] == magic0 and buf[i + 1] == magic1:
            return i
    return -1


@dataclass
class AckData:
    cmd_seq:  int
    cmd_type: int


def decode_ack(raw: bytes) -> Optional[AckData]:
    """
    Decode a raw AckPacket (6 bytes: magic0, magic1, cmdSeq, cmdType, checksum).
    Returns None if magic bytes are wrong or checksum fails.
    """
    if len(raw) < ACK_SIZE:
        return None

    raw = raw[:ACK_SIZE]

    if raw[0] != ACK_MAGIC_0 or raw[1] != ACK_MAGIC_1:
        return None

    expected = sum(raw[:-2]) & 0xFFFF
    stored   = struct.unpack_from('<H', raw, ACK_SIZE - 2)[0]
    if expected != stored:
        return None

    try:
        _, _, cmd_seq, cmd_type, _ = struct.unpack(ACK_FORMAT, raw)
    except struct.error:
        return None

    return AckData(cmd_seq=cmd_seq, cmd_type=cmd_type)
