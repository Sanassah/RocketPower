"""
Encodes CommandPackets to send to the flight computer.

CommandPacket layout (packed, little-endian, 7 bytes):
  0  uint8   magic[0]  = 0xBB
  1  uint8   magic[1]  = 0x44
  2  uint8   type      (CommandType enum)
  3  uint8   param     (channel for FIRE_PYRO, else 0)
  4  uint8   seq       (ground-assigned, echoed back in AckPacket)
  5  uint16  checksum  (sum of bytes 0..4)
"""

import struct
from enum import IntEnum

CMD_MAGIC_0 = 0xBB
CMD_MAGIC_1 = 0x44

CMD_FORMAT = '<BBBBBH'
CMD_SIZE   = struct.calcsize(CMD_FORMAT)  # 7


class CommandType(IntEnum):
    ARM            = 0x01
    DISARM         = 0x02
    FIRE_PYRO      = 0x03
    PING           = 0x04
    CALIBRATE_BARO = 0x05   # re-zero barometer at current ground level
    SERVO_TEST      = 0x06  # param = fin channel (1-4); sweeps center->min->max->center
    CAM_TOGGLE       = 0x07  # toggles camera recording (the camera only supports a toggle)
    SERVO_NUDGE_POS  = 0x09  # param = fin channel (1-4); +trim step, not persisted
    SERVO_NUDGE_NEG  = 0x0A  # param = fin channel (1-4); -trim step, not persisted
    SERVO_SAVE_CAL   = 0x0B  # param unused; persists all 4 channels' live position as new trim
    SERVO_CENTER_ALL = 0x0C  # param unused; drives all 4 to raw center, ignoring trim, not persisted
    SERVO_PREFLIGHT  = 0x0D  # param unused; blocking ~6s all-4 choreography
    SD_START_RECORDING = 0x0E  # param unused; opens a new log file (no-op if already recording)
    SD_STOP_RECORDING  = 0x0F  # param unused; flushes and closes the current log file
    ATTITUDE_CONTROL_ENABLE  = 0x10  # param unused; real in-flight fin control, engages POWERED_ASCENT/COAST
    ATTITUDE_CONTROL_DISABLE = 0x11  # param unused
    ATTITUDE_DEMO_ENABLE     = 0x12  # param unused; ground-demo fin control, engages in IDLE or ARMED
    ATTITUDE_DEMO_DISABLE    = 0x13  # param unused
    RESET = 0x14  # param unused; soft reset -- fresh-boot-equivalent IDLE without a power cycle


def encode_command(cmd_type: CommandType, param: int = 0, seq: int = 0) -> bytes:
    """Build a validated CommandPacket as bytes ready to send over serial."""
    payload  = struct.pack('<BBBBB', CMD_MAGIC_0, CMD_MAGIC_1, int(cmd_type), param & 0xFF, seq & 0xFF)
    checksum = sum(payload) & 0xFFFF
    return payload + struct.pack('<H', checksum)


def encode_arm(seq: int = 0)                  -> bytes: return encode_command(CommandType.ARM, 0, seq)
def encode_disarm(seq: int = 0)               -> bytes: return encode_command(CommandType.DISARM, 0, seq)
def encode_fire_pyro(ch: int, seq: int = 0)   -> bytes: return encode_command(CommandType.FIRE_PYRO, ch, seq)
def encode_calibrate(seq: int = 0)            -> bytes: return encode_command(CommandType.CALIBRATE_BARO, 0, seq)
def encode_servo_test(ch: int, seq: int = 0)  -> bytes: return encode_command(CommandType.SERVO_TEST, ch, seq)
def encode_cam_toggle(seq: int = 0)           -> bytes: return encode_command(CommandType.CAM_TOGGLE, 0, seq)

def encode_servo_nudge(ch: int, positive: bool, seq: int = 0) -> bytes:
    return encode_command(CommandType.SERVO_NUDGE_POS if positive else CommandType.SERVO_NUDGE_NEG, ch, seq)

def encode_servo_save_cal(seq: int = 0)     -> bytes: return encode_command(CommandType.SERVO_SAVE_CAL, 0, seq)
def encode_servo_center_all(seq: int = 0)   -> bytes: return encode_command(CommandType.SERVO_CENTER_ALL, 0, seq)
def encode_servo_preflight(seq: int = 0)    -> bytes: return encode_command(CommandType.SERVO_PREFLIGHT, 0, seq)
def encode_sd_start(seq: int = 0)           -> bytes: return encode_command(CommandType.SD_START_RECORDING, 0, seq)
def encode_sd_stop(seq: int = 0)            -> bytes: return encode_command(CommandType.SD_STOP_RECORDING, 0, seq)

def encode_attitude_control_enable(seq: int = 0)  -> bytes: return encode_command(CommandType.ATTITUDE_CONTROL_ENABLE, 0, seq)
def encode_attitude_control_disable(seq: int = 0) -> bytes: return encode_command(CommandType.ATTITUDE_CONTROL_DISABLE, 0, seq)
def encode_attitude_demo_enable(seq: int = 0)     -> bytes: return encode_command(CommandType.ATTITUDE_DEMO_ENABLE, 0, seq)
def encode_attitude_demo_disable(seq: int = 0)    -> bytes: return encode_command(CommandType.ATTITUDE_DEMO_DISABLE, 0, seq)

def encode_reset(seq: int = 0) -> bytes: return encode_command(CommandType.RESET, 0, seq)
