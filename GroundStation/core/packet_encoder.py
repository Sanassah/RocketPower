"""
Encodes CommandPackets to send to the flight computer.

CommandPacket layout (packed, little-endian, 6 bytes):
  0  uint8   magic[0]  = 0xBB
  1  uint8   magic[1]  = 0x44
  2  uint8   type      (CommandType enum)
  3  uint8   param     (channel for FIRE_PYRO, else 0)
  4  uint16  checksum  (sum of bytes 0..3)
"""

import struct
from enum import IntEnum

CMD_MAGIC_0 = 0xBB
CMD_MAGIC_1 = 0x44

CMD_FORMAT = '<BBBBH'
CMD_SIZE   = struct.calcsize(CMD_FORMAT)  # 6


class CommandType(IntEnum):
    ARM            = 0x01
    DISARM         = 0x02
    FIRE_PYRO      = 0x03
    PING           = 0x04
    CALIBRATE_BARO = 0x05   # re-zero barometer at current ground level
    SERVO_TEST      = 0x06  # param = fin channel (1-4); sweeps center->min->max->center
    CAM_START       = 0x07  # manually start camera recording (bench test)
    CAM_STOP        = 0x08  # manually stop camera recording (bench test)
    SERVO_NUDGE_POS  = 0x09  # param = fin channel (1-4); +trim step, not persisted
    SERVO_NUDGE_NEG  = 0x0A  # param = fin channel (1-4); -trim step, not persisted
    SERVO_SAVE_CAL   = 0x0B  # param unused; persists all 4 channels' live position as new trim
    SERVO_CENTER_ALL = 0x0C  # param unused; drives all 4 to raw center, ignoring trim, not persisted
    SERVO_PREFLIGHT  = 0x0D  # param unused; blocking ~6s all-4 choreography


def encode_command(cmd_type: CommandType, param: int = 0) -> bytes:
    """Build a validated CommandPacket as bytes ready to send over serial."""
    payload  = struct.pack('<BBBB', CMD_MAGIC_0, CMD_MAGIC_1, int(cmd_type), param & 0xFF)
    checksum = sum(payload) & 0xFFFF
    return payload + struct.pack('<H', checksum)


def encode_arm()                -> bytes: return encode_command(CommandType.ARM)
def encode_disarm()             -> bytes: return encode_command(CommandType.DISARM)
def encode_fire_pyro(ch: int)   -> bytes: return encode_command(CommandType.FIRE_PYRO, ch)
def encode_ping()               -> bytes: return encode_command(CommandType.PING)
def encode_calibrate()          -> bytes: return encode_command(CommandType.CALIBRATE_BARO)
def encode_servo_test(ch: int)  -> bytes: return encode_command(CommandType.SERVO_TEST, ch)
def encode_cam_start()          -> bytes: return encode_command(CommandType.CAM_START)
def encode_cam_stop()           -> bytes: return encode_command(CommandType.CAM_STOP)

def encode_servo_nudge(ch: int, positive: bool) -> bytes:
    return encode_command(CommandType.SERVO_NUDGE_POS if positive else CommandType.SERVO_NUDGE_NEG, ch)

def encode_servo_save_cal()     -> bytes: return encode_command(CommandType.SERVO_SAVE_CAL)
def encode_servo_center_all()   -> bytes: return encode_command(CommandType.SERVO_CENTER_ALL)
def encode_servo_preflight()    -> bytes: return encode_command(CommandType.SERVO_PREFLIGHT)
