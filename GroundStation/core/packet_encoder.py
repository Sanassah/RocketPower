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
    ARM       = 0x01
    DISARM    = 0x02
    FIRE_PYRO = 0x03
    PING      = 0x04


def encode_command(cmd_type: CommandType, param: int = 0) -> bytes:
    """Build a validated CommandPacket as bytes ready to send over serial."""
    payload  = struct.pack('<BBBB', CMD_MAGIC_0, CMD_MAGIC_1, int(cmd_type), param & 0xFF)
    checksum = sum(payload) & 0xFFFF
    return payload + struct.pack('<H', checksum)


def encode_arm()                -> bytes: return encode_command(CommandType.ARM)
def encode_disarm()             -> bytes: return encode_command(CommandType.DISARM)
def encode_fire_pyro(ch: int)   -> bytes: return encode_command(CommandType.FIRE_PYRO, ch)
def encode_ping()               -> bytes: return encode_command(CommandType.PING)
