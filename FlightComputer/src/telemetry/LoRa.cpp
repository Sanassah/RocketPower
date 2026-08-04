#include "LoRa.h"
#include "../config.h"
#include <string.h>

bool LoRaRadio::begin() {
    LORA_SERIAL.begin(LORA_BAUD);
    return true;
}

bool LoRaRadio::send(const TelemetryPacket& pkt) {
    TelemetryPacket out = pkt;
    out.seq = _txSeq++;
    size_t checksumLen = sizeof(TelemetryPacket) - sizeof(out.checksum);
    out.checksum = packetChecksum(reinterpret_cast<const uint8_t*>(&out), checksumLen);
    size_t written = LORA_SERIAL.write(reinterpret_cast<const uint8_t*>(&out), sizeof(out));

    // Binary mirror on USB so the ground station GUI can connect directly.
    // 0xAA is not a valid ASCII byte, so text output can't produce the magic pair
    // 0xAA 0x55 — the GUI's scanner ignores all bytes between packets safely.
    Serial.write(reinterpret_cast<const uint8_t*>(&out), sizeof(out));

    // Human-readable summary for terminal debugging (pure ASCII, won't confuse the binary parser)
    // Wire fields are scaled fixed-point (see Packet.h) -- unscale for display.
    Serial.print("[TELEM] seq="); Serial.print(out.seq);
    Serial.print(" alt=");        Serial.print(out.baro_alt_dm / 10.0f, 1); Serial.print("m");
    Serial.print(" vel=");        Serial.print(out.vert_vel_cms / 100.0f, 1); Serial.print("m/s");
    Serial.print(" accel=");      Serial.print(out.accel_x_cg / 100.0f, 2); Serial.print("g");
    Serial.print(" v=");          Serial.print(out.voltage_cv / 100.0f, 2); Serial.print("V");
    Serial.print(" sats=");       Serial.print(out.gps_sats);
    Serial.println();

    return (written == sizeof(out));
}

bool LoRaRadio::receiveCommandFrom(Stream& src, CommandPacket& pkt) {
    return _drainCommand(src, _usbBuf, _usbBufLen, pkt);
}

bool LoRaRadio::receiveCommand(CommandPacket& pkt) {
    return _drainCommand(LORA_SERIAL, _loraBuf, _loraBufLen, pkt);
}

bool LoRaRadio::_drainCommand(Stream& src, uint8_t* buf, uint8_t& len, CommandPacket& pkt) {
    // Top up the buffer with whatever's newly arrived, bounded so it can't overflow.
    while (src.available() && len < _CMD_BUF_CAP) {
        buf[len++] = (uint8_t)src.read();
    }

    while (true) {
        // Search buffered bytes for the magic pair.
        int magicIdx = -1;
        for (uint8_t i = 0; (int)i + 1 < (int)len; i++) {
            if (buf[i] == CMD_MAGIC_0 && buf[i + 1] == CMD_MAGIC_1) {
                magicIdx = i;
                break;
            }
        }
        if (magicIdx < 0) {
            // No magic in the buffer; keep the last byte in case it's the
            // first half of a magic pair split across reads.
            if (len > 0) buf[0] = buf[len - 1];
            len = (len > 0) ? 1 : 0;
            return false;
        }
        if (magicIdx > 0) {
            // Discard bytes before the magic.
            memmove(buf, buf + magicIdx, len - magicIdx);
            len -= magicIdx;
        }
        if (len < sizeof(CommandPacket)) return false;  // wait for more bytes

        memcpy(&pkt, buf, sizeof(CommandPacket));
        memmove(buf, buf + sizeof(CommandPacket), len - sizeof(CommandPacket));
        len -= sizeof(CommandPacket);

        if (_validateCommand(pkt)) return true;
        // Bad checksum at this offset -- loop and search the remaining
        // buffered bytes for another magic occurrence instead of giving up.
    }
}

bool LoRaRadio::_validateCommand(const CommandPacket& pkt) const {
    if (pkt.magic[0] != CMD_MAGIC_0 || pkt.magic[1] != CMD_MAGIC_1) return false;
    size_t checksumLen = sizeof(CommandPacket) - sizeof(pkt.checksum);
    uint16_t expected = packetChecksum(reinterpret_cast<const uint8_t*>(&pkt), checksumLen);
    return (pkt.checksum == expected);
}
