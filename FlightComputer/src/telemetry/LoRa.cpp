#include "LoRa.h"
#include "../config.h"

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
    Serial.print("[TELEM] seq="); Serial.print(out.seq);
    Serial.print(" alt=");        Serial.print(out.baro_alt_m, 1); Serial.print("m");
    Serial.print(" vel=");        Serial.print(out.vert_vel_ms, 1); Serial.print("m/s");
    Serial.print(" accel=");      Serial.print(out.accel_x_g, 2); Serial.print("g");
    Serial.print(" v=");          Serial.print(out.voltage_v, 2); Serial.print("V");
    Serial.print(" sats=");       Serial.print(out.gps_sats);
    Serial.println();

    return (written == sizeof(out));
}

bool LoRaRadio::receiveCommandFrom(Stream& src, CommandPacket& pkt) {
    if (src.available() < (int)sizeof(CommandPacket)) return false;
    uint8_t* buf = reinterpret_cast<uint8_t*>(&pkt);
    size_t n = src.readBytes(buf, sizeof(CommandPacket));
    if (n != sizeof(CommandPacket)) return false;
    if (!_validateCommand(pkt)) return false;
    return true;
}

bool LoRaRadio::receiveCommand(CommandPacket& pkt) {
    return receiveCommandFrom(LORA_SERIAL, pkt);
}

bool LoRaRadio::_validateCommand(const CommandPacket& pkt) const {
    if (pkt.magic[0] != CMD_MAGIC_0 || pkt.magic[1] != CMD_MAGIC_1) return false;
    size_t checksumLen = sizeof(CommandPacket) - sizeof(pkt.checksum);
    uint16_t expected = packetChecksum(reinterpret_cast<const uint8_t*>(&pkt), checksumLen);
    return (pkt.checksum == expected);
}
