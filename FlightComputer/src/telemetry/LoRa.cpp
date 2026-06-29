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
    // Mirror to USB serial so the GCS can connect directly via USB for bench testing
    Serial.write(reinterpret_cast<const uint8_t*>(&out), sizeof(out));
    return (written == sizeof(out));
}

bool LoRaRadio::receiveCommand(CommandPacket& pkt) {
    if (LORA_SERIAL.available() < (int)sizeof(CommandPacket)) return false;
    uint8_t* buf = reinterpret_cast<uint8_t*>(&pkt);
    size_t n = LORA_SERIAL.readBytes(buf, sizeof(CommandPacket));
    if (n != sizeof(CommandPacket)) return false;
    if (!_validateCommand(pkt)) return false;
    return true;
}

bool LoRaRadio::_validateCommand(const CommandPacket& pkt) const {
    if (pkt.magic[0] != CMD_MAGIC_0 || pkt.magic[1] != CMD_MAGIC_1) return false;
    size_t checksumLen = sizeof(CommandPacket) - sizeof(pkt.checksum);
    uint16_t expected = packetChecksum(reinterpret_cast<const uint8_t*>(&pkt), checksumLen);
    return (pkt.checksum == expected);
}
