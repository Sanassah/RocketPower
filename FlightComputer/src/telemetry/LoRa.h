#pragma once
#include <Arduino.h>
#include "Packet.h"

// E22-900T22D is hardwired in Mode 0 (transparent UART) — M0/M1/AUX not
// connected to MCU. No library needed: bytes written to Serial1 are broadcast.
class LoRaRadio {
public:
    bool begin();
    bool send(const TelemetryPacket& pkt);
    bool receiveCommand(CommandPacket& pkt);                  // reads from LoRa UART
    bool receiveCommandFrom(Stream& src, CommandPacket& pkt); // reads from any stream (e.g. USB Serial)
    int  getRSSI() const { return -1; }   // unavailable in transparent mode

private:
    uint16_t _txSeq = 0;
    bool _validateCommand(const CommandPacket& pkt) const;
};
