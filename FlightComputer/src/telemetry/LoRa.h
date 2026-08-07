#pragma once
#include <Arduino.h>
#include "Packet.h"

// E22-900T22D is hardwired in Mode 0 (transparent UART) — M0/M1/AUX not
// connected to MCU. No library needed: bytes written to Serial1 are broadcast.
class LoRaRadio {
public:
    bool begin();
    bool send(const TelemetryPacket& pkt);

    // High-rate USB-only telemetry mirror, decoupled from the LoRa airtime
    // budget -- see USB_TELEMETRY_INTERVAL_MS in config.h. Binary bytes only,
    // no human-readable summary line (that's fine at 1Hz, not at 20Hz+).
    bool sendUsbFast(const TelemetryPacket& pkt);

    bool sendAck(const AckPacket& pkt);
    bool receiveCommand(CommandPacket& pkt);                  // reads from LoRa UART
    bool receiveCommandFrom(Stream& src, CommandPacket& pkt); // reads from any stream (e.g. USB Serial)
    int  getRSSI() const { return -1; }   // unavailable in transparent mode

private:
    uint16_t _txSeq = 0;
    bool _validateCommand(const CommandPacket& pkt) const;

    // Assigns the next seq number and checksum -- shared by send() and
    // sendUsbFast() so both draw from the same seq space.
    TelemetryPacket _finalize(const TelemetryPacket& pkt);

    // Resyncing receive buffers, one per stream so USB and LoRa command
    // reception never interleave. LoRa is a noisier link than a wired USB
    // connection -- a single stray/corrupted byte can permanently misalign a
    // naive fixed-size read that never re-searches for the magic bytes
    // within already-buffered data, so this scans for the magic pair and
    // only discards one byte at a time on a bad frame instead of the whole
    // window.
    static const uint8_t _CMD_BUF_CAP = 32;
    uint8_t _usbBuf[_CMD_BUF_CAP];
    uint8_t _usbBufLen = 0;
    uint8_t _loraBuf[_CMD_BUF_CAP];
    uint8_t _loraBufLen = 0;

    bool _drainCommand(Stream& src, uint8_t* buf, uint8_t& len, CommandPacket& pkt);
};
