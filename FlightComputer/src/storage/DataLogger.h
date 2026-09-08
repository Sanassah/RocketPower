#pragma once
#include <SD.h>
#include "../sensors/SensorManager.h"

// Logs FlightData to SD card as a binary file at LOG_INTERVAL_MS (100 Hz).
// File format:
//   - ASCII CSV header (self-describing the binary field layout)
//   - Binary records, each is a raw FlightData struct
// A new numbered file is created each time recording starts: FLTXXXXX.BIN
//
// Recording starts automatically at boot (see main.cpp) and stops on LANDED,
// so a normal flight never depends on anyone remembering to press a button.
// open()/close() are also safe to call directly from a ground command (see
// TelemetryManager) to start/stop a log on the bench without power-cycling --
// open() is idempotent (a no-op if already recording) and self-contained (it
// re-probes the card itself rather than assuming an earlier SD.begin() is
// still valid), since by the time a ground command arrives, boot is long over.
class DataLogger {
public:
    bool open();      // starts a new log file; false if no card. No-op (true) if already open.
    void close();     // flush and close file (call on landed, power-off, or ground command)
    void flush();     // flush without closing (call on state change)

    // Call every loop. Writes a record if LOG_INTERVAL_MS has elapsed while
    // open; a no-op while closed (see cardPresent() below for why this
    // doesn't also poll for a card).
    void update(const FlightData& d);

    bool isOpen() const { return _isOpen; }

    // Whether a card answered the last presence check. NOT continuously
    // re-probed while idle -- SD.begin() isn't cheap (re-negotiates with the
    // card, worse with none present), and doing that every 2s from the real-
    // time loop was a plausible cause of bench telemetry stalls. Only
    // reflects the last *explicit* attempt (boot/ground command/write
    // result) -- won't notice a card inserted later on its own.
    bool cardPresent() const { return _cardPresent; }

private:
    File     _file;
    bool     _isOpen        = false;
    bool     _cardPresent   = false;
    uint32_t _lastLogMs     = 0;
    uint32_t _recordCount   = 0;

    // Require 2 consecutive write failures (not 1) before closing -- so one
    // transient hiccup (e.g. in-flight vibration) can't end a recording
    // early -- but no higher: each failed write() can itself block ~1-3s
    // (SdFat's hardcoded 1s-per-bus-wait BUSY_TIMEOUT_MICROS, uncustomizable
    // short of forking all of SdFat, declined). Worst case before giving up
    // on a truly pulled card: ~2-6s frozen, down from ~5-15s at the old
    // retry count.
    uint8_t  _consecutiveWriteFails = 0;
    static constexpr uint8_t _MAX_CONSECUTIVE_WRITE_FAILS = 2;

    void _writeCsvHeader();
    String _nextFilename();
};
