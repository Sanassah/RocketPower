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

    // Whether a card answered the last presence check. Deliberately NOT
    // continuously re-probed while idle -- an earlier version called
    // SD.begin() every 2s from this same real-time loop to keep this live,
    // and on real hardware SD.begin() is not cheap (it re-negotiates with
    // the card, and can take a while when there's no card to answer at
    // all). That periodic call was a plausible cause of intermittent
    // telemetry stalls seen on the bench. This now only reflects the last
    // *explicit* attempt: boot (main.cpp), a ground SD_START_RECORDING
    // command, or write success/failure while actively recording. It won't
    // notice a card inserted later on its own -- press Start again.
    bool cardPresent() const { return _cardPresent; }

private:
    File     _file;
    bool     _isOpen        = false;
    bool     _cardPresent   = false;
    uint32_t _lastLogMs     = 0;
    uint32_t _recordCount   = 0;

    // A single short write() isn't necessarily a lost card -- require a run
    // of consecutive failures before closing, so one transient hiccup can't
    // silently end a flight recording early.
    uint8_t  _consecutiveWriteFails = 0;
    static constexpr uint8_t _MAX_CONSECUTIVE_WRITE_FAILS = 5;

    void _writeCsvHeader();
    String _nextFilename();
};
