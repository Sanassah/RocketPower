#pragma once
#include <SD.h>
#include "../sensors/SensorManager.h"

// Logs FlightData to SD card as a binary file at LOG_INTERVAL_MS (100 Hz).
// File format:
//   - ASCII CSV header (self-describing the binary field layout)
//   - Binary records, each is a raw FlightData struct
// A new numbered file is created at each boot: FLTXXXXX.BIN
class DataLogger {
public:
    bool open();      // call in setup() after SD.begin()
    void close();     // flush and close file (call on landed or power-off)
    void flush();     // flush without closing (call on state change)

    // Call every loop; writes a record if LOG_INTERVAL_MS has elapsed
    void update(const FlightData& d);

    bool isOpen() const { return _isOpen; }

private:
    File     _file;
    bool     _isOpen      = false;
    uint32_t _lastLogMs   = 0;
    uint32_t _recordCount = 0;

    void _writeCsvHeader();
    String _nextFilename();
};
