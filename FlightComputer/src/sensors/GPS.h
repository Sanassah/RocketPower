#pragma once
#include <TinyGPSPlus.h>
#include <Wire.h>

struct GPSData {
    double   lat;
    double   lon;
    float    alt_m;
    uint8_t  sats;
    bool     fix;
    bool     valid;

    // Diagnostics — populated even without a fix, useful to tell
    // "no signal reaching the chip" apart from "tracking but no fix yet".
    uint8_t  satsInView;    // from GSV: satellites tracked (fix not required)
    float    hdop;
    uint32_t bytesRxTotal;  // cumulative NMEA bytes drained over I2C
    uint32_t sentencesOk;   // NMEA sentences with valid checksum
    uint32_t sentencesBad;  // checksum failures (I2C read corruption)
};

// ZOEM8 communicates over I2C DDC (Wire1, 0x42).
// NMEA bytes are read from the module and fed to TinyGPSPlus.
class GPS {
public:
    bool begin();
    void update();           // call every loop to drain the I2C NMEA buffer
    const GPSData& data() const { return _data; }

    void printDebug();       // dump a diagnostic line to Serial

private:
    TinyGPSPlus   _parser;
    GPSData       _data{};
    TinyGPSCustom _gsvSatsInView;   // $--GSV field 3: total satellites in view

    void _drainI2C();
};
