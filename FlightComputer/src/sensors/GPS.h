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
};

// ZOEM8 communicates over I2C DDC (Wire1, 0x42).
// NMEA bytes are read from the module and fed to TinyGPSPlus.
class GPS {
public:
    bool begin();
    void update();           // call every loop to drain the I2C NMEA buffer
    const GPSData& data() const { return _data; }

private:
    TinyGPSPlus _parser;
    GPSData     _data{};

    void _drainI2C();
};
