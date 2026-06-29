#include "GPS.h"
#include "../config.h"

// ZOEM8 I2C DDC registers
#define ZOEM8_REG_BYTES_AVAIL 0xFD   // MSB of bytes-available (followed by 0xFE)

bool GPS::begin() {
    // Probe for device on Wire1
    ZOEM8_I2C_BUS.beginTransmission(ZOEM8_I2C_ADDR);
    if (ZOEM8_I2C_BUS.endTransmission() != 0) return false;
    _data.valid = true;
    return true;
}

void GPS::update() {
    _drainI2C();

    if (_parser.location.isValid()) {
        _data.lat    = _parser.location.lat();
        _data.lon    = _parser.location.lng();
        _data.fix    = true;
    } else {
        _data.fix    = false;
    }
    if (_parser.altitude.isValid()) {
        _data.alt_m  = (float)_parser.altitude.meters();
    }
    _data.sats   = (uint8_t)_parser.satellites.value();
    _data.valid  = true;
}

void GPS::_drainI2C() {
    // Read how many NMEA bytes the module has buffered
    ZOEM8_I2C_BUS.beginTransmission(ZOEM8_I2C_ADDR);
    ZOEM8_I2C_BUS.write(ZOEM8_REG_BYTES_AVAIL);
    if (ZOEM8_I2C_BUS.endTransmission(false) != 0) return;

    ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, (uint8_t)2);
    if (ZOEM8_I2C_BUS.available() < 2) return;
    uint16_t avail = ((uint16_t)ZOEM8_I2C_BUS.read() << 8) | ZOEM8_I2C_BUS.read();

    if (avail == 0 || avail == 0xFFFF) return;
    if (avail > 512) avail = 512;   // sanity cap

    while (avail) {
        uint8_t chunk = (avail > 32) ? 32 : (uint8_t)avail;
        ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, chunk);
        while (ZOEM8_I2C_BUS.available()) _parser.encode((char)ZOEM8_I2C_BUS.read());
        avail -= chunk;
    }
}
