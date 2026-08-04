#include "GPS.h"
#include "../config.h"

// ZOEM8 I2C DDC registers
#define ZOEM8_REG_BYTES_AVAIL 0xFD   // MSB of bytes-available (followed by 0xFE)

bool GPS::begin() {
    // Probe for device on Wire1
    ZOEM8_I2C_BUS.beginTransmission(ZOEM8_I2C_ADDR);
    if (ZOEM8_I2C_BUS.endTransmission() != 0) return false;
    _gsvSatsInView.begin(_parser, "GPGSV", 3);
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
    if (_parser.hdop.isValid()) {
        _data.hdop   = (float)_parser.hdop.hdop();
    }
    _data.sats         = (uint8_t)_parser.satellites.value();
    _data.satsInView   = (uint8_t)atoi(_gsvSatsInView.value());
    _data.sentencesOk  = _parser.passedChecksum();
    _data.sentencesBad = _parser.failedChecksum();
    _data.valid        = true;
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

    _data.bytesRxTotal += avail;

    while (avail) {
        uint8_t chunk = (avail > 32) ? 32 : (uint8_t)avail;
        ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, chunk);
        while (ZOEM8_I2C_BUS.available()) {
            char c = (char)ZOEM8_I2C_BUS.read();
#if GPS_DEBUG_RAW_NMEA
            Serial.write(c);   // echo raw NMEA sentences for manual inspection
#endif
            _parser.encode(c);
        }
        avail -= chunk;
    }
}

void GPS::printDebug() {
    Serial.print("[GPS] bytesRx=");    Serial.print(_data.bytesRxTotal);
    Serial.print(" sentOk=");          Serial.print(_data.sentencesOk);
    Serial.print(" sentBad=");         Serial.print(_data.sentencesBad);
    Serial.print(" satsInView=");      Serial.print(_data.satsInView);
    Serial.print(" satsUsed=");        Serial.print(_data.sats);
    Serial.print(" fix=");             Serial.print(_data.fix ? "Y" : "N");
    Serial.print(" hdop=");            Serial.print(_data.hdop, 1);
    if (_data.fix) {
        Serial.print(" lat="); Serial.print(_data.lat, 6);
        Serial.print(" lon="); Serial.print(_data.lon, 6);
        Serial.print(" alt="); Serial.print(_data.alt_m, 1);
    }
    Serial.println();

    if (_data.bytesRxTotal == 0) {
        Serial.println("[GPS] WARNING: zero bytes ever received over I2C — check antenna/wiring, not just sky view.");
    } else if (_data.sentencesOk == 0 && _data.sentencesBad > 0) {
        Serial.println("[GPS] WARNING: bytes arriving but all checksums fail — possible I2C corruption.");
    }
}
