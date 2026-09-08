#include "GPS.h"
#include "../config.h"
#include "../DebugPrint.h"

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

bool GPS::update() {
    bool acked = _drainI2C();

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
    return acked;
}

bool GPS::_drainI2C() {
    // [GPS BREAKDOWN] below: per-stage I2C timing (i2cXfer cost model:
    // ~680us fixed + ~98us/byte, bench-derived).
    uint32_t _dbg_t0 = micros();

    // Read how many NMEA bytes the module has buffered
    ZOEM8_I2C_BUS.beginTransmission(ZOEM8_I2C_ADDR);
    ZOEM8_I2C_BUS.write(ZOEM8_REG_BYTES_AVAIL);
    if (ZOEM8_I2C_BUS.endTransmission(false) != 0) return false;   // module didn't ack
    uint32_t _dbg_t1 = micros();

    ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, (uint8_t)2);
    if (ZOEM8_I2C_BUS.available() < 2) return false;   // module didn't return the count bytes
    uint16_t avail = ((uint16_t)ZOEM8_I2C_BUS.read() << 8) | ZOEM8_I2C_BUS.read();
    uint32_t _dbg_t2 = micros();

    uint16_t availOrig = avail;
    uint32_t xferUs = 0;

    // 0 or 0xFFFF just means "nothing buffered right now" -- not a comms
    // failure, so still report the module as alive.
    if (avail != 0 && avail != 0xFFFF) {
        // Cap per-call drain: module bursts ~200-500B once/sec -- draining
        // it all in one update() would stall that loop iteration (same fix
        // as IMU's bounded per-call cap). Spreads across ~100 iterations
        // before the next burst, no risk of falling behind.
        if (avail > 64) avail = 64;
        _data.bytesRxTotal += avail;

        // requestFrom() is blocking -- the real I2C transfer cost happens
        // INSIDE this call, before it returns, not in the drain loop after
        // it. Timed from _dbg_t2 (before this call) for that reason.
        ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, (uint8_t)avail);

        while (ZOEM8_I2C_BUS.available()) {
            char c = (char)ZOEM8_I2C_BUS.read();
            _parser.encode(c);
#if GPS_DEBUG_RAW_NMEA
            DEBUG_SERIAL.write(c);
#endif
        }

        xferUs = micros() - _dbg_t2;
    }

    {
        static uint32_t _dbg_maxRegWrite = 0, _dbg_maxCountRead = 0, _dbg_maxXfer = 0;
        static uint16_t _dbg_maxAvail = 0;
        uint32_t regWriteUs  = _dbg_t1 - _dbg_t0;
        uint32_t countReadUs = _dbg_t2 - _dbg_t1;
        if (regWriteUs  > _dbg_maxRegWrite)  _dbg_maxRegWrite  = regWriteUs;
        if (countReadUs > _dbg_maxCountRead) _dbg_maxCountRead = countReadUs;
        if (xferUs      > _dbg_maxXfer)      _dbg_maxXfer      = xferUs;
        if (availOrig   > _dbg_maxAvail)     _dbg_maxAvail     = availOrig;
        static uint32_t _dbg_lastPrintMs = 0;
        if (debugPrintReady(_dbg_lastPrintMs, 5000)) {
            // ms, 3 decimals. maxAvail is a byte count, left unconverted.
            DEBUG_SERIAL.print("[GPS BREAKDOWN] regWrite="); DEBUG_SERIAL.print(_dbg_maxRegWrite / 1000.0f, 3);
            DEBUG_SERIAL.print("ms countRead="); DEBUG_SERIAL.print(_dbg_maxCountRead / 1000.0f, 3);
            DEBUG_SERIAL.print("ms xfer+parse="); DEBUG_SERIAL.print(_dbg_maxXfer / 1000.0f, 3);
            DEBUG_SERIAL.print("ms maxAvail="); DEBUG_SERIAL.println(_dbg_maxAvail);
            _dbg_maxRegWrite = _dbg_maxCountRead = _dbg_maxXfer = 0;
            _dbg_maxAvail = 0;
        }
    }
    return true;
}

void GPS::printDebug() {
    DEBUG_SERIAL.print("[GPS] bytesRx=");    DEBUG_SERIAL.print(_data.bytesRxTotal);
    DEBUG_SERIAL.print(" sentOk=");          DEBUG_SERIAL.print(_data.sentencesOk);
    DEBUG_SERIAL.print(" sentBad=");         DEBUG_SERIAL.print(_data.sentencesBad);
    DEBUG_SERIAL.print(" satsInView=");      DEBUG_SERIAL.print(_data.satsInView);
    DEBUG_SERIAL.print(" satsUsed=");        DEBUG_SERIAL.print(_data.sats);
    DEBUG_SERIAL.print(" fix=");             DEBUG_SERIAL.print(_data.fix ? "Y" : "N");
    DEBUG_SERIAL.print(" hdop=");            DEBUG_SERIAL.print(_data.hdop, 1);
    if (_data.fix) {
        DEBUG_SERIAL.print(" lat="); DEBUG_SERIAL.print(_data.lat, 6);
        DEBUG_SERIAL.print(" lon="); DEBUG_SERIAL.print(_data.lon, 6);
        DEBUG_SERIAL.print(" alt="); DEBUG_SERIAL.print(_data.alt_m, 1);
    }
    DEBUG_SERIAL.println();

    if (_data.bytesRxTotal == 0) {
        DEBUG_SERIAL.println("[GPS] WARNING: zero bytes ever received over I2C — check antenna/wiring, not just sky view.");
    } else if (_data.sentencesOk == 0 && _data.sentencesBad > 0) {
        DEBUG_SERIAL.println("[GPS] WARNING: bytes arriving but all checksums fail — possible I2C corruption.");
    }
}
