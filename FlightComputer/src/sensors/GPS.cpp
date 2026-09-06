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
    // TEMPORARY diagnostic (RocketPower bench investigation) -- splits this
    // call into its 3 I2C stages to find where the remaining ~8ms goes,
    // since the byte-budget cap below already cut this from ~24ms and the
    // module's own data rate (~230 bytes/s, bench-measured via bytesRx) is
    // too low for "big burst needs many chunks" to explain the rest -- most
    // calls should see a small `avail`. Not durable, remove once the cause
    // is found. See [GPS BREAKDOWN] below.
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
    uint32_t i2cXferUs = 0, drainBufUs = 0, parseUs = 0;

    // 0 or 0xFFFF just means "nothing buffered right now" -- not a comms
    // failure, so still report the module as alive.
    if (avail != 0 && avail != 0xFFFF) {
        // BENCH-MEASURED (see main.cpp's [SENSOR BREAKDOWN] diagnostic): the
        // module bursts a whole second's worth of NMEA sentences (~200-500
        // bytes) at once, roughly 1Hz. Draining an arbitrarily large burst
        // in one update() call would stall this loop iteration -- same
        // category of problem as the IMU's own bounded events-per-call cap
        // in SensorManager.cpp, and the same fix: bound how much ONE call
        // can drain, so a burst spreads across several loop iterations
        // instead of stalling a single one. A whole second of NMEA easily
        // drains across the ~100 loop iterations before the next burst
        // arrives, so this doesn't risk falling behind -- it only spreads
        // the SAME total work out.
        if (avail > 64) avail = 64;
        _data.bytesRxTotal += avail;

        ZOEM8_I2C_BUS.requestFrom((uint8_t)ZOEM8_I2C_ADDR, (uint8_t)avail);
        uint32_t _dbg_t2b = micros();   // TEMPORARY diagnostic -- isolates the I2C transfer itself

        // TEMPORARY diagnostic -- drain Wire's already-filled buffer into a
        // local array with NO parsing yet, so parsing cost (below) is
        // isolated from I2C/Wire-buffer cost. GPS_DEBUG_RAW_NMEA is
        // confirmed off (config.h), so that's not it -- last remaining
        // candidate after ruling out I2C transaction count (1 vs 2 chunks
        // made no difference to the total) is _parser.encode() itself
        // (TinyGPS++, plus the GPGSV custom-field hook).
        uint8_t localBuf[64];
        uint8_t n = 0;
        while (ZOEM8_I2C_BUS.available() && n < sizeof(localBuf)) {
            localBuf[n++] = (uint8_t)ZOEM8_I2C_BUS.read();
        }
        uint32_t _dbg_t2c = micros();

        for (uint8_t i = 0; i < n; i++) {
            _parser.encode((char)localBuf[i]);
        }
        uint32_t _dbg_t3 = micros();

        i2cXferUs  = _dbg_t2b - _dbg_t2;
        drainBufUs = _dbg_t2c - _dbg_t2b;
        parseUs    = _dbg_t3  - _dbg_t2c;
    }

    {
        static uint32_t _dbg_maxRegWrite = 0, _dbg_maxCountRead = 0;
        static uint32_t _dbg_maxI2cXfer = 0, _dbg_maxDrainBuf = 0, _dbg_maxParse = 0;
        static uint16_t _dbg_maxAvail = 0;
        uint32_t regWriteUs  = _dbg_t1 - _dbg_t0;
        uint32_t countReadUs = _dbg_t2 - _dbg_t1;
        if (regWriteUs  > _dbg_maxRegWrite)  _dbg_maxRegWrite  = regWriteUs;
        if (countReadUs > _dbg_maxCountRead) _dbg_maxCountRead = countReadUs;
        if (i2cXferUs   > _dbg_maxI2cXfer)   _dbg_maxI2cXfer   = i2cXferUs;
        if (drainBufUs  > _dbg_maxDrainBuf)  _dbg_maxDrainBuf  = drainBufUs;
        if (parseUs     > _dbg_maxParse)     _dbg_maxParse     = parseUs;
        if (availOrig   > _dbg_maxAvail)     _dbg_maxAvail     = availOrig;
        static uint32_t _dbg_lastPrintMs = 0;
        if (millis() - _dbg_lastPrintMs >= 5000) {
            _dbg_lastPrintMs = millis();
            // ms, 3 decimals -- full us precision, just relabeled (see main.cpp).
            // maxAvail is a byte count, not a time -- left unconverted.
            DEBUG_SERIAL.print("[GPS BREAKDOWN] regWrite="); DEBUG_SERIAL.print(_dbg_maxRegWrite / 1000.0f, 3);
            DEBUG_SERIAL.print("ms countRead="); DEBUG_SERIAL.print(_dbg_maxCountRead / 1000.0f, 3);
            DEBUG_SERIAL.print("ms i2cXfer="); DEBUG_SERIAL.print(_dbg_maxI2cXfer / 1000.0f, 3);
            DEBUG_SERIAL.print("ms drainBuf="); DEBUG_SERIAL.print(_dbg_maxDrainBuf / 1000.0f, 3);
            DEBUG_SERIAL.print("ms parse="); DEBUG_SERIAL.print(_dbg_maxParse / 1000.0f, 3);
            DEBUG_SERIAL.print("ms maxAvail="); DEBUG_SERIAL.println(_dbg_maxAvail);
            _dbg_maxRegWrite = _dbg_maxCountRead = 0;
            _dbg_maxI2cXfer = _dbg_maxDrainBuf = _dbg_maxParse = 0;
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
