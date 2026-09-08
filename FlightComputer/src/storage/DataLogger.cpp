#include "DataLogger.h"
#include "../config.h"
#include "../DebugPrint.h"

// Binary file starts with an ASCII CSV header block so post-processing tools
// can discover field names and offsets without a separate schema file.
// NOTE: this lists field NAMES in struct order, not byte offsets -- FlightData
// is a plain (non-packed) struct, so the compiler inserts alignment padding
// between some fields (notably after `state` and after `gps_fix`). A decoder
// needs the real struct layout, not just this name list, to get offsets
// right -- see GroundStation/tools/log_convert.py, which hardcodes the
// current layout and must be updated if FlightData's fields ever change.
static const char CSV_HEADER[] =
    "# RocketFlightComputer binary log\n"
    "# Record format: raw FlightData struct, little-endian, sizeof == 136 bytes\n"
    "# Fields (in order, NOT packed -- see GroundStation/tools/log_convert.py for real offsets):\n"
    "# timestamp_ms,state,"
    "quat_w,quat_x,quat_y,quat_z,"
    "lin_accel_x,lin_accel_y,lin_accel_z,"
    "gyro_x,gyro_y,gyro_z,"
    "accel_mag_ms2,"
    "pressure_hpa,temperature_c,baro_alt_m,vert_vel_ms,"
    "highg_x_g,highg_y_g,highg_z_g,highg_mag_g,"
    "lat,lon,gps_alt_m,gps_sats,gps_fix,"
    "voltage_v,current_ma,power_mw,"
    "imu_ok,baro_ok,accel_ok,gps_ok,power_ok\n"
    "# END_HEADER\n";

// Header sentinel lets a parser skip past ASCII bytes to binary records
#define BINARY_SENTINEL 0x1E   // ASCII "record separator"

String DataLogger::_nextFilename() {
    for (uint32_t i = 1; i <= 99999; i++) {
        char name[16];
        snprintf(name, sizeof(name), "FLT%05lu.BIN", (unsigned long)i);
        if (!SD.exists(name)) return String(name);
    }
    return "FLIGHT.BIN";
}

bool DataLogger::open() {
    if (_isOpen) return true;   // already recording -- no-op, not an error

    // Self-contained: re-probes the card itself rather than trusting an
    // earlier SD.begin() elsewhere is still valid. Matters once this can be
    // called long after boot from a ground command, not just from setup().
    if (!SD.begin(SD_CS_PIN)) {
        _cardPresent = false;
        return false;
    }
    _cardPresent = true;

    String name = _nextFilename();
    _file = SD.open(name.c_str(), FILE_WRITE);
    if (!_file) return false;

    _writeCsvHeader();
    _isOpen      = true;
    _recordCount = 0;
    _consecutiveWriteFails = 0;
    DEBUG_SERIAL.print("[LOG] Opened "); DEBUG_SERIAL.println(name);
    return true;
}

void DataLogger::_writeCsvHeader() {
    _file.write((const uint8_t*)CSV_HEADER, strlen(CSV_HEADER));
    _file.write(BINARY_SENTINEL);   // marks start of binary section
    _file.flush();
}

void DataLogger::update(const FlightData& d) {
    if (!_isOpen) return;   // idle: no periodic re-probe -- see cardPresent() doc comment

    uint32_t now = millis();
    if (now - _lastLogMs < LOG_INTERVAL_MS) return;
    _lastLogMs = now;

    uint32_t _dbg_t0 = micros();
    size_t written = _file.write(reinterpret_cast<const uint8_t*>(&d), sizeof(FlightData));

    if (written != sizeof(FlightData)) {
        _consecutiveWriteFails++;
        if (_consecutiveWriteFails < _MAX_CONSECUTIVE_WRITE_FAILS) return;   // could be a one-off blip

        // Several writes in a row failed -- card pulled, or a genuine fault.
        // Stop instead of hammering a dead card every loop.
        DEBUG_SERIAL.println("[LOG] WARNING: repeated write failures -- card removed? Closing log.");
        _cardPresent = false;
        close();
        return;
    }
    _consecutiveWriteFails = 0;
    _cardPresent = true;
    _recordCount++;

    // Flush every 100 records to bound data loss on power failure
    if (_recordCount % 100 == 0) _file.flush();

    // Combined write+occasional-flush time -- main.cpp's [LOOP BREAKDOWN]
    // log= already tracks this same call from outside; this is just a
    // permanent record of it in DataLogger's own bench output too.
    {
        static uint32_t _dbg_maxUs = 0;
        static uint32_t _dbg_lastPrintMs = 0;
        uint32_t stepUs = micros() - _dbg_t0;
        if (stepUs > _dbg_maxUs) _dbg_maxUs = stepUs;
        if (debugPrintReady(_dbg_lastPrintMs, 5000)) {
            DEBUG_SERIAL.print("[LOG] step="); DEBUG_SERIAL.print(_dbg_maxUs / 1000.0f, 3);
            DEBUG_SERIAL.println("ms max over the last 5s");
            _dbg_maxUs = 0;
        }
    }
}

void DataLogger::flush() {
    if (_isOpen) _file.flush();
}

void DataLogger::close() {
    if (_isOpen) {
        _file.flush();
        _file.close();
        _isOpen = false;
        DEBUG_SERIAL.print("[LOG] Closed. Records: "); DEBUG_SERIAL.println(_recordCount);
    }
}
