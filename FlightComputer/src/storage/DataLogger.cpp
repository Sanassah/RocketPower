#include "DataLogger.h"
#include "../config.h"

// Binary file starts with an ASCII CSV header block so post-processing tools
// can discover field names and offsets without a separate schema file.
static const char CSV_HEADER[] =
    "# RocketFlightComputer binary log\n"
    "# Record format: raw FlightData struct, little-endian\n"
    "# Fields (in order): timestamp_ms,state,"
    "quat_w,quat_x,quat_y,quat_z,"
    "lin_accel_x,lin_accel_y,lin_accel_z,"
    "gyro_x,gyro_y,gyro_z,"
    "pressure_hpa,temperature_c,baro_alt_m,vert_vel_ms,"
    "highg_x_g,highg_y_g,highg_z_g,highg_mag_g,"
    "lat,lon,gps_alt_m,gps_sats,gps_fix,"
    "voltage_v,current_ma,power_mw\n"
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
    String name = _nextFilename();
    _file = SD.open(name.c_str(), FILE_WRITE);
    if (!_file) return false;

    _writeCsvHeader();
    _isOpen      = true;
    _recordCount = 0;
    Serial.print("[LOG] Opened "); Serial.println(name);
    return true;
}

void DataLogger::_writeCsvHeader() {
    _file.write((const uint8_t*)CSV_HEADER, strlen(CSV_HEADER));
    _file.write(BINARY_SENTINEL);   // marks start of binary section
    _file.flush();
}

void DataLogger::update(const FlightData& d) {
    if (!_isOpen) return;
    uint32_t now = millis();
    if (now - _lastLogMs < LOG_INTERVAL_MS) return;
    _lastLogMs = now;

    _file.write(reinterpret_cast<const uint8_t*>(&d), sizeof(FlightData));
    _recordCount++;

    // Flush every 100 records to bound data loss on power failure
    if (_recordCount % 100 == 0) _file.flush();
}

void DataLogger::flush() {
    if (_isOpen) _file.flush();
}

void DataLogger::close() {
    if (_isOpen) {
        _file.flush();
        _file.close();
        _isOpen = false;
        Serial.print("[LOG] Closed. Records: "); Serial.println(_recordCount);
    }
}
