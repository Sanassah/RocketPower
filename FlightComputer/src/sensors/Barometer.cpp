#include "Barometer.h"
#include "../config.h"
#include "../DebugPrint.h"
#include <math.h>

bool Barometer::begin() {
    if (!_bmp.begin_I2C(BMP390_I2C_ADDR, &BMP390_I2C_BUS)) return false;
    // 4X/4X, not 8X/4X: 8X temp + 4X press needs ~24.9ms/conversion, exceeds
    // the 50Hz ODR's 20ms budget -- enableNormalMode() hard-fails on that
    // combo (bench-confirmed). 4X/4X fits in ~16.9ms. Temp is the one
    // reduced since it's only a pressure-compensation input, not the output.
    _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    _bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // Normal mode: chip samples continuously in the background, performReading()
    // just fetches the latest value (<1ms). Forced mode (library default)
    // blocks ~13-19ms/call, every loop, forever -- see the RocketPower fork
    // in lib/Adafruit BMP3XX Library/. Return value checked: a silent
    // failure here previously left baro_ok permanently false with no error.
    if (!_bmp.enableNormalMode()) return false;
    _data.valid = true;
    return true;
}

bool Barometer::update() {
    if (!_bmp.performReading()) return false;

    _data.pressure_hpa  = _bmp.pressure / 100.0f;
    _data.temperature_c = _bmp.temperature;

    // Pressure ratio vs ground pressure -- immune to QNH error (a fixed
    // sea-level reference would be off by a constant on a day it doesn't
    // match reality; dividing pressures directly cancels that out).
    _data.rel_altitude_m = 44330.0f * (1.0f - powf(_data.pressure_hpa / _groundPressure_hpa, 0.1903f));

    // Kept permanently. UNRESOLVED (2026-09): several-meter swing during LoRa
    // TX. Cause: RF coupling from antenna (~7cm, near-field @915MHz), NOT
    // supply-rail -- antenna unplugged = clean, same TX current either way.
    // Fix = shielding/distance, not capacitors (bulk cap tried, inconsistent).
    {
        static uint32_t lastPrintMs = 0;
        if (debugPrintReady(lastPrintMs)) {
            DEBUG_SERIAL.print("[BARO RAW] pressure_hpa="); DEBUG_SERIAL.print(_data.pressure_hpa, 4);
            DEBUG_SERIAL.print(" groundPressure_hpa="); DEBUG_SERIAL.print(_groundPressure_hpa, 4);
            DEBUG_SERIAL.print(" bmp.pressure="); DEBUG_SERIAL.print(_bmp.pressure, 4);
            DEBUG_SERIAL.print(" temp_c="); DEBUG_SERIAL.print(_data.temperature_c, 2);
            DEBUG_SERIAL.print(" rel_alt_m="); DEBUG_SERIAL.println(_data.rel_altitude_m, 3);
        }
    }

    uint32_t now = millis();
    if (_prevTime_ms > 0) {
        float dt = (now - _prevTime_ms) * 0.001f;
        if (dt > 0.0f) {
            // Low-pass filtered vertical velocity
            float raw_vel = (_data.rel_altitude_m - _prevRelAlt_m) / dt;
            _data.vert_vel_ms = _data.vert_vel_ms * 0.8f + raw_vel * 0.2f;
        }
    }
    _prevRelAlt_m = _data.rel_altitude_m;
    _prevTime_ms  = now;
    return true;
}

void Barometer::calibrate() {
    // Average pressure (not altitude) over 2s for a stable ground reference --
    // storing pressure avoids depending on a fixed sea-level assumption.
    float sum = 0.0f;
    int   n   = 0;
    uint32_t start = millis();
    while (millis() - start < 2000) {
        if (_bmp.performReading()) {
            sum += _bmp.pressure / 100.0f;  // Pa → hPa
            n++;
        }
        delay(50);
    }
    if (n > 0) _groundPressure_hpa = sum / n;
    _data.rel_altitude_m = 0.0f;
    _data.vert_vel_ms    = 0.0f;
    _prevRelAlt_m        = 0.0f;   // matches rel_altitude_m just zeroed above
    _prevTime_ms         = 0;
}
