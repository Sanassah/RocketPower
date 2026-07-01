#include "Barometer.h"
#include "../config.h"
#include <math.h>

bool Barometer::begin() {
    if (!_bmp.begin_I2C(BMP390_I2C_ADDR, &BMP390_I2C_BUS)) return false;
    _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    _bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    _data.valid = true;
    return true;
}

bool Barometer::update() {
    if (!_bmp.performReading()) return false;

    _data.pressure_hpa  = _bmp.pressure / 100.0f;
    _data.temperature_c = _bmp.temperature;
    _data.altitude_m    = _bmp.readAltitude(SEA_LEVEL_HPA);

    // Relative altitude uses the pressure ratio vs ground pressure.
    // This approach is immune to QNH errors: if today's sea-level pressure
    // differs from SEA_LEVEL_HPA, both altitude_m and groundAlt_m would be
    // wrong by the same constant — but dividing pressures directly cancels
    // that error out entirely.
    _data.rel_altitude_m = 44330.0f * (1.0f - powf(_data.pressure_hpa / _groundPressure_hpa, 0.1903f));

    uint32_t now = millis();
    if (_prevTime_ms > 0) {
        float dt = (now - _prevTime_ms) * 0.001f;
        if (dt > 0.0f) {
            // Low-pass filtered vertical velocity
            float raw_vel = (_data.altitude_m - _prevAlt_m) / dt;
            _data.vert_vel_ms = _data.vert_vel_ms * 0.8f + raw_vel * 0.2f;
        }
    }
    _prevAlt_m   = _data.altitude_m;
    _prevTime_ms = now;
    return true;
}

void Barometer::calibrate() {
    // Average pressure (not altitude) over 2 seconds for a stable ground reference.
    // Storing pressure avoids any dependence on SEA_LEVEL_HPA for relative altitude.
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
    _prevAlt_m           = _data.altitude_m;  // reset velocity differentiator
    _prevTime_ms         = 0;
}
