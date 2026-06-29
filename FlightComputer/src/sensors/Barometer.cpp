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
    _data.rel_altitude_m = _data.altitude_m - _groundAlt_m;

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
    // Average several readings to get a stable ground reference
    float sum = 0.0f;
    int   n   = 0;
    uint32_t start = millis();
    while (millis() - start < 2000) {
        if (_bmp.performReading()) {
            sum += _bmp.readAltitude(SEA_LEVEL_HPA);
            n++;
        }
        delay(50);
    }
    if (n > 0) _groundAlt_m = sum / n;
    _data.rel_altitude_m = 0.0f;
    _data.vert_vel_ms    = 0.0f;
}
