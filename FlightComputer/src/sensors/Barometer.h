#pragma once
#include <Adafruit_BMP3XX.h>

struct BarometerData {
    float pressure_hpa;
    float temperature_c;
    float rel_altitude_m;  // relative to launch site (set by calibrate())
    float vert_vel_ms;     // m/s, derived by differentiation, positive = up
    bool  valid;
};

class Barometer {
public:
    bool begin();
    bool update();
    void calibrate();            // call at launch site to set ground reference
    const BarometerData& data() const { return _data; }

private:
    Adafruit_BMP3XX _bmp;
    BarometerData   _data{};

    float    _groundPressure_hpa = 1013.25f;  // pressure at launch site, set by calibrate()
    float    _prevRelAlt_m       = 0.0f;
    uint32_t _prevTime_ms        = 0;
};
