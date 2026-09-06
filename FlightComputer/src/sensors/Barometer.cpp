#include "Barometer.h"
#include "../config.h"
#include <math.h>

bool Barometer::begin() {
    if (!_bmp.begin_I2C(BMP390_I2C_ADDR, &BMP390_I2C_BUS)) return false;
    // Temp oversampling at 4X, not 8X -- 8X + PRESS_4X together need ~24.9ms
    // per conversion (Bosch's own validate_osr_and_odr_settings: 313+8*2000
    // (temp) + 392+4*2000 (press) + 234 (fixed) = 24939us), but the 50Hz ODR
    // below only allows 20ms between conversions -- enableNormalMode() would
    // fail with BMP3_E_INVALID_ODR_OSR_SETTINGS (-3) on that combination
    // (bench-confirmed). 4X+4X needs ~16.9ms, comfortably inside budget.
    // Temperature accuracy matters far less here than pressure (altitude
    // derives from pressure, temperature is only a compensation input), so
    // it's the one reduced.
    _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    _bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // BENCH-MEASURED: the library's default forced (one-shot) mode re-triggers
    // a fresh conversion AND pays a ~5ms sleep-transition delay on every
    // single performReading() call -- ~13-19ms/call here, every loop, forever.
    // Normal mode configures the sensor to sample continuously in the
    // background at the ODR set above; performReading() then just fetches
    // the latest ready value (bench-measured well under 1ms). See the
    // RocketPower-forked lib/Adafruit BMP3XX Library/ (not the
    // lib_deps-fetched copy in .pio/libdeps, which gets wiped on a clean
    // rebuild) for enableNormalMode()'s full reasoning.
    //
    // Return value checked: if this ever fails for any reason, begin() must
    // report it, not silently claim success while every future
    // performReading() fails the same way.
    if (!_bmp.enableNormalMode()) return false;
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

    // Bench diagnostic (kept deliberately, not temporary). ROOT CAUSE FOUND:
    // rel_altitude_m was swinging several meters, well past this sensor's
    // rated ~0.1m noise floor, identically under BOTH the normal-mode fork
    // AND stock Adafruit forced mode (ruling out our BMP3XX usage) -- then
    // confirmed BENCH-CONFIRMED to be the LoRa (E22) module's transmit
    // current draw coupling noise into the barometer's shared 3.3V rail:
    // disabling LoRa TX entirely made it rock-steady. Real fix is hardware
    // (bulk capacitance at the LoRa module's supply -- the existing 0.1uF
    // caps on this rail are sized for fast transients, not for sustaining a
    // draw over a whole ~217ms transmission), planned for a v2 board.
    // Software workarounds were tried and deliberately reverted (skipping
    // baro reads during a known TX window fixed the symptom but risked
    // biasing SensorManager's IMU/baro velocity fusion during powered
    // ascent, which does matter for apogee detection). This print is kept
    // as the ongoing way to tell a real pressure event (bmp.pressure
    // genuinely moves) apart from a calibration/math issue
    // (groundPressure_hpa or the ratio itself misbehaving instead), until
    // the hardware fix lands and this can be confirmed resolved.
    {
        static uint32_t lastPrintMs = 0;
        if (millis() - lastPrintMs >= 300) {
            lastPrintMs = millis();
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
