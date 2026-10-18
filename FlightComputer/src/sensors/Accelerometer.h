#pragma once
#include <Adafruit_ADXL375.h>
#include "../config.h"

struct AccelerometerData {
    float x_g, y_g, z_g;    // in g (±200g range)
    float magnitude_g;       // sqrt(x²+y²+z²)
    bool  valid;
};

class Accelerometer {
public:
    bool begin();

    // allowBenchDiagnostic gates the blocking delay(15)+re-read bench
    // diagnostic (see update()'s own comment) -- caller passes true only in
    // flight-timing-safe states (SensorManager: IDLE/ARMED/LANDED). Defaults
    // false so any other/future caller doesn't accidentally re-enable a
    // 15ms stall by omission.
    bool update(bool allowBenchDiagnostic = false);
    const AccelerometerData& data() const { return _data; }

private:
    // ADXL375_I2C_BUS (config.h) passed in constructor -- unlike the other
    // sensors (which take their bus at begin_I2C() time), this library's
    // constructor is where the bus is set, so it has to happen here instead.
    Adafruit_ADXL375 _adxl{1, &ADXL375_I2C_BUS};
    AccelerometerData _data{};
};
