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
    bool update();
    const AccelerometerData& data() const { return _data; }

private:
    // ADXL375_I2C_BUS (config.h) passed in constructor -- unlike the other
    // sensors (which take their bus at begin_I2C() time), this library's
    // constructor is where the bus is set, so it has to happen here instead.
    Adafruit_ADXL375 _adxl{1, &ADXL375_I2C_BUS};
    AccelerometerData _data{};
};
