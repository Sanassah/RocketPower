#pragma once
#include <Adafruit_ADXL375.h>

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
    // Wire2 passed in constructor (SCL2/SDA2 from schematic)
    Adafruit_ADXL375 _adxl{1, &Wire2};
    AccelerometerData _data{};
};
