#include "Accelerometer.h"
#include "../config.h"
#include <math.h>

bool Accelerometer::begin() {
    Wire2.begin();   // ensure Wire2 is up before ADXL375 init
    if (!_adxl.begin(ADXL375_I2C_ADDR)) return false;
    _adxl.setDataRate(ADXL3XX_DATARATE_100_HZ);
    _data.valid = true;
    return true;
}

bool Accelerometer::update() {
    sensors_event_t e;
    if (!_adxl.getEvent(&e)) return false;   // I2C read failed; leave last-known values in place
    _data.x_g = e.acceleration.x / SENSORS_GRAVITY_STANDARD;
    _data.y_g = e.acceleration.y / SENSORS_GRAVITY_STANDARD;
    _data.z_g = e.acceleration.z / SENSORS_GRAVITY_STANDARD;
    _data.magnitude_g = sqrtf(_data.x_g * _data.x_g +
                               _data.y_g * _data.y_g +
                               _data.z_g * _data.z_g);
    return true;
}
