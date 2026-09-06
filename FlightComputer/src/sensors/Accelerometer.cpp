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

    // Bench diagnostic (DEBUG_SERIAL only -- separate USB port from LoRa
    // telemetry, see config.h's DEBUG_SERIAL macro -- never costs airtime).
    {
        static uint32_t lastPrintMs = 0;
        if (millis() - lastPrintMs >= 300) {
            lastPrintMs = millis();
            DEBUG_SERIAL.print("[ACCEL RAW] x_g="); DEBUG_SERIAL.print(_data.x_g, 3);
            DEBUG_SERIAL.print(" y_g="); DEBUG_SERIAL.print(_data.y_g, 3);
            DEBUG_SERIAL.print(" z_g="); DEBUG_SERIAL.print(_data.z_g, 3);
            DEBUG_SERIAL.print(" mag_g="); DEBUG_SERIAL.println(_data.magnitude_g, 3);
        }
    }

    return true;
}
