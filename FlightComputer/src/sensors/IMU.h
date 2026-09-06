#pragma once
#include <Adafruit_BNO08x.h>

struct IMUData {
    float quat_w, quat_x, quat_y, quat_z;
    float lin_accel_x, lin_accel_y, lin_accel_z;  // m/s², gravity removed
    float gyro_x, gyro_y, gyro_z;                 // rad/s
    bool  valid;
};

class IMU {
public:
    bool begin();
    bool update();          // call every loop; returns true if new data arrived
    const IMUData& data() const { return _data; }

private:
    Adafruit_BNO08x _bno;
    sh2_SensorValue_t _event;
    IMUData _data{};

    // Most recent PRE-correction rotation vector, kept only so [IMU RAW]'s
    // bench print can show it -- see IMU.cpp's update()/config.h's
    // IMU_MOUNT_CAL_QUAT_* comment. Recalibrating requires the raw sensor
    // value, not _data.quat_* (which is mounting-offset-corrected downstream
    // of _applyEvent() and would trivially read ~identity once already
    // calibrated).
    float _rawQuatW = 1.0f, _rawQuatX = 0.0f, _rawQuatY = 0.0f, _rawQuatZ = 0.0f;

    void _applyEvent();
};
