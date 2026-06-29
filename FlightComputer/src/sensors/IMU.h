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

    void _applyEvent();
};
