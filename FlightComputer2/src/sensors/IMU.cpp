#include "../config.h"
#include "IMU.h"

bool IMU::begin() {
    // Initialize the IMU sensor
    if (!_bno.begin_I2C(BNO085_I2C_ADDR, &BNO085_I2C_BUS)) return false;
    _data.valid = _bno.enableReport(SH2_ROTATION_VECTOR, 10000)
                && _bno.enableReport(SH2_LINEAR_ACCELERATION, 10000) // Gravity removed
                && _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000); // 10000 microseconds = 10ms = 100Hz
    return _data.valid;
}

// Drain all waiting BNO085 reports (orientation, accel, gyro) per cycle to prevent 
// FIFO backup and fin control lag. Bounded to 10 iterations to prevent loop starvation.
bool IMU::update() {
    bool gotNewPacket = false;

    for (uint8_t i = 0; i < 10; i++) {
        if (!_bno.getSensorEvent(&_sensorReport)) break;

        _unpackSensorReport();
        gotNewPacket = true;
    }

    return gotNewPacket;
}

void IMU::_unpackSensorReport() {
    switch (_sensorReport.sensorId) {
        case SH2_ROTATION_VECTOR:
            _data.quat_w = _sensorReport.un.rotationVector.real;
            _data.quat_x = _sensorReport.un.rotationVector.i;
            _data.quat_y = _sensorReport.un.rotationVector.j;
            _data.quat_z = _sensorReport.un.rotationVector.k;
            _data.valid  = true;
            break;

        case SH2_LINEAR_ACCELERATION:
            _data.lin_accel_x = _sensorReport.un.linearAcceleration.x;
            _data.lin_accel_y = _sensorReport.un.linearAcceleration.y;
            _data.lin_accel_z = _sensorReport.un.linearAcceleration.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            _data.gyro_x = _sensorReport.un.gyroscope.x;
            _data.gyro_y = _sensorReport.un.gyroscope.y;
            _data.gyro_z = _sensorReport.un.gyroscope.z;
            break;

        default:
            break;
    }
}