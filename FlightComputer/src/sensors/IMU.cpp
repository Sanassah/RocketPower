#include "IMU.h"
#include "../config.h"

bool IMU::begin() {
    if (!_bno.begin_I2C(BNO085_I2C_ADDR, &BNO085_I2C_BUS)) return false;
    bool ok = true;
    ok &= _bno.enableReport(SH2_ROTATION_VECTOR,    10000);  // 100 Hz
    ok &= _bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    ok &= _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    _data.valid = ok;
    return ok;
}

bool IMU::update() {
    if (!_bno.getSensorEvent(&_event)) return false;
    _applyEvent();
    return true;
}

void IMU::_applyEvent() {
    switch (_event.sensorId) {
        case SH2_ROTATION_VECTOR:
            _data.quat_w = _event.un.rotationVector.real;
            _data.quat_x = _event.un.rotationVector.i;
            _data.quat_y = _event.un.rotationVector.j;
            _data.quat_z = _event.un.rotationVector.k;
            _data.valid  = true;
            break;

        case SH2_LINEAR_ACCELERATION:
            _data.lin_accel_x = _event.un.linearAcceleration.x;
            _data.lin_accel_y = _event.un.linearAcceleration.y;
            _data.lin_accel_z = _event.un.linearAcceleration.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            _data.gyro_x = _event.un.gyroscope.x;
            _data.gyro_y = _event.un.gyroscope.y;
            _data.gyro_z = _event.un.gyroscope.z;
            break;

        default:
            break;
    }
}
