#include "IMU.h"
#include "../config.h"

bool IMU::begin() {
    if (!_bno.begin_I2C(BNO085_I2C_ADDR, &BNO085_I2C_BUS)) return false;
    bool ok = true;
    // BENCH-MEASURED (see SensorManager.cpp's [SENSOR BREAKDOWN] diagnostic):
    // all three reports were scheduled on the identical 10000us tick, which
    // means the BNO085 hub tends to have all three queued at once when the
    // host polls -- traced through the vendor driver (Adafruit_BNO08x.cpp's
    // i2chal_read()) to a real cost: a bigger queued payload needs more
    // 32-byte I2C chunk reads to drain (Teensy's Adafruit_I2CDevice caps
    // each chunk at 32 bytes -- see Adafruit_I2CDevice.cpp), and each chunk
    // is its own I2C transaction. Linear accel only feeds SensorManager's
    // vertical-velocity complementary filter (already low-pass filtered,
    // see VERT_VEL_ALPHA), unlike rotation vector/gyro which AttitudeController
    // reads directly -- so it's the one that can afford a slower, off-tick
    // rate: halved to 50Hz AND detuned off the other two's exact tick, so
    // it stops reliably landing in the same queued delivery as them.
    ok &= _bno.enableReport(SH2_ROTATION_VECTOR,     10000);  // 100 Hz
    ok &= _bno.enableReport(SH2_LINEAR_ACCELERATION, 20000);  // 50 Hz, deliberately off-tick from the other two
    ok &= _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);  // 100 Hz
    _data.valid = ok;
    return ok;
}

bool IMU::update() {
    if (!_bno.getSensorEvent(&_event)) return false;

    // Capture the rotation vector's own confidence BEFORE _applyEvent() lets
    // a later event (gyro/linAccel) overwrite _event's union -- status/accuracy
    // are only meaningful when _event.sensorId == SH2_ROTATION_VECTOR.
    bool rvThisEvent = (_event.sensorId == SH2_ROTATION_VECTOR);
    uint8_t rvStatus  = rvThisEvent ? (_event.status & 0x03) : 0;
    float   rvAccDeg  = rvThisEvent ? _event.un.rotationVector.accuracy * 57.2958f : 0.0f;

    _applyEvent();

    // Bench diagnostic (DEBUG_SERIAL only -- see config.h's DEBUG_SERIAL
    // macro -- physically a separate USB port from LoRa telemetry, so this
    // never costs airtime or competes with the binary link either way).
    {
        static uint32_t lastPrintMs = 0;
        if (millis() - lastPrintMs >= 300) {
            lastPrintMs = millis();
            // q=(...) is the RAW, pre-mounting-correction quaternion straight
            // off the sensor -- this is what config.h's "TO RECALIBRATE" note
            // wants pasted into IMU_MOUNT_CAL_QUAT_*, not qCorrected (which
            // will already read ~identity once calibrated, and so is useless
            // as a new calibration reference).
            DEBUG_SERIAL.print("[IMU RAW] q=(");
            DEBUG_SERIAL.print(_rawQuatW, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatX, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatY, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatZ, 4); DEBUG_SERIAL.print(")");
            DEBUG_SERIAL.print(" qCorrected=(");
            DEBUG_SERIAL.print(_data.quat_w, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_data.quat_x, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_data.quat_y, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_data.quat_z, 4); DEBUG_SERIAL.print(")");
            DEBUG_SERIAL.print(" gyro_rad_s=("); DEBUG_SERIAL.print(_data.gyro_x, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.gyro_y, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.gyro_z, 3); DEBUG_SERIAL.print(")");
            DEBUG_SERIAL.print(" linAccel_ms2=("); DEBUG_SERIAL.print(_data.lin_accel_x, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.lin_accel_y, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.lin_accel_z, 3);
            DEBUG_SERIAL.print(")");
            // rvStatus: 0=Unreliable 1=Low 2=Medium 3=High -- the BNO085's own
            // confidence in the rotation vector, only printed on the loop
            // iteration that actually delivered a fresh rotation vector event
            // (this report is scheduled every ~10ms, so it prints often).
            if (rvThisEvent) {
                DEBUG_SERIAL.print(" rvStatus="); DEBUG_SERIAL.print(rvStatus);
                DEBUG_SERIAL.print(" rvAccuracyDeg="); DEBUG_SERIAL.print(rvAccDeg, 2);
            }
            DEBUG_SERIAL.println();
        }
    }

    return true;
}

void IMU::_applyEvent() {
    switch (_event.sensorId) {
        case SH2_ROTATION_VECTOR: {
            // Correct the fixed mounting-offset bias -- see config.h's
            // IMU_MOUNT_CAL_QUAT_* comment for the full derivation. This is
            // q_raw (* q_cal^-1), where q_cal is the raw reading captured
            // with the airframe verified vertical: a reading identical to
            // q_cal collapses to identity (tilt=0), while a real tilt is
            // still measured correctly since this is a fixed frame
            // correction, not a per-reading approximation.
            float rw = _event.un.rotationVector.real;
            float rx = _event.un.rotationVector.i;
            float ry = _event.un.rotationVector.j;
            float rz = _event.un.rotationVector.k;
            _rawQuatW = rw; _rawQuatX = rx; _rawQuatY = ry; _rawQuatZ = rz;

            // q_cal^-1 == conjugate(q_cal), since q_cal is (near-)unit length.
            constexpr float cw =  IMU_MOUNT_CAL_QUAT_W;
            constexpr float cx = -IMU_MOUNT_CAL_QUAT_X;
            constexpr float cy = -IMU_MOUNT_CAL_QUAT_Y;
            constexpr float cz = -IMU_MOUNT_CAL_QUAT_Z;

            _data.quat_w = rw * cw - rx * cx - ry * cy - rz * cz;
            _data.quat_x = rw * cx + rx * cw + ry * cz - rz * cy;
            _data.quat_y = rw * cy - rx * cz + ry * cw + rz * cx;
            _data.quat_z = rw * cz + rx * cy - ry * cx + rz * cw;
            _data.valid  = true;
            break;
        }

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
