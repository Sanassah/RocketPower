#include "IMU.h"
#include "../config.h"
#include "../DebugPrint.h"
#include <math.h>

// Rotates a body vector (lin_accel/gyro) from the chip's own raw axes into
// the airframe's true axes, using the SAME IMU_MOUNT_CAL_QUAT_* already
// used to correct orientation -- CONFIRMED BUG fix (2026-10): the old code
// combined the CORRECTED quaternion with UNCORRECTED accel/gyro, leaving a
// frame mismatch that leaked sideways motion into the vertical estimate
// and the attitude damping term. Standard quaternion-rotates-a-vector
// formula (same family as SensorManager.cpp's accel_vert Z-row, extended
// to all 3 rows here) -- uses q_cal directly, NOT its conjugate/inverse
// (that's for composing ORIENTATIONS; this is a VECTOR rotation, and
// landing in the same frame the corrected quaternion already describes
// needs the forward q_cal, not the inverse used in _applyEvent() below).
static void rotateByMountCal(float& x, float& y, float& z) {
    constexpr float w  = IMU_MOUNT_CAL_QUAT_W;
    constexpr float qx = IMU_MOUNT_CAL_QUAT_X;
    constexpr float qy = IMU_MOUNT_CAL_QUAT_Y;
    constexpr float qz = IMU_MOUNT_CAL_QUAT_Z;

    float rx = (1.0f - 2.0f*(qy*qy + qz*qz)) * x + 2.0f*(qx*qy - w*qz)       * y + 2.0f*(qx*qz + w*qy)       * z;
    float ry = 2.0f*(qx*qy + w*qz)       * x + (1.0f - 2.0f*(qx*qx + qz*qz)) * y + 2.0f*(qy*qz - w*qx)       * z;
    float rz = 2.0f*(qx*qz - w*qy)       * x + 2.0f*(qy*qz + w*qx)       * y + (1.0f - 2.0f*(qx*qx + qy*qy)) * z;

    x = rx; y = ry; z = rz;
}

bool IMU::begin() {
    // begin_I2C probes the address BEFORE the driver's hardware reset.
    // Ensure NRST is released and the device has time to boot for that probe.
    pinMode(BNO085_RESET_PIN, OUTPUT);
    digitalWrite(BNO085_RESET_PIN, HIGH);
    delay(100);
    if (!_bno.begin_I2C(BNO085_I2C_ADDR, &BNO085_I2C_BUS)) return false;
    _bno.wasReset(); // consume the expected startup-reset notification
    return _enableReports();
}

bool IMU::_enableReports() {
    bool ok = true;
    // Linear accel detuned to 50Hz, off-tick from the other two (100Hz
    // each) -- same tick = queued together = more 32-byte I2C chunks to
    // drain. Accel only feeds the already-filtered velocity fusion, can
    // afford it; rotation vector/gyro feed AttitudeController directly.
    ok &= _bno.enableReport(SH2_ROTATION_VECTOR,     10000);  // 100 Hz
    ok &= _bno.enableReport(SH2_LINEAR_ACCELERATION, 20000);  // 50 Hz, deliberately off-tick from the other two
    ok &= _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);  // 100 Hz
    _data.valid = ok;
    return ok;
}

bool IMU::update() {
    if (_bno.wasReset()) {
        _data.valid = false;
        DEBUG_SERIAL.println("[IMU] Sensor reset detected; restoring reports");
        if (!_enableReports()) {
            DEBUG_SERIAL.println("[IMU] Failed to restore sensor reports");
        }
        // A reset/configuration operation is not a fresh measurement.
        return false;
    }
    if (!_bno.getSensorEvent(&_event)) return false;

    // Captured before _applyEvent() overwrites _event's union with a later
    // gyro/accel event -- only meaningful when sensorId==SH2_ROTATION_VECTOR.
    bool rvThisEvent = (_event.sensorId == SH2_ROTATION_VECTOR);
    uint8_t rvStatus  = rvThisEvent ? (_event.status & 0x03) : 0;
    float   rvAccDeg  = rvThisEvent ? _event.un.rotationVector.accuracy * 57.2958f : 0.0f;

    _applyEvent();

    // Bench diagnostic -- DEBUG_SERIAL only (separate USB port, no airtime cost).
    {
        static uint32_t lastPrintMs = 0;
        if (debugPrintReady(lastPrintMs)) {
            constexpr float kRadToDeg = 57.29577951308232f;

            // Same tilt-from-true-vertical formulas AttitudeController.cpp's
            // angle term and main.cpp's [AXIS CAL] print already use, so
            // this reads consistently with everything else in the project
            // (GroundStation's TEST tab tilt_x_deg/tilt_y_deg included).
            auto eulerDeg = [kRadToDeg](float qw, float qx, float qy, float qz,
                                         float& tiltXdeg, float& tiltYdeg, float& spinZdeg) {
                float sinTiltY = 2.0f * (qw * qy - qz * qx);
                sinTiltY = sinTiltY > 1.0f ? 1.0f : (sinTiltY < -1.0f ? -1.0f : sinTiltY);
                tiltXdeg = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * kRadToDeg;
                tiltYdeg = asinf(sinTiltY) * kRadToDeg;
                spinZdeg = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * kRadToDeg;
            };

            // q= is RAW (pre-correction) -- paste into IMU_MOUNT_CAL_QUAT_*
            // to recalibrate. Kept as a quaternion, not Euler, specifically
            // because that's the exact format IMU_MOUNT_CAL_QUAT_* expects.
            DEBUG_SERIAL.print("[IMU RAW] q=(");
            DEBUG_SERIAL.print(_rawQuatW, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatX, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatY, 4); DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(_rawQuatZ, 4); DEBUG_SERIAL.print(")");

            float rawTiltXdeg, rawTiltYdeg, rawSpinZdeg;
            eulerDeg(_rawQuatW, _rawQuatX, _rawQuatY, _rawQuatZ,
                     rawTiltXdeg, rawTiltYdeg, rawSpinZdeg);
            DEBUG_SERIAL.print(" eulerRaw_deg=(tiltX=");
            DEBUG_SERIAL.print(rawTiltXdeg, 2); DEBUG_SERIAL.print(",tiltY=");
            DEBUG_SERIAL.print(rawTiltYdeg, 2); DEBUG_SERIAL.print(",spin=");
            DEBUG_SERIAL.print(rawSpinZdeg, 2); DEBUG_SERIAL.print(")");

            float corrTiltXdeg, corrTiltYdeg, corrSpinZdeg;
            eulerDeg(_data.quat_w, _data.quat_x, _data.quat_y, _data.quat_z,
                     corrTiltXdeg, corrTiltYdeg, corrSpinZdeg);
            DEBUG_SERIAL.print(" eulerCorrected_deg=(tiltX=");
            DEBUG_SERIAL.print(corrTiltXdeg, 2); DEBUG_SERIAL.print(",tiltY=");
            DEBUG_SERIAL.print(corrTiltYdeg, 2); DEBUG_SERIAL.print(",spin=");
            DEBUG_SERIAL.print(corrSpinZdeg, 2); DEBUG_SERIAL.print(")");

            // Both now mounting-corrected (see rotateByMountCal) -- no
            // longer raw despite this print's [IMU RAW] tag, which refers
            // to the q=(...) field above only.
            DEBUG_SERIAL.print(" gyroCorrected_deg_s=("); DEBUG_SERIAL.print(_data.gyro_x * kRadToDeg, 2);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.gyro_y * kRadToDeg, 2);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.gyro_z * kRadToDeg, 2); DEBUG_SERIAL.print(")");
            DEBUG_SERIAL.print(" linAccelCorrected_ms2=("); DEBUG_SERIAL.print(_data.lin_accel_x, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.lin_accel_y, 3);
            DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(_data.lin_accel_z, 3);
            DEBUG_SERIAL.print(")");
            // rvStatus 0-3 = Unreliable/Low/Medium/High (BNO085's own
            // confidence), only valid the cycle a fresh RV event arrived.
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
            // Mounting-offset fix: q_raw * q_cal^-1 (config.h's
            // IMU_MOUNT_CAL_QUAT_*). Reading == q_cal collapses to identity;
            // a real tilt still measures correctly (fixed frame correction,
            // not a per-reading approximation).
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
            rotateByMountCal(_data.lin_accel_x, _data.lin_accel_y, _data.lin_accel_z);
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            _data.gyro_x = _event.un.gyroscope.x;
            _data.gyro_y = _event.un.gyroscope.y;
            _data.gyro_z = _event.un.gyroscope.z;
            rotateByMountCal(_data.gyro_x, _data.gyro_y, _data.gyro_z);
            break;

        default:
            break;
    }
}
