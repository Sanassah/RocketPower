#include "Accelerometer.h"
#include "../config.h"
#include "../DebugPrint.h"
#include <math.h>

bool Accelerometer::begin() {
    ADXL375_I2C_BUS.begin();   // ensure the bus is up before ADXL375 init
    if (!_adxl.begin(ADXL375_I2C_ADDR)) return false;
    _adxl.setDataRate(ADXL3XX_DATARATE_100_HZ);
    _data.valid = true;

    // Bench diagnostic, kept for the ongoing hardware investigation (solder
    // joint / sensor swap check) -- rules out SELF_TEST (DATA_FORMAT bit 7)
    // being stuck set. Bandwidth sweep (1.56Hz-3200Hz) already ruled out
    // ODR-scaled noise as the cause -- fault is hardware-layer, not noise.
    uint8_t fmt = _adxl.readRegister(ADXL3XX_REG_DATA_FORMAT);
    DEBUG_SERIAL.print("[ACCEL INIT] DATA_FORMAT=0b");
    for (int8_t i = 7; i >= 0; i--) DEBUG_SERIAL.print((fmt >> i) & 1);
    DEBUG_SERIAL.print(" SELF_TEST="); DEBUG_SERIAL.println((fmt & 0x80) ? "SET (BAD)" : "clear (ok)");

    return true;
}

bool Accelerometer::update(bool allowBenchDiagnostic) {
    // getXYZ() (NOT getEvent(), which this used to call) reads all 3 axes
    // in one atomic 6-byte I2C burst. getEvent() instead calls getX()/getY()/
    // getZ() as 3 SEPARATE I2C reads -- if the chip's internal registers
    // update in between (its ODR is 100Hz, this gets polled at a similar,
    // jittery rate), you get axes from different instants stitched into one
    // physically-impossible vector. Bench-confirmed root cause of wildly
    // noisy readings at rest (0.3-4.7g magnitude, should be a steady ~1.0g).
    int16_t rawX, rawY, rawZ;
    if (!_adxl.getXYZ(rawX, rawY, rawZ)) return false;   // I2C read failed; leave last-known values in place
    _data.x_g = rawX * ADXL375_MG2G_MULTIPLIER;
    _data.y_g = rawY * ADXL375_MG2G_MULTIPLIER;
    _data.z_g = rawZ * ADXL375_MG2G_MULTIPLIER;
    _data.magnitude_g = sqrtf(_data.x_g * _data.x_g +
                               _data.y_g * _data.y_g +
                               _data.z_g * _data.z_g);

    // Bench diagnostic (DEBUG_SERIAL only -- separate USB port from LoRa
    // telemetry, see config.h's DEBUG_SERIAL macro -- never costs airtime).
    // CONFIRMED BUG (2026-10): rate-limiting this to ~every 300ms (instead
    // of every loop, the original 2026-09 bug) was NOT enough -- it still
    // blocks for delay(15) once every ~30 loops, unconditionally, including
    // during POWERED_ASCENT/COAST/APOGEE/DESCENT. 15ms exceeds the entire
    // intended 10ms loop period by itself, stalling AttitudeController's
    // control update and StateMachine/BackupDeploy's apogee/deployment
    // checks whenever it fires. allowBenchDiagnostic (SensorManager: true
    // only in IDLE/ARMED/LANDED) is the actual fix -- this block now cannot
    // run at all once READY_FOR_FLIGHT is reached, regardless of timer.
    if (allowBenchDiagnostic) {
        static uint32_t lastPrintMs = 0;
        if (debugPrintReady(lastPrintMs)) {
            delay(15);
            int16_t rawX2, rawY2, rawZ2;
            bool got2 = _adxl.getXYZ(rawX2, rawY2, rawZ2);

            DEBUG_SERIAL.print("[ACCEL RAW] x_g="); DEBUG_SERIAL.print(_data.x_g, 3);
            DEBUG_SERIAL.print(" y_g="); DEBUG_SERIAL.print(_data.y_g, 3);
            DEBUG_SERIAL.print(" z_g="); DEBUG_SERIAL.print(_data.z_g, 3);
            DEBUG_SERIAL.print(" mag_g="); DEBUG_SERIAL.print(_data.magnitude_g, 3);
            if (got2) {
                float x2 = rawX2 * ADXL375_MG2G_MULTIPLIER;
                float y2 = rawY2 * ADXL375_MG2G_MULTIPLIER;
                float z2 = rawZ2 * ADXL375_MG2G_MULTIPLIER;
                DEBUG_SERIAL.print(" | immediate re-read: x_g="); DEBUG_SERIAL.print(x2, 3);
                DEBUG_SERIAL.print(" y_g="); DEBUG_SERIAL.print(y2, 3);
                DEBUG_SERIAL.print(" z_g="); DEBUG_SERIAL.print(z2, 3);
            } else {
                DEBUG_SERIAL.print(" | immediate re-read FAILED");
            }
            DEBUG_SERIAL.println();
        }
    }

    return true;
}
