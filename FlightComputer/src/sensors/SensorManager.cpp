#include "SensorManager.h"
#include "../config.h"
#include <Wire.h>

bool SensorManager::begin() {
    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    bool ok = true;

    _data.imu_ok   = _imu.begin();
    _data.baro_ok  = _baro.begin();
    _data.accel_ok = _accel.begin();
    _data.gps_ok   = _gps.begin();
    _data.power_ok = _power.begin();

    // Seed each sensor's "last known good" clock so update() has a baseline
    // to measure staleness from. A sensor that failed begin() keeps its
    // timestamp at 0, which reads as "unhealthy" from the very first update().
    uint32_t now = millis();
    if (_data.imu_ok)   _lastImuOkMs   = now;
    if (_data.baro_ok)  _lastBaroOkMs  = now;
    if (_data.accel_ok) _lastAccelOkMs = now;
    if (_data.gps_ok)   _lastGpsOkMs   = now;
    if (_data.power_ok) _lastPowerOkMs = now;

    // IMU and barometer are critical — flight is unsafe without them
    ok = _data.imu_ok && _data.baro_ok;
    return ok;
}

void SensorManager::update() {
    uint32_t now = millis();

    // The BNO085 streams 3 report types (rotation vector, linear accel,
    // gyro) at 100Hz each -- up to 300 events/s -- but this loop only runs
    // at 100Hz itself. Pulling just one event per call can't keep up (it
    // drains at most 1/3 of what's arriving), so the FIFO backs up and
    // whichever report type loses the round-robin sees a growing lag before
    // its value updates -- exactly what shows up as "the fins react late."
    // Drain everything actually queued each call instead, bounded so a
    // pathological flood can't stall the rest of the loop.
    bool imuGotData = false;
    for (uint8_t i = 0; i < 10 && _imu.update(); i++) imuGotData = true;
    if (imuGotData) _lastImuOkMs = now;
    if (_baro.update())  _lastBaroOkMs  = now;
    if (_accel.update()) _lastAccelOkMs = now;
    if (_gps.update())   _lastGpsOkMs   = now;
    if (_power.update()) _lastPowerOkMs = now;

    // "OK" means a successful read within the last SENSOR_HEALTH_TIMEOUT_MS,
    // not just that the sensor passed its boot check.
    _data.imu_ok   = (now - _lastImuOkMs)   < SENSOR_HEALTH_TIMEOUT_MS;
    _data.baro_ok  = (now - _lastBaroOkMs)  < SENSOR_HEALTH_TIMEOUT_MS;
    _data.accel_ok = (now - _lastAccelOkMs) < SENSOR_HEALTH_TIMEOUT_MS;
    _data.gps_ok   = (now - _lastGpsOkMs)   < SENSOR_HEALTH_TIMEOUT_MS;
    _data.power_ok = (now - _lastPowerOkMs) < SENSOR_HEALTH_TIMEOUT_MS;

    _data.timestamp_ms = now;

    // IMU
    const auto& imu = _imu.data();
    _data.quat_w       = imu.quat_w;
    _data.quat_x       = imu.quat_x;
    _data.quat_y       = imu.quat_y;
    _data.quat_z       = imu.quat_z;
    _data.lin_accel_x  = imu.lin_accel_x;
    _data.lin_accel_y  = imu.lin_accel_y;
    _data.lin_accel_z  = imu.lin_accel_z;
    _data.gyro_x       = imu.gyro_x;
    _data.gyro_y       = imu.gyro_y;
    _data.gyro_z       = imu.gyro_z;

    // Barometer
    const auto& baro = _baro.data();
    _data.pressure_hpa = baro.pressure_hpa;
    _data.temperature_c = baro.temperature_c;
    _data.baro_alt_m   = baro.rel_altitude_m;
    _data.vert_vel_ms  = baro.vert_vel_ms;

    // High-g accelerometer
    const auto& accel = _accel.data();
    _data.highg_x_g  = accel.x_g;
    _data.highg_y_g  = accel.y_g;
    _data.highg_z_g  = accel.z_g;
    _data.highg_mag_g = accel.magnitude_g;

    // GPS
    const auto& gps = _gps.data();
    _data.lat       = gps.lat;
    _data.lon       = gps.lon;
    _data.gps_alt_m = gps.alt_m;
    _data.gps_sats  = gps.sats;
    _data.gps_fix   = gps.fix;

    // Power
    const auto& pwr = _power.data();
    _data.voltage_v  = pwr.voltage_v;
    _data.current_ma = pwr.current_ma;
    _data.power_mw   = pwr.power_mw;

    // Fused vertical velocity: complementary filter combining IMU and barometer.
    // The IMU gives fast, low-noise dynamics; the baro corrects long-term drift.
    // Falls back to barometer-only (already in _data.vert_vel_ms) if IMU is absent.
    if (_data.imu_ok && _prevFuseTime_ms > 0) {
        float dt = (_data.timestamp_ms - _prevFuseTime_ms) * 0.001f;
        if (dt > 0.0f && dt < 0.1f) {
            // Rotate body-frame linear acceleration to world-frame vertical (Z-up)
            // using the BNO085 quaternion. Row 3 of the rotation matrix:
            float qw = _data.quat_w, qx = _data.quat_x;
            float qy = _data.quat_y, qz = _data.quat_z;
            float accel_vert = 2.0f*(qx*qz - qw*qy) * _data.lin_accel_x
                             + 2.0f*(qy*qz + qw*qx) * _data.lin_accel_y
                             + (1.0f - 2.0f*(qx*qx + qy*qy)) * _data.lin_accel_z;

            float baro_vel = (_data.baro_alt_m - _prevBaroAlt_m) / dt;
            _fusedVel_ms   = VERT_VEL_ALPHA * (_fusedVel_ms + accel_vert * dt)
                           + (1.0f - VERT_VEL_ALPHA) * baro_vel;
            _data.vert_vel_ms = _fusedVel_ms;
        }
    }
    _prevBaroAlt_m   = _data.baro_alt_m;
    _prevFuseTime_ms = _data.timestamp_ms;
}

void SensorManager::calibrateBaro() {
    _baro.calibrate();
    _fusedVel_ms     = 0.0f;
    _prevFuseTime_ms = 0;     // forces a one-cycle skip so no spurious velocity spike
}
