#include "SensorManager.h"
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

    // IMU and barometer are critical — flight is unsafe without them
    ok = _data.imu_ok && _data.baro_ok;
    return ok;
}

void SensorManager::update() {
    _imu.update();
    _baro.update();
    _accel.update();
    _gps.update();
    _power.update();

    _data.timestamp_ms = millis();

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
}

void SensorManager::calibrateBaro() {
    _baro.calibrate();
}
