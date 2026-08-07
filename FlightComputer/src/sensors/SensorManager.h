#pragma once
#include "../states/FlightStates.h"
#include "IMU.h"
#include "Barometer.h"
#include "Accelerometer.h"
#include "GPS.h"
#include "PowerMonitor.h"

// Unified snapshot of all sensor readings passed through the system
struct FlightData {
    uint32_t timestamp_ms;
    FlightState state;

    // IMU (BNO085 — Wire)
    float quat_w, quat_x, quat_y, quat_z;
    float lin_accel_x, lin_accel_y, lin_accel_z;  // m/s²
    float gyro_x, gyro_y, gyro_z;                 // rad/s

    // Barometer (BMP390 — Wire1)
    float pressure_hpa;
    float temperature_c;
    float baro_alt_m;    // relative to launch site
    float vert_vel_ms;   // positive = up

    // High-g accelerometer (ADXL375 — Wire2)
    float highg_x_g, highg_y_g, highg_z_g;
    float highg_mag_g;

    // GPS (ZOEM8 — Wire1 I2C DDC)
    double  lat;
    double  lon;
    float   gps_alt_m;
    uint8_t gps_sats;
    bool    gps_fix;

    // Power (INA260 — Wire1)
    float voltage_v;
    float current_ma;
    float power_mw;

    // Sensor health, re-derived every loop from SENSOR_HEALTH_TIMEOUT_MS
    // recency (see SensorManager::update) -- not just a one-time boot check.
    bool imu_ok, baro_ok, accel_ok, gps_ok, power_ok;
};

class SensorManager {
public:
    bool begin();              // init all sensors; returns false if any critical sensor fails
    void update();             // read all sensors, populate _data
    void calibrateBaro();      // call at launch site to zero relative altitude
    const FlightData& data() const { return _data; }

    // StateMachine owns the authoritative flight state; main.cpp stamps it onto
    // this loop's data snapshot so logging/telemetry can see it alongside the
    // sensor readings it was captured with.
    void setState(FlightState s) { _data.state = s; }

    // Individual sensor access for diagnostics
    IMU&          imu()   { return _imu; }
    Barometer&    baro()  { return _baro; }
    Accelerometer& accel() { return _accel; }
    GPS&          gps()   { return _gps; }
    PowerMonitor& power() { return _power; }

private:
    IMU           _imu;
    Barometer     _baro;
    Accelerometer _accel;
    GPS           _gps;
    PowerMonitor  _power;
    FlightData    _data{};

    // Complementary filter state for fused vertical velocity
    float    _fusedVel_ms     = 0.0f;
    float    _prevBaroAlt_m   = 0.0f;
    uint32_t _prevFuseTime_ms = 0;

    // Timestamp of each sensor's last successful read. A sensor's _ok flag is
    // (now - this) < SENSOR_HEALTH_TIMEOUT_MS. Left at 0 if the sensor never
    // passed begin() in the first place, which reads as "unhealthy" from
    // millis()==SENSOR_HEALTH_TIMEOUT_MS onward -- i.e. correct well before
    // any real pad procedure would arm, but technically not from t=0.
    uint32_t _lastImuOkMs   = 0;
    uint32_t _lastBaroOkMs  = 0;
    uint32_t _lastAccelOkMs = 0;
    uint32_t _lastGpsOkMs   = 0;
    uint32_t _lastPowerOkMs = 0;
};
