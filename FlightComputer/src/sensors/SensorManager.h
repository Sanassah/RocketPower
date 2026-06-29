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

    // Sensor health flags
    bool imu_ok, baro_ok, accel_ok, gps_ok, power_ok;
};

class SensorManager {
public:
    bool begin();              // init all sensors; returns false if any critical sensor fails
    void update();             // read all sensors, populate _data
    void calibrateBaro();      // call at launch site to zero relative altitude
    const FlightData& data() const { return _data; }

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
};
