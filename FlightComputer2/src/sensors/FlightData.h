#pragma once
#include "IMU.h"
#include "Accelerometer.h"
#include "Barometer.h"
#include "GPS.h"
#include "PowerMonitor.h"
#include "states/FlightStates.h"

struct FlightData {
    IMUData imu;
    AccelerometerData accel;
    BarometerData baro;
    GPSdata gps;
    PowerMonitorData power;
    FlightState state; 
    uint32_t timestamp_ms; // Timestamp of the flight data
    float vert_vel_ms; // Vertical velocity in meters per second (IMU x Barometer)
};