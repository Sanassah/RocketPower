#pragma once

struct IMUData {
    float quat_w, quat_x, quat_y, quat_z; // Quaternion components 
    float lin_accel_x, lin_accel_y, lin_accel_z; // Accelerometer readings in m/s²
    float gyro_x, gyro_y, gyro_z; // Gyroscope readings in rad/s
    bool valid;   // True if the readings are valid, false otherwise
};