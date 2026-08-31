#pragma once

struct BarometerData {
    float pressure_hpa;
    float temperature_c;
    float baro_alt_m;    // relative to launch site
    float vert_vel_ms;   // positive = up
};