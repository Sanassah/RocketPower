#pragma once

struct PowerMonitorData {
    float voltage; // Voltage in volts
    float current; // Current in amperes
    float power;   // Power in milli watts
    bool valid;   // True if the readings are valid, false otherwise
};