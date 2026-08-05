#pragma once
#include "../sensors/SensorManager.h"

// Drives the 4 sensor-status LEDs on the schematic (D3 GPS, D4 Altimeter,
// D5 200G ACC, D6 IMU): solid on while that sensor is healthy, flashing at
// ~2 Hz while it isn't. GPS additionally requires a satellite fix, not just
// a responding module, before it's considered "ok".
class StatusLED {
public:
    void begin();
    void update(const FlightData& d);
};
