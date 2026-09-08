#pragma once
#include <stdint.h>

enum class FlightState : uint8_t {
    IDLE            = 0,  // powered on, sensors running, waiting for arm
    ARMED           = 1,  // arm command received, ready to detect liftoff
    POWERED_ASCENT  = 2,  // motor burning, accel > threshold
    COAST           = 3,  // motor out, coasting to apogee
    APOGEE          = 4,  // apogee confirmed, parachute fired
    DESCENT         = 5,  // descending under parachute
    LANDED          = 6,  // altitude stable, flight over
};

inline const char* flightStateName(FlightState s) {
    switch (s) {
        case FlightState::IDLE:           return "IDLE";
        case FlightState::ARMED:          return "ARMED";
        case FlightState::POWERED_ASCENT: return "POWERED_ASCENT";
        case FlightState::COAST:          return "COAST";
        case FlightState::APOGEE:         return "APOGEE";
        case FlightState::DESCENT:        return "DESCENT";
        case FlightState::LANDED:         return "LANDED";
        default:                          return "UNKNOWN";
    }
}
