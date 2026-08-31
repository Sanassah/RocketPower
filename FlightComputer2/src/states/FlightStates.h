#pragma once

enum class FlightState : uint8_t {
    IDLE            = 0,  // powered on, sensors running, waiting for arm
    ARMED           = 1,  // arm command received, ready to detect liftoff
    POWERED_ASCENT  = 2,  // motor burning, accel > threshold
    COAST           = 3,  // motor out, coasting to apogee
    APOGEE          = 4,  // apogee confirmed, drogue fired
    DESCENT         = 5,  // descending under drogue
    LANDED          = 6,  // altitude stable, flight over
};