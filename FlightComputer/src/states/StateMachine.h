#pragma once
#include "FlightStates.h"
#include "../sensors/SensorManager.h"

// Forward declaration — PyroController included in .cpp to avoid circular deps
class PyroController;

class StateMachine {
public:
    explicit StateMachine(PyroController& pyro) : _pyro(pyro) {}

    void update(const FlightData& d);

    FlightState state() const { return _state; }
    uint32_t    stateEnteredMs() const { return _stateEnteredMs; }

    // Called by TelemetryManager when ARM/DISARM command arrives
    void onArm();
    void onDisarm();

private:
    FlightState   _state          = FlightState::IDLE;
    uint32_t      _stateEnteredMs = 0;

    // Liftoff detection: must hold above threshold for LIFTOFF_CONFIRM_MS
    uint32_t _liftoffFirstMs   = 0;
    bool     _liftoffDetecting = false;

    // Apogee detection: track sign change in vertical velocity
    float    _prevVertVel      = 0.0f;
    uint32_t _apogeeWindowMs   = 0;
    bool     _apogeeDetecting  = false;

    // Landed detection: track altitude stability
    float    _landedRefAlt     = 0.0f;
    uint32_t _landedStableMs   = 0;

    PyroController& _pyro;

    void _enterState(FlightState next, uint32_t now);
};
