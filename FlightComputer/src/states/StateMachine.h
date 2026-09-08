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

    // Ground-commanded soft reset (see Packet.h's RESET) -- unconditionally
    // back to IDLE and clears every detection variable, regardless of
    // current state. Unlike onDisarm(), which only transitions if currently
    // ARMED, this always resets -- main.cpp calls it directly, not through
    // PyroController/onDisarm's normal path.
    void reset();

private:
    FlightState   _state          = FlightState::IDLE;
    uint32_t      _stateEnteredMs = 0;

    // Liftoff detection: must hold above threshold for LIFTOFF_CONFIRM_MS
    uint32_t _liftoffFirstMs   = 0;
    bool     _liftoffDetecting = false;

    // Burnout detection: must hold below threshold for BURNOUT_CONFIRM_MS
    uint32_t _burnoutFirstMs   = 0;
    bool     _burnoutDetecting = false;

    // Burnout cross-check via vertical velocity (motor-agnostic fallback,
    // independent of the accelerometer) -- see POWERED_ASCENT_VELOCITY_DROP_MS
    float    _peakAscentVertVel = 0.0f;
    uint32_t _decelFirstMs      = 0;
    bool     _decelDetecting    = false;

    // Apogee detection: sustained non-positive vertical velocity
    uint32_t _apogeeWindowMs   = 0;
    bool     _apogeeDetecting  = false;

    // Landed detection: track altitude stability
    float    _landedRefAlt     = 0.0f;
    uint32_t _landedStableMs   = 0;

    PyroController& _pyro;

    void _enterState(FlightState next, uint32_t now);
};
