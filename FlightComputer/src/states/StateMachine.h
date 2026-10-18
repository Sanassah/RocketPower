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
    void onReady();
    void onDisarm();

    // Clear flight history. Telemetry command policy permits this only
    // in IDLE/ARMED/LANDED; never in READY or an airborne state.
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

    // Second, independent apogee check: sustained raw baro_alt_m drop from
    // peak, no vert_vel_ms involved -- see BARO_APOGEE_DROP_M in config.h.
    // _peakAltM resets at every _enterState() (same as _peakAscentVertVel),
    // so it effectively starts tracking at COAST entry -- harmless even
    // though COAST begins before the real apogee, since altitude keeps
    // rising monotonically through the rest of the climb either way.
    float    _peakAltM            = 0.0f;
    uint32_t _baroDescendFirstMs  = 0;
    bool     _baroDescendDetecting = false;

    // _peakAltM itself must be debounced the same way everything else in
    // this file is -- a single RF-glitched high baro_alt_m sample must NOT
    // be allowed to instantly and permanently inflate the peak (that would
    // make _peakAltM - BARO_APOGEE_DROP_M look "descended past" on every
    // later genuine reading, even mid-ascent with positive vert_vel_ms).
    // A candidate must stay above the CURRENT confirmed _peakAltM for the
    // full BARO_PEAK_CONFIRM_MS before promotion -- reset after every
    // promotion (not just once at COAST entry), so a later glitch can't
    // exploit an already-latched "still above peak" flag and get promoted
    // instantly. Promotion uses the window's minimum sample, not whatever
    // is live at the instant the window closes, so a glitch landing on
    // that exact tick can't sneak through either.
    float    _peakCandidateMinM      = 0.0f;
    uint32_t _peakCandidateFirstMs   = 0;
    bool     _peakCandidateDetecting = false;

    // Landed detection: track altitude stability
    float    _landedRefAlt     = 0.0f;
    uint32_t _landedStableMs   = 0;

    PyroController& _pyro;

    void _enterState(FlightState next, uint32_t now);
};
