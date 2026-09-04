#pragma once
#include "../sensors/SensorManager.h"
#include "FinController.h"

// Active fin-based attitude control: a PD controller per axis (roll/pitch/
// yaw), not a pure rate damper. Two independent terms, summed before
// allocation:
//   ANGLE (position): how far the current orientation has drifted from a
//     REFERENCE orientation, expressed as a small-angle quaternion error
//     (see update() for the math). This is what makes it HOLD an attitude
//     -- tilt it and let go, and this term pulls it back -- rather than
//     just resisting active motion.
//   RATE (damping): sensed angular rate, driven toward zero. Keeps the
//     angle term from overshooting/oscillating on the way back.
// The reference orientation isn't a hardcoded "vertical" -- there's no
// verified absolute-orientation convention for this board's IMU mounting.
// Instead it's LATCHED the instant this controller starts actually running
// (the first update() call after main.cpp's state-gating opens, i.e. at
// liftoff for real flight, or at ARM/Enable for demo -- see update()).
//
// A wrong RATE gain only ever under/over-damps -- same as a pure rate
// damper. A wrong ANGLE gain is a different, higher-stakes failure mode: it
// can actively steer AWAY from the reference instead of toward it, which is
// worse than doing nothing. Both need the bench-verification discipline
// below before ever trusting this for a real flight.
//
// Two independent runtime enable flags, deliberately not one, set via
// ground-commanded ATTITUDE_CONTROL_ENABLE/DISABLE and
// ATTITUDE_DEMO_ENABLE/DISABLE packets (see Packet.h + TelemetryManager):
//   controlEnabled() -- the real in-flight engage, gated to
//     POWERED_ASCENT/COAST by main.cpp. OFF at every boot and forced back
//     OFF on DISARM; read the checklist on it in config.h before ever
//     enabling it for a real flight.
//   demoEnabled() -- same control law/gains, gated to IDLE or ARMED (ground-
//     held or on the pad, never in flight) by main.cpp. This is the bench
//     validation check as a standing capability: no need to arm pyro just
//     to hand-rotate the airframe and watch whether the fins actually
//     oppose the rotation -- IDLE alone is enough. Leaving this on can't
//     sneak active control into a real flight -- it stops mattering the
//     instant the state machine moves past ARMED.
//
// ATTITUDE_ROLL_RATE/PITCH_RATE/YAW_RATE and the matching
// ATTITUDE_*_ANGLE_ERR macros (which raw gyro_x/y/z channel / quaternion-
// error component maps to which physical rotation) are UNVERIFIED against
// this board's actual IMU mounting -- there is no documented convention for
// it anywhere else in this codebase. This is exactly what
// ATTITUDE_DEMO_ENABLED is for.
//
// Fin layout (bench-confirmed via testSweep against the physical airframe):
//   CH1 = S, CH2 = E, CH3 = N, CH4 = W
// Control allocation, once rate errors are known (roll/pitch/yaw here
// follow Simulation/RocketPowerSim.slx's convention, NOT the more common
// aerospace one where "roll" is the spin axis -- see the header note above
// config.h's ATTITUDE_ROLL_RATE/etc. macros):
//   Yaw:   all 4 fins deflect the same rotational sense (spin about the
//          rocket's own longitudinal axis)
//   Roll:  N/S differential (CH3 vs CH1) -- a transverse tilt axis
//   Pitch: E/W differential (CH2 vs CH4) -- the other transverse tilt axis
// The channel-to-compass mapping above is confirmed; the +/- signs in that
// allocation are not -- exactly as unverified as the gyro mapping, and
// checked by the same demo-mode bench test (see config.h).
class AttitudeController {
public:
    // Call every loop, but only while there's a reason to -- main.cpp gates
    // this to POWERED_ASCENT/COAST (if controlEnabled()) and/or ARMED (if
    // demoEnabled()). A no-op entirely when both are false.
    void update(const FlightData& d, FinController& fins);

    // Runtime enable/disable, driven by ground-commanded packets. Both
    // default false (safe at every boot) and reset() forces them back to
    // false -- main.cpp calls reset() on DISARM so nothing carries over
    // between sessions.
    void setControlEnabled(bool en) { _controlEnabled = en; }
    void setDemoEnabled(bool en)    { _demoEnabled = en; }
    bool controlEnabled() const     { return _controlEnabled; }
    bool demoEnabled() const        { return _demoEnabled; }
    void reset();

private:
    bool _controlEnabled = false;
    bool _demoEnabled    = false;

    // True from the moment update() first actually runs after being
    // (re-)engaged, until it goes back to fully disabled. Used purely to
    // detect the rising edge where the reference orientation below needs
    // (re-)latching -- see update().
    bool _wasEngaged = false;

    // Reference orientation to hold, latched on the engage edge (see
    // update()). Defaults to identity, but that value is never actually
    // used as a reference -- it's always overwritten before the first real
    // control computation.
    float _refQuatW = 1.0f, _refQuatX = 0.0f, _refQuatY = 0.0f, _refQuatZ = 0.0f;

    // Integral accumulators -- inert while ATTITUDE_*_RATE_KI are 0 (the
    // recommended starting point), but kept so enabling PI later is a
    // config.h change, not a restructure.
    float _rollIntegral  = 0.0f;
    float _pitchIntegral = 0.0f;
    float _yawIntegral   = 0.0f;
    uint32_t _prevUpdateMs = 0;

    static float _clampf(float v, float lo, float hi);
};
