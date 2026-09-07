#pragma once
#include "../sensors/SensorManager.h"
#include "../control/PyroController.h"

// Independent, state-machine-agnostic backup deployment monitor.
//
// Why this exists: StateMachine's flight state and PyroController's armed
// flag both live in RAM only, with no persistence across a reset. If the
// flight computer resets mid-flight (vibration-induced brownout on a custom
// board is a real risk, ESD, a firmware hang followed by a reset), it boots
// back into IDLE + disarmed -- and since IDLE only ever exits via a
// ground-commanded ARM, the state machine can never reach APOGEE again on
// its own. The primary parachute deployment (StateMachine::update()'s
// APOGEE case) then never fires, and the only remaining recovery path is a
// ground operator noticing the telemetry dropout and manually firing the
// backup channel -- during the exact window telemetry has, by definition,
// just been interrupted.
//
// This class is a second, completely independent detector. It runs every
// loop regardless of StateMachine's state or PyroController's armed flag,
// watching raw sensor data directly for the same physical signature a real
// flight produces (sustained high acceleration, then a real altitude gain,
// then a vertical-velocity zero-crossing) -- and if it sees that, it fires
// the backup channel through PyroController::fireBackupUnconditional(),
// which deliberately bypasses the normal arm check. This class's own
// multi-stage evidence requirement IS the safety gate for that path -- the
// same principle commercial dual-deploy altimeters use for their backup
// channel: it fires a few seconds after apogee, arm state or primary
// deployment success be damned, because a redundant fire into an
// already-open bay is harmless and a missed primary is not.
//
// Deliberately conservative: requires real sustained accel (not a bump),
// requires real altitude gain (not just a G-spike from being carried or
// dropped on the bench), requires a confirmed velocity zero-crossing, AND
// waits an extra delay after that before firing -- giving the primary path
// (and a ground operator watching telemetry) every chance to act first.
// Fires at most once per boot.
class BackupDeploy {
public:
    // Call every loop, unconditionally -- no state or armed-flag gating on
    // the caller's side. That's the entire point.
    void update(const FlightData& d, PyroController& pyro);

    // Ground-commanded soft reset (see Packet.h's RESET). This class
    // otherwise fires at MOST ONCE PER BOOT by design -- a real flight only
    // gets one apogee, so that's correct for an actual flight computer reset
    // (a physical power cycle). But a soft reset explicitly exists to run
    // repeated bench/HITL test flights without power-cycling, and without
    // this, the backup monitor would silently stay latched DONE (or stuck
    // mid-stage) for every test after the first -- a real, easy-to-miss gap
    // in exactly the tool meant to make repeated testing safe. Back to
    // WAITING_FOR_BOOST and all detection state cleared, same as a fresh
    // boot would give it.
    void reset();

private:
    enum class _Stage {
        WAITING_FOR_BOOST,
        WAITING_FOR_ALTITUDE,
        WAITING_FOR_APOGEE,
        ARMED_TO_FIRE,
        DONE,
    };
    _Stage _stage = _Stage::WAITING_FOR_BOOST;

    // Mirrors StateMachine's own liftoff-detection logic (see
    // LIFTOFF_ACCEL_THRESHOLD/LIFTOFF_CONFIRM_MS in config.h) -- tracked
    // completely separately so this never depends on StateMachine having
    // survived whatever caused a reset.
    uint32_t _boostFirstMs   = 0;
    bool     _boostDetecting = false;

    float    _refAltAtBoost  = 0.0f;

    // Mirrors StateMachine's apogee sustained-non-positive-velocity logic
    // (see APOGEE_DETECTION_WINDOW_MS in config.h), independently tracked.
    uint32_t _apogeeWindowMs  = 0;
    bool     _apogeeDetecting = false;

    uint32_t _apogeeConfirmedMs = 0;
};
