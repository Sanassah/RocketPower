#pragma once
#include <stdint.h>

// Controls the 3 pyro channels on the custom MIMXRT1062 board.
// From schematic (HighCurrentComponents.kicad_sch):
//   CH1 (ignition):  fire=pin2, continuity=pin40
//   CH2 (parachute): fire=pin3, continuity=pin41
//   CH3 (backup):    fire=pin4, continuity=pin39
class PyroController {
public:
    bool begin();    // configure GPIO directions

    void arm();      // enable pyro logic (software arm)
    void disarm();   // disable pyro logic

    // Call once per main loop iteration (unconditionally, every state) --
    // turns a channel's pin back LOW once PYRO_FIRE_DURATION_MS has elapsed
    // since fire()/fireBackupUnconditional() set it HIGH. Non-blocking on
    // purpose: fire()/fireBackupUnconditional() used to hold the whole
    // flight computer loop frozen for that entire duration with a plain
    // delay() call -- sensors, state machine, telemetry, logging, all
    // stalled at the exact moment of deployment. Bounded and one-time per
    // flight, so not urgent, but free to remove, so removed.
    void update();

    // Fire a channel for PYRO_FIRE_DURATION_MS ms.
    // Refuses to fire if not armed or continuity check fails.
    bool fire(uint8_t channel);

    // Fires CH3 (backup) UNCONDITIONALLY -- deliberately bypasses the
    // normal _armed check. This exists for exactly one caller: BackupDeploy,
    // an independent apogee monitor that runs regardless of StateMachine's
    // state or this class's armed flag (neither survives a reset -- see the
    // comment above BACKUP_MIN_ALT_GAIN_M in config.h for why that matters).
    // Its own multi-stage physical evidence (sustained accel, real altitude
    // gain, a confirmed velocity zero-crossing, plus an extra delay) IS the
    // safety gate for this path -- the same principle commercial dual-
    // deploy altimeters use for their backup channel. Hardcoded to channel 3
    // only; this is not a general-purpose bypass of fire()'s safety check.
    bool fireBackupUnconditional();

    // Returns true if continuity detected on channel (1-3)
    bool continuityOk(uint8_t channel) const;

    bool isArmed() const { return _armed; }

private:
    bool _armed = false;

    static const uint8_t _firePins[3];
    static const uint8_t _contPins[3];

    // Non-blocking fire timer state, one slot per channel -- see update().
    bool     _firing[3]      = {false, false, false};
    uint32_t _fireStartMs[3] = {0, 0, 0};

    bool _safetyCheck(uint8_t ch) const;
};
