#pragma once
#include "../sensors/SensorManager.h"
#include "../control/PyroController.h"

// Separate backup apogee detector for an explicitly readied flight session.
// IDLE/ARMED clear its evidence; READY and subsequent flight states permit
// boost -> altitude gain -> apogee -> delayed CH3 deployment. It shares the
// primary's sensors and velocity estimate, and does not survive a reboot.
// See PR-005. Automatic firing bypasses PyroController's arm flag only after
// this sequence; cancellation to IDLE clears it. At most once per session.
class BackupDeploy {
public:
    // Call every loop with the current state stamped into FlightData.
    void update(const FlightData& d, PyroController& pyro);

    // Clear evidence on cancellation or a permitted ground reset.
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
    // IMU_LIFTOFF_ACCEL_THRESHOLD_MS2/LIFTOFF_CONFIRM_MS in config.h) --
    // tracked completely separately so this never depends on StateMachine
    // having survived whatever caused a reset.
    uint32_t _boostFirstMs   = 0;
    bool     _boostDetecting = false;

    float    _refAltAtBoost  = 0.0f;

    // Timeout fallback for WAITING_FOR_ALTITUDE -- see
    // BACKUP_ALTITUDE_TIMEOUT_MS in config.h for why this exists: without
    // it, a stuck/bad baro reading could leave this stage waiting forever,
    // silently disabling the whole backup-deploy path for the rest of the
    // flight. Timestamp of when WAITING_FOR_ALTITUDE was entered.
    uint32_t _altitudeWaitStartMs = 0;

    // Mirrors StateMachine's apogee sustained-non-positive-velocity logic
    // (see APOGEE_DETECTION_WINDOW_MS in config.h), independently tracked.
    uint32_t _apogeeWindowMs  = 0;
    bool     _apogeeDetecting = false;

    uint32_t _apogeeConfirmedMs = 0;
};
