#include "BackupDeploy.h"
#include "../config.h"

void BackupDeploy::update(const FlightData& d, PyroController& pyro) {
    if (_stage == _Stage::DONE) return;
    uint32_t now = millis();

    switch (_stage) {

        // ---- Watch for a real, sustained boost -- same threshold/hold
        // time as StateMachine's own liftoff detection, tracked separately. ----
        case _Stage::WAITING_FOR_BOOST: {
            bool highG = (d.highg_mag_g > LIFTOFF_ACCEL_THRESHOLD);
            if (highG && !_boostDetecting) {
                _boostDetecting = true;
                _boostFirstMs   = now;
            } else if (!highG) {
                _boostDetecting = false;
            }
            if (_boostDetecting && (now - _boostFirstMs >= LIFTOFF_CONFIRM_MS)) {
                _refAltAtBoost = d.baro_alt_m;
                _stage = _Stage::WAITING_FOR_ALTITUDE;
                Serial.println("[BACKUP] Boost detected -- watching for real altitude gain");
            }
            break;
        }

        // ---- Require genuine altitude gain before ever arming the apogee
        // watch -- rules out a G-spike from being carried/dropped on the
        // bench, which could plausibly cross LIFTOFF_ACCEL_THRESHOLD
        // briefly but will never produce real sustained climb. ----
        case _Stage::WAITING_FOR_ALTITUDE:
            if (d.baro_alt_m - _refAltAtBoost >= BACKUP_MIN_ALT_GAIN_M) {
                _stage = _Stage::WAITING_FOR_APOGEE;
                Serial.println("[BACKUP] Real altitude gain confirmed -- watching for apogee");
            }
            break;

        // ---- Same zero-crossing logic as StateMachine's COAST case,
        // independently tracked. ----
        case _Stage::WAITING_FOR_APOGEE: {
            bool zeroCross = (_prevVertVel > 0.0f && d.vert_vel_ms <= 0.0f);
            if (zeroCross && !_apogeeDetecting) {
                _apogeeDetecting = true;
                _apogeeWindowMs  = now;
            }
            if (_apogeeDetecting && (now - _apogeeWindowMs >= APOGEE_DETECTION_WINDOW_MS)) {
                _apogeeConfirmedMs = now;
                _stage = _Stage::ARMED_TO_FIRE;
                Serial.println("[BACKUP] Apogee confirmed -- firing backup after delay");
            }
            if (!zeroCross && !_apogeeDetecting) {
                _apogeeWindowMs = now;   // keep sliding window until velocity goes negative
            }
            _prevVertVel = d.vert_vel_ms;
            break;
        }

        // ---- Extra delay after confirmation before actually firing --
        // gives the primary path (and a ground operator watching telemetry)
        // every chance to act first. ----
        case _Stage::ARMED_TO_FIRE:
            if (now - _apogeeConfirmedMs >= BACKUP_FIRE_DELAY_MS) {
                Serial.println("[BACKUP] Independent apogee monitor firing CH3 backup now.");
                pyro.fireBackupUnconditional();
                _stage = _Stage::DONE;
            }
            break;

        case _Stage::DONE:
            break;
    }
}

void BackupDeploy::reset() {
    _stage           = _Stage::WAITING_FOR_BOOST;
    _boostFirstMs    = 0;
    _boostDetecting  = false;
    _refAltAtBoost   = 0.0f;
    _prevVertVel     = 0.0f;
    _apogeeWindowMs  = 0;
    _apogeeDetecting = false;
    _apogeeConfirmedMs = 0;
}
