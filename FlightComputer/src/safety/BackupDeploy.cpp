#include "BackupDeploy.h"
#include "../config.h"

void BackupDeploy::update(const FlightData& d, PyroController& pyro) {
    if (_stage == _Stage::DONE) return;
    uint32_t now = millis();

    switch (_stage) {

        // ---- Watch for a real, sustained boost -- same threshold/hold
        // time as StateMachine's own liftoff detection, tracked separately. ----
        case _Stage::WAITING_FOR_BOOST: {
            // TEMPORARY: accel_mag_ms2 (BNO085) instead of highg_mag_g
            // (ADXL375) -- see IMU_LIFTOFF_ACCEL_THRESHOLD_MS2 in config.h.
            // Kept in sync with StateMachine.cpp's own liftoff check.
            bool highG = (d.accel_mag_ms2 > IMU_LIFTOFF_ACCEL_THRESHOLD_MS2);
            if (highG && !_boostDetecting) {
                _boostDetecting = true;
                _boostFirstMs   = now;
            } else if (!highG) {
                _boostDetecting = false;
            }
            if (_boostDetecting && (now - _boostFirstMs >= LIFTOFF_CONFIRM_MS)) {
                _refAltAtBoost        = d.baro_alt_m;
                _altitudeWaitStartMs  = now;
                _stage = _Stage::WAITING_FOR_ALTITUDE;
                DEBUG_SERIAL.println("[BACKUP] Boost detected -- watching for real altitude gain");
            }
            break;
        }

        // ---- Require genuine altitude gain before ever arming the apogee
        // watch -- rules out a G-spike from being carried/dropped on the
        // bench, which could plausibly cross IMU_LIFTOFF_ACCEL_THRESHOLD_MS2
        // briefly but will never produce real sustained climb. ----
        case _Stage::WAITING_FOR_ALTITUDE:
            if (d.baro_alt_m - _refAltAtBoost >= BACKUP_MIN_ALT_GAIN_M) {
                _stage = _Stage::WAITING_FOR_APOGEE;
                DEBUG_SERIAL.println("[BACKUP] Real altitude gain confirmed -- watching for apogee");
            } else if (now - _altitudeWaitStartMs >= BACKUP_ALTITUDE_TIMEOUT_MS) {
                // Timeout fallback -- see BACKUP_ALTITUDE_TIMEOUT_MS in
                // config.h. Re-arms WAITING_FOR_BOOST instead of forcing the
                // apogee watch open: a genuine still-climbing flight (stuck
                // on a bad _refAltAtBoost sample) just gets a fresh
                // reference to try again next boost detection; a false
                // bench trigger correctly stays inert.
                DEBUG_SERIAL.println("[BACKUP] WARNING: altitude gain timeout -- re-arming boost watch");
                _stage           = _Stage::WAITING_FOR_BOOST;
                _boostDetecting  = false;
                _boostFirstMs    = 0;
            }
            break;

        // ---- Same sustained-non-positive-velocity logic as StateMachine's
        // COAST case, independently tracked -- see its comment for why this
        // can't just latch on the first non-positive sample: vert_vel_ms is
        // confirmed noisy enough (RF-coupled baro, see [VELFUSE]) to dip
        // below zero on a single glitch mid-ascent, so this requires it to
        // STAY non-positive continuously for the full window, cancelling
        // immediately if it goes positive again before that. ----
        case _Stage::WAITING_FOR_APOGEE: {
            bool nonPositive = (d.vert_vel_ms <= 0.0f);
            if (nonPositive) {
                if (!_apogeeDetecting) {
                    _apogeeDetecting = true;
                    _apogeeWindowMs  = now;
                }
                if (now - _apogeeWindowMs >= APOGEE_DETECTION_WINDOW_MS) {
                    _apogeeConfirmedMs = now;
                    _stage = _Stage::ARMED_TO_FIRE;
                    DEBUG_SERIAL.println("[BACKUP] Apogee confirmed -- firing backup after delay");
                }
            } else {
                _apogeeDetecting = false;   // still climbing -- cancel, not a real apogee
            }
            break;
        }

        // ---- Extra delay after confirmation before actually firing --
        // gives the primary path (and a ground operator watching telemetry)
        // every chance to act first. ----
        case _Stage::ARMED_TO_FIRE:
            if (now - _apogeeConfirmedMs >= BACKUP_FIRE_DELAY_MS) {
                DEBUG_SERIAL.println("[BACKUP] Independent apogee monitor firing CH3 backup now.");
                pyro.fireBackupUnconditional();
                _stage = _Stage::DONE;
            }
            break;

        case _Stage::DONE:
            break;
    }
}

void BackupDeploy::reset() {
    _stage               = _Stage::WAITING_FOR_BOOST;
    _boostFirstMs        = 0;
    _boostDetecting      = false;
    _refAltAtBoost       = 0.0f;
    _altitudeWaitStartMs = 0;
    _apogeeWindowMs      = 0;
    _apogeeDetecting     = false;
    _apogeeConfirmedMs   = 0;
}
