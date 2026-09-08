#include "StateMachine.h"
#include "../config.h"
#include "../control/PyroController.h"
#include "../DebugPrint.h"

void StateMachine::_enterState(FlightState next, uint32_t now) {
    _state          = next;
    _stateEnteredMs = now;

    // Reset per-state detection variables
    _liftoffDetecting   = false;
    _burnoutDetecting   = false;
    _decelDetecting     = false;
    _peakAscentVertVel  = 0.0f;   // fresh peak-tracker every (re-)entry to POWERED_ASCENT
    _apogeeDetecting    = false;
    _landedStableMs     = 0;

    DEBUG_SERIAL.print("[FSM] → ");
    DEBUG_SERIAL.println(flightStateName(next));
}

void StateMachine::onArm() {
    if (_state == FlightState::IDLE) {
        _enterState(FlightState::ARMED, millis());
    }
}

void StateMachine::onDisarm() {
    if (_state == FlightState::ARMED) {
        _enterState(FlightState::IDLE, millis());
    }
}

void StateMachine::reset() {
    // Unconditional, unlike onDisarm() -- goes to IDLE from ANY state. Also
    // clears timing/reference vars _enterState() doesn't touch, so nothing
    // stale leaks into the next (reset) flight's first loop.
    _enterState(FlightState::IDLE, millis());
    _liftoffFirstMs = 0;
    _burnoutFirstMs = 0;
    _decelFirstMs   = 0;
    _apogeeWindowMs = 0;
    _landedRefAlt   = 0.0f;
}

void StateMachine::update(const FlightData& d) {
    uint32_t now = millis();

    switch (_state) {

        // ---- IDLE: wait for arm command (handled by onArm()) ----
        case FlightState::IDLE:
            break;

        // ---- ARMED: detect liftoff ----
        case FlightState::ARMED: {
            // TEMPORARY: accel_mag_ms2 (BNO085) instead of highg_mag_g
            // (ADXL375) -- see IMU_LIFTOFF_ACCEL_THRESHOLD_MS2 in config.h.
            bool highG = (d.accel_mag_ms2 > IMU_LIFTOFF_ACCEL_THRESHOLD_MS2);
            {
                static uint32_t lastMs = 0;
                if (debugPrintReady(lastMs)) {
                    DEBUG_SERIAL.print("[LIFTOFF CHECK] accelMag_ms2=");
                    DEBUG_SERIAL.print(d.accel_mag_ms2, 3);
                    DEBUG_SERIAL.print(" threshold="); DEBUG_SERIAL.println(IMU_LIFTOFF_ACCEL_THRESHOLD_MS2);
                }
            }
            if (highG && !_liftoffDetecting) {
                _liftoffDetecting = true;
                _liftoffFirstMs   = now;
            } else if (!highG) {
                _liftoffDetecting = false;
            }
            if (_liftoffDetecting && (now - _liftoffFirstMs >= LIFTOFF_CONFIRM_MS)) {
                _enterState(FlightState::POWERED_ASCENT, now);
            }
            break;
        }

        // ---- POWERED_ASCENT: detect SUSTAINED burnout (accel + velocity cross-check) ----
        case FlightState::POWERED_ASCENT: {
            // TEMPORARY: accel_mag_ms2 (BNO085) instead of highg_mag_g
            // (ADXL375) -- see IMU_BURNOUT_ACCEL_THRESHOLD_MS2 in config.h.
            bool lowG = (d.accel_mag_ms2 < IMU_BURNOUT_ACCEL_THRESHOLD_MS2);
            if (lowG) {
                if (!_burnoutDetecting) {
                    _burnoutDetecting = true;
                    _burnoutFirstMs   = now;
                }
                if (now - _burnoutFirstMs >= BURNOUT_CONFIRM_MS) {
                    _enterState(FlightState::COAST, now);
                }
            } else {
                _burnoutDetecting = false;   // still thrusting -- cancel, not a real burnout
            }

            // Fallback: catches a stuck/failed accelerometer -- see
            // POWERED_ASCENT_VELOCITY_DROP_MS.
            if (_state == FlightState::POWERED_ASCENT) {
                if (d.vert_vel_ms > _peakAscentVertVel) _peakAscentVertVel = d.vert_vel_ms;
                bool decelerating = (d.vert_vel_ms < _peakAscentVertVel - POWERED_ASCENT_VELOCITY_DROP_MS);
                if (decelerating) {
                    if (!_decelDetecting) {
                        _decelDetecting = true;
                        _decelFirstMs   = now;
                    }
                    if (now - _decelFirstMs >= BURNOUT_CONFIRM_MS) {
                        _enterState(FlightState::COAST, now);
                    }
                } else {
                    _decelDetecting = false;   // still climbing at/near peak rate -- cancel
                }
            }
            break;
        }

        // ---- COAST: detect apogee via SUSTAINED non-positive vertical velocity ----
        // Must stay <=0 continuously for the full window, cancelling
        // immediately if it goes positive again -- vert_vel_ms can cross
        // zero on baro noise alone (see [VELFUSE]), not just at real apogee,
        // so a single sample can't be allowed to fire the main parachute.
        case FlightState::COAST: {
            bool nonPositive = (d.vert_vel_ms <= 0.0f);
            if (nonPositive) {
                if (!_apogeeDetecting) {
                    _apogeeDetecting = true;
                    _apogeeWindowMs  = now;
                }
                if (now - _apogeeWindowMs >= APOGEE_DETECTION_WINDOW_MS) {
                    _enterState(FlightState::APOGEE, now);
                }
            } else {
                _apogeeDetecting = false;   // still climbing -- cancel, not a real apogee
            }
            break;
        }

        // ---- APOGEE: deploy parachute, immediately transition to DESCENT ----
        case FlightState::APOGEE:
            _pyro.fire(PYRO_PARACHUTE);
            _enterState(FlightState::DESCENT, now);
            break;

        // ---- DESCENT: detect landing via stable altitude ----
        case FlightState::DESCENT: {
            bool stable = (fabsf(d.baro_alt_m - _landedRefAlt) < LANDED_ALT_TOLERANCE_M);
            if (stable) {
                if (_landedStableMs == 0) _landedStableMs = now;
                if (now - _landedStableMs >= LANDED_STABLE_MS) {
                    _enterState(FlightState::LANDED, now);
                }
            } else {
                _landedRefAlt   = d.baro_alt_m;
                _landedStableMs = 0;
            }
            break;
        }

        // ---- LANDED: nothing to do ----
        case FlightState::LANDED:
            break;
    }
}
