#include "StateMachine.h"
#include "../config.h"
#include "../control/PyroController.h"

void StateMachine::_enterState(FlightState next, uint32_t now) {
    _state          = next;
    _stateEnteredMs = now;

    // Reset per-state detection variables
    _liftoffDetecting = false;
    _apogeeDetecting  = false;
    _landedStableMs   = 0;

    Serial.print("[FSM] → ");
    Serial.println(flightStateName(next));
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
    // Unconditional, unlike onDisarm() -- goes to IDLE from ANY state.
    // _enterState() already clears _liftoffDetecting/_apogeeDetecting/
    // _landedStableMs; the remaining timing/reference variables aren't
    // touched by a normal state transition, so clear them explicitly too --
    // otherwise a stale _apogeeWindowMs or _landedRefAlt from the previous
    // (reset) flight could feed a spurious detection on the very first loop
    // of the next one.
    _enterState(FlightState::IDLE, millis());
    _liftoffFirstMs = 0;
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
            bool highG = (d.highg_mag_g > LIFTOFF_ACCEL_THRESHOLD);
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

        // ---- POWERED_ASCENT: detect burnout ----
        case FlightState::POWERED_ASCENT:
            if (d.highg_mag_g < BURNOUT_ACCEL_THRESHOLD) {
                _enterState(FlightState::COAST, now);
            }
            break;

        // ---- COAST: detect apogee via SUSTAINED non-positive vertical velocity ----
        // BUG FIX (2026-09): this used to latch _apogeeDetecting permanently
        // on the first now-confirmed-noisy sample where vert_vel_ms dipped
        // to/below zero (see SensorManager.cpp's [VELFUSE] diagnostic --
        // fusedVel genuinely crosses zero on baro RF-coupling noise alone,
        // not just at real apogee), then fired APOGEE 200ms later
        // regardless of what velocity did in between -- a live path to
        // firing the main parachute during POWERED ASCENT from noise, not a
        // real apogee. Fixed: require vert_vel_ms to stay <= 0
        // CONTINUOUSLY for the full window, cancelling immediately (not
        // just failing to arm) the moment it goes positive again.
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
