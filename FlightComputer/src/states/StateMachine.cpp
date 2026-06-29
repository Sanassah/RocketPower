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

        // ---- COAST: detect apogee via vertical velocity zero-crossing ----
        case FlightState::COAST: {
            bool zeroCross = (_prevVertVel > 0.0f && d.vert_vel_ms <= 0.0f);
            if (zeroCross && !_apogeeDetecting) {
                _apogeeDetecting = true;
                _apogeeWindowMs  = now;
            }
            if (_apogeeDetecting && (now - _apogeeWindowMs >= APOGEE_DETECTION_WINDOW_MS)) {
                _enterState(FlightState::APOGEE, now);
            }
            if (!zeroCross && !_apogeeDetecting) {
                _apogeeWindowMs = now;   // keep sliding window until velocity goes negative
            }
            _prevVertVel = d.vert_vel_ms;
            break;
        }

        // ---- APOGEE: fire drogue, immediately transition to DESCENT ----
        case FlightState::APOGEE:
            _pyro.fire(PYRO_DROGUE);
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
