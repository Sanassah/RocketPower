#pragma once
#include "Packet.h"

// Explicit allowlist: future commands are blocked until deliberately classified.
// State values are protocol IDs, not an ordered flight-phase sequence.
constexpr bool commandAllowed(CommandType command, FlightState state) {
    const bool preparing = state == FlightState::IDLE || state == FlightState::ARMED;
    const bool ready = state == FlightState::READY_FOR_FLIGHT;
    const bool flying = state == FlightState::POWERED_ASCENT || state == FlightState::COAST ||
                        state == FlightState::APOGEE || state == FlightState::DESCENT;
    switch (command) {
        case CommandType::PING: return true;
        case CommandType::ARM: return state == FlightState::IDLE;
        case CommandType::READY_FOR_FLIGHT: return state == FlightState::ARMED;
        case CommandType::DISARM: return state == FlightState::ARMED || ready;
        case CommandType::FIRE_PYRO: return state == FlightState::ARMED || ready || flying;
        case CommandType::RESET: return preparing || state == FlightState::LANDED;
        case CommandType::CALIBRATE_BARO:
        case CommandType::SERVO_TEST:
        case CommandType::CAM_TOGGLE:
        case CommandType::SERVO_NUDGE_POS:
        case CommandType::SERVO_NUDGE_NEG:
        case CommandType::SERVO_SAVE_CAL:
        case CommandType::SERVO_CENTER_ALL:
        case CommandType::SERVO_PREFLIGHT:
        case CommandType::SD_START_RECORDING:
        case CommandType::SD_STOP_RECORDING:
        case CommandType::ATTITUDE_CONTROL_ENABLE:
        case CommandType::ATTITUDE_CONTROL_DISABLE:
        case CommandType::ATTITUDE_DEMO_ENABLE:
        case CommandType::ATTITUDE_DEMO_DISABLE:
            return preparing;
        default: return false;
    }
}
