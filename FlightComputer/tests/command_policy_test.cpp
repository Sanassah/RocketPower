// Compile-only tests: g++ -std=c++14 -fsyntax-only tests/command_policy_test.cpp
// Uses the actual flight command policy; no hardware or Arduino runtime needed.
#include "../src/telemetry/CommandPolicy.h"

constexpr unsigned bit(unsigned command) { return 1u << command; }
constexpr unsigned preparation = bit(0x05) | bit(0x06) | bit(0x07) |
    bit(0x09) | bit(0x0A) | bit(0x0B) | bit(0x0C) | bit(0x0D) |
    bit(0x0E) | bit(0x0F) | bit(0x10) | bit(0x11) | bit(0x12) | bit(0x13);
// Expected command IDs by wire state. These masks specify the operator contract.
constexpr unsigned permitted[] = {
    preparation | bit(0x01) | bit(0x04) | bit(0x14), // IDLE: prepare, ARM, reset
    preparation | bit(0x02) | bit(0x03) | bit(0x04) | bit(0x14) | bit(0x15), // ARMED
    bit(0x03) | bit(0x04), // ascent: manual recovery, ping
    bit(0x03) | bit(0x04), // coast
    bit(0x03) | bit(0x04), // apogee
    bit(0x03) | bit(0x04), // descent
    bit(0x04) | bit(0x14), // landed: reset to unlock preparation
    bit(0x02) | bit(0x03) | bit(0x04), // READY: cancel, manual recovery, ping
};
constexpr bool checkPolicy() {
    for (unsigned s = 0; s < 8; ++s) {
        for (unsigned c = 0; c < 256; ++c) {
            bool expected = c < 32 && (permitted[s] & bit(c));
            if (commandAllowed(static_cast<CommandType>(c), static_cast<FlightState>(s)) != expected)
                return false;
        }
    }
    return true;
}
static_assert(checkPolicy(), "Command restriction matrix mismatch");
static_assert(static_cast<unsigned>(FlightState::READY_FOR_FLIGHT) == 7, "READY wire ID");
static_assert(static_cast<unsigned>(FlightState::DESCENT) == 5, "Preserve existing wire IDs");
static_assert(sizeof(CommandPacket) == 7 && sizeof(AckPacket) == 6 && sizeof(TelemetryPacket) == 52,
              "Preserve packet layouts");
