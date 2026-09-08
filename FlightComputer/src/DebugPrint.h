#pragma once
#include <Arduino.h>

// Returns true at most once every intervalMs, tracking state in the
// caller-owned lastMs (declare it as `static uint32_t lastMs = 0;` at the
// call site so it persists between calls). Shared by every bench
// DEBUG_SERIAL print in the firmware so the rate-limit mechanics live in one
// place instead of being copy-pasted per print site.
inline bool debugPrintReady(uint32_t& lastMs, uint32_t intervalMs = 300) {
    uint32_t now = millis();
    if (now - lastMs < intervalMs) return false;
    lastMs = now;
    return true;
}
