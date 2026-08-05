#include "StatusLED.h"
#include "../config.h"
#include <Arduino.h>

void StatusLED::begin() {
    pinMode(IMU_LED_PIN, OUTPUT);
    pinMode(BARO_LED_PIN, OUTPUT);
    pinMode(ACCEL_LED_PIN, OUTPUT);
    pinMode(GPS_LED_PIN, OUTPUT);

    digitalWrite(IMU_LED_PIN, LOW);
    digitalWrite(BARO_LED_PIN, LOW);
    digitalWrite(ACCEL_LED_PIN, LOW);
    digitalWrite(GPS_LED_PIN, LOW);
}

static void driveLed(uint8_t pin, bool ok, bool blinkPhase) {
    digitalWrite(pin, ok ? HIGH : blinkPhase);
}

void StatusLED::update(const FlightData& d) {
    // Shared flash clock so all faulted LEDs blink in sync, no per-LED timers needed.
    bool blinkPhase = (millis() / STATUS_LED_FLASH_MS) & 1;

    driveLed(IMU_LED_PIN,   d.imu_ok,   blinkPhase);
    driveLed(BARO_LED_PIN,  d.baro_ok,  blinkPhase);
    driveLed(ACCEL_LED_PIN, d.accel_ok, blinkPhase);

    // "GPS ok" means locked with satellites, not just an acknowledging module.
    bool gpsLocked = d.gps_ok && d.gps_fix && d.gps_sats > 0;
    driveLed(GPS_LED_PIN, gpsLocked, blinkPhase);
}
