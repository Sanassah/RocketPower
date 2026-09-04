#include "FinController.h"
#include "../config.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

const uint8_t FinController::_pins[4] = {
    SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN,
};

struct FinCalibration {
    uint8_t magic;
    int16_t trimUs[4];
};

bool FinController::begin() {
    _loadCalibration();

    // Stagger each channel's first command instead of firing all 4 at once --
    // spreads inrush current on the shared, unregulated servo rail instead of
    // stacking it. See SERVO_INIT_STAGGER_MS in config.h.
    for (int i = 0; i < 4; i++) {
        if (FIN_TEST_SKIP_CHANNEL == i + 1) {
            Serial.print("[FIN] ch"); Serial.print(i + 1);
            Serial.println(" left unattached (FIN_TEST_SKIP_CHANNEL)");
            continue;
        }
        _servos[i].attach(_pins[i]);
        _writeUs(i, (int32_t)SERVO_CENTER_US + _trimUs[i]);
        delay(SERVO_INIT_STAGGER_MS);
    }
    return true;
}

void FinController::_loadCalibration() {
    FinCalibration cal;
    EEPROM.get(SERVO_CAL_EEPROM_ADDR, cal);
    if (cal.magic != SERVO_CAL_MAGIC) {
        Serial.println("[FIN] No saved calibration -- starting at center.");
        for (int i = 0; i < 4; i++) _trimUs[i] = 0;
        return;
    }
    for (int i = 0; i < 4; i++) _trimUs[i] = cal.trimUs[i];
    Serial.println("[FIN] Loaded calibration from EEPROM.");
}

void FinController::_writeUs(uint8_t idx, int32_t us) {
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    _liveUs[idx] = (uint16_t)us;
    _servos[idx].writeMicroseconds(_liveUs[idx]);
}

void FinController::setAngle(uint8_t channel, uint8_t angleDeg) {
    if (channel < 1 || channel > 4) return;
    if (angleDeg > 180) angleDeg = 180;
    uint8_t idx = channel - 1;
    int32_t us  = SERVO_MIN_US + (int32_t)(SERVO_MAX_US - SERVO_MIN_US) * angleDeg / 180;
    _writeUs(idx, us);
}

void FinController::setCorrectionDeg(uint8_t channel, float correctionDeg) {
    if (channel < 1 || channel > 4) return;
    uint8_t idx = channel - 1;
    // Same us-per-degree scale as SERVO_MIN_US/SERVO_MAX_US in config.h
    // (800us / 90deg on the BMS-101HV datasheet).
    int32_t us = (int32_t)SERVO_CENTER_US + _trimUs[idx]
               + (int32_t)lroundf(800.0f * correctionDeg / 90.0f);
    _writeUs(idx, us);   // still hard-clamped to SERVO_MIN_US/MAX_US in here
}

void FinController::testSweep(uint8_t channel) {
    if (channel < 1 || channel > 4) return;
    uint8_t idx = channel - 1;
    int32_t center = (int32_t)SERVO_CENTER_US + _trimUs[idx];

    Serial.print("[FIN] Test sweep ch"); Serial.println(channel);
    _writeUs(idx, center);        delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, SERVO_MIN_US);  delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, SERVO_MAX_US);  delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, center);        delay(SERVO_TEST_STEP_MS);
    Serial.print("[FIN] ch"); Serial.print(channel); Serial.println(" complete");
}

void FinController::_rampTo(uint8_t idx, int32_t targetUs, uint32_t durationMs) {
    const uint8_t steps = 20;
    int32_t startUs = (int32_t)_liveUs[idx];
    uint32_t stepDelay = durationMs / steps;
    for (uint8_t s = 1; s <= steps; s++) {
        _writeUs(idx, startUs + (targetUs - startUs) * (int32_t)s / steps);
        delay(stepDelay);
    }
}

void FinController::_rampAllTo(const int32_t targets[4], uint32_t durationMs) {
    const uint8_t steps = 20;
    int32_t startUs[4];
    for (int i = 0; i < 4; i++) startUs[i] = (int32_t)_liveUs[i];
    uint32_t stepDelay = durationMs / steps;
    for (uint8_t s = 1; s <= steps; s++) {
        for (int i = 0; i < 4; i++) {
            _writeUs(i, startUs[i] + (targets[i] - startUs[i]) * (int32_t)s / steps);
        }
        delay(stepDelay);
    }
}

void FinController::preflightSequence() {
    static const char* const compass[4] = {"S", "E", "N", "W"};

    int32_t centerUs[4], maxAll[4], minAll[4];
    for (int i = 0; i < 4; i++) {
        centerUs[i] = (int32_t)SERVO_CENTER_US + _trimUs[i];
        maxAll[i]   = SERVO_MAX_US;
        minAll[i]   = SERVO_MIN_US;
    }

    Serial.println("[FIN] Preflight sequence start");

    // 1. Settle at trimmed center.
    _rampAllTo(centerUs, 300);
    delay(150);

    // 2. FAST: compass wave, CH1(S) -> CH2(E) -> CH3(N) -> CH4(W).
    Serial.println("[FIN] Preflight fast wave");
    for (int i = 0; i < 4; i++) {
        Serial.print("[FIN] Preflight ch"); Serial.print(i + 1);
        Serial.print(" ("); Serial.print(compass[i]); Serial.println(")");
        _rampTo(i, SERVO_MAX_US, 180);
        delay(90);
        _rampTo(i, centerUs[i], 180);
    }

    // 3. SLOW: unison sweep -- out to max, down through min, back to center.
    Serial.println("[FIN] Preflight slow sweep");
    _rampAllTo(maxAll, 550);
    delay(200);
    _rampAllTo(minAll, 1200);
    delay(200);
    _rampAllTo(centerUs, 750);

    // 4. FAST: quick synchronized flutter, a snappy confidence check.
    Serial.println("[FIN] Preflight fast flutter");
    int32_t wiggleHi[4], wiggleLo[4];
    for (int i = 0; i < 4; i++) {
        wiggleHi[i] = centerUs[i] + 60;
        wiggleLo[i] = centerUs[i] - 60;
    }
    for (int rep = 0; rep < 3; rep++) {
        for (int i = 0; i < 4; i++) _writeUs(i, wiggleHi[i]);
        delay(90);
        for (int i = 0; i < 4; i++) _writeUs(i, wiggleLo[i]);
        delay(90);
    }

    // 5. Final settle at trimmed center.
    _rampAllTo(centerUs, 300);

    Serial.println("[FIN] Preflight sequence complete");
}

void FinController::nudge(uint8_t channel, bool positive) {
    if (channel < 1 || channel > 4) return;
    uint8_t idx  = channel - 1;
    int32_t step = positive ? SERVO_TRIM_STEP_US : -SERVO_TRIM_STEP_US;
    _writeUs(idx, (int32_t)_liveUs[idx] + step);

    Serial.print("[FIN] ch"); Serial.print(channel);
    Serial.print(" nudged to "); Serial.print(_liveUs[idx]); Serial.println("us");
}

void FinController::centerAll() {
    Serial.println("[FIN] Centering all channels to raw SERVO_CENTER_US.");
    for (int i = 0; i < 4; i++) {
        _writeUs(i, SERVO_CENTER_US);
    }
}

float FinController::liveCorrectionDeg(uint8_t channel) const {
    if (channel < 1 || channel > 4) return 0.0f;
    uint8_t idx = channel - 1;
    // Inverse of setCorrectionDeg()'s us-per-degree scale, applied to the
    // post-clamp live value -- see the comment on _writeUs().
    return ((float)_liveUs[idx] - (float)SERVO_CENTER_US - (float)_trimUs[idx]) * 90.0f / 800.0f;
}

void FinController::saveCalibration() {
    FinCalibration cal;
    cal.magic = SERVO_CAL_MAGIC;
    for (int i = 0; i < 4; i++) {
        cal.trimUs[i] = (int16_t)_liveUs[i] - (int16_t)SERVO_CENTER_US;
        _trimUs[i]    = cal.trimUs[i];
    }
    EEPROM.put(SERVO_CAL_EEPROM_ADDR, cal);
    Serial.println("[FIN] Calibration saved to EEPROM.");
}
