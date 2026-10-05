#include "FinController.h"
#include "../config.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

const uint8_t FinController::_pins[SERVO_NUM_CHANNELS] = {
    SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN,
};

struct FinCalibration {
    uint8_t magic;
    int16_t trimUs[SERVO_NUM_CHANNELS];
};

bool FinController::begin() {
    _loadCalibration();

    // Stagger each channel's first command instead of firing all 4 at once --
    // spreads inrush current on the shared, unregulated servo rail instead of
    // stacking it. See SERVO_INIT_STAGGER_MS in config.h.
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
        if (FIN_TEST_SKIP_CHANNEL == i + 1) {
            DEBUG_SERIAL.print("[FIN] ch"); DEBUG_SERIAL.print(i + 1);
            DEBUG_SERIAL.println(" left unattached (FIN_TEST_SKIP_CHANNEL)");
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
        DEBUG_SERIAL.println("[FIN] No saved calibration -- starting at center.");
        for (int i = 0; i < SERVO_NUM_CHANNELS; i++) _trimUs[i] = 0;
        return;
    }
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) _trimUs[i] = cal.trimUs[i];
    DEBUG_SERIAL.println("[FIN] Loaded calibration from EEPROM.");
}

void FinController::_writeUs(uint8_t idx, int32_t us, int32_t loUs, int32_t hiUs) {
    if (us < loUs) us = loUs;
    if (us > hiUs) us = hiUs;
    _liveUs[idx] = (uint16_t)us;
    _servos[idx].writeMicroseconds(_liveUs[idx]);
}

void FinController::_symmetricFlightLimits(uint8_t idx, int32_t& loUs, int32_t& hiUs) const {
    int32_t center = (int32_t)SERVO_CENTER_US + _trimUs[idx];
    int32_t marginDown = center - SERVO_MIN_US;
    int32_t marginUp   = SERVO_MAX_US - center;
    int32_t symMarginUs = (marginDown < marginUp) ? marginDown : marginUp;
    loUs = center - symMarginUs;
    hiUs = center + symMarginUs;
}

void FinController::setCorrectionDeg(uint8_t channel, float correctionDeg) {
    if (channel < 1 || channel > SERVO_NUM_CHANNELS) return;
    uint8_t idx = channel - 1;
    int32_t center = (int32_t)SERVO_CENTER_US + _trimUs[idx];

    // SERVO_MIN_US/MAX_US are measured from the MECHANICAL center, not this
    // channel's trimmed center -- so a nonzero trim leaves unequal room each
    // direction before that absolute limit (see _symmetricFlightLimits).
    // Clamp to the SAME distance from center both ways so a commanded
    // +/-X deg always produces matched real authority, same reasoning
    // (and now the same fix) as testSweep()/preflightSequence().
    int32_t loUs, hiUs;
    _symmetricFlightLimits(idx, loUs, hiUs);

    // Same us-per-degree scale as SERVO_MIN_US/SERVO_MAX_US in config.h
    // (800us / 90deg on the BMS-101HV datasheet).
    int32_t correctionUs = (int32_t)lroundf(800.0f * correctionDeg / 90.0f);
    int32_t us = center + correctionUs;
    if (us < loUs) us = loUs;
    if (us > hiUs) us = hiUs;

    _writeUs(idx, us);   // still hard-clamped to SERVO_MIN_US/MAX_US in here as a backstop
}

void FinController::_symmetricSweepLimits(uint8_t idx, int32_t& loUs, int32_t& hiUs) const {
    int32_t center = (int32_t)SERVO_CENTER_US + _trimUs[idx];
    int32_t sweepUs    = (int32_t)lroundf(800.0f * SERVO_TEST_SWEEP_ANGLE_DEG / 90.0f);
    int32_t marginDown = center - SERVO_RATED_MIN_US;
    int32_t marginUp   = SERVO_RATED_MAX_US - center;
    int32_t symMarginUs = sweepUs;
    if (symMarginUs > marginDown) symMarginUs = marginDown;
    if (symMarginUs > marginUp)   symMarginUs = marginUp;
    loUs = center - symMarginUs;
    hiUs = center + symMarginUs;
}

void FinController::testSweep(uint8_t channel) {
    if (channel < 1 || channel > SERVO_NUM_CHANNELS) return;
    uint8_t idx = channel - 1;
    int32_t center = (int32_t)SERVO_CENTER_US + _trimUs[idx];
    int32_t loUs, hiUs;
    _symmetricSweepLimits(idx, loUs, hiUs);

    DEBUG_SERIAL.print("[FIN] Test sweep ch"); DEBUG_SERIAL.println(channel);
    _writeUs(idx, center, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);  delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, loUs,   SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);  delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, hiUs,   SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);  delay(SERVO_TEST_STEP_MS);
    _writeUs(idx, center, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);  delay(SERVO_TEST_STEP_MS);
    DEBUG_SERIAL.print("[FIN] ch"); DEBUG_SERIAL.print(channel); DEBUG_SERIAL.println(" complete");
}

void FinController::_rampTo(uint8_t idx, int32_t targetUs, uint32_t durationMs, int32_t loUs, int32_t hiUs) {
    const uint8_t steps = 20;
    int32_t startUs = (int32_t)_liveUs[idx];
    uint32_t stepDelay = durationMs / steps;
    for (uint8_t s = 1; s <= steps; s++) {
        _writeUs(idx, startUs + (targetUs - startUs) * (int32_t)s / steps, loUs, hiUs);
        delay(stepDelay);
    }
}

void FinController::_rampAllTo(const int32_t targets[SERVO_NUM_CHANNELS], uint32_t durationMs, int32_t loUs, int32_t hiUs) {
    const uint8_t steps = 20;
    int32_t startUs[SERVO_NUM_CHANNELS];
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) startUs[i] = (int32_t)_liveUs[i];
    uint32_t stepDelay = durationMs / steps;
    for (uint8_t s = 1; s <= steps; s++) {
        for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
            _writeUs(i, startUs[i] + (targets[i] - startUs[i]) * (int32_t)s / steps, loUs, hiUs);
        }
        delay(stepDelay);
    }
}

void FinController::preflightSequence() {
    static const char* const compass[SERVO_NUM_CHANNELS] = {"S", "E", "N", "W"};

    // Per-channel +-SERVO_TEST_SWEEP_ANGLE_DEG limits, same as testSweep()
    // (see _symmetricSweepLimits) -- reaches further than flight authority,
    // bounded by the servo's rated range instead, symmetric per channel.
    int32_t centerUs[SERVO_NUM_CHANNELS], maxAll[SERVO_NUM_CHANNELS], minAll[SERVO_NUM_CHANNELS];
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
        centerUs[i] = (int32_t)SERVO_CENTER_US + _trimUs[i];
        _symmetricSweepLimits(i, minAll[i], maxAll[i]);
    }

    DEBUG_SERIAL.println("[FIN] Preflight sequence start");

    // 1. Settle at trimmed center.
    _rampAllTo(centerUs, 300, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
    delay(150);

    // 2. FAST: compass wave, CH1(S) -> CH2(E) -> CH3(N) -> CH4(W).
    DEBUG_SERIAL.println("[FIN] Preflight fast wave");
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
        DEBUG_SERIAL.print("[FIN] Preflight ch"); DEBUG_SERIAL.print(i + 1);
        DEBUG_SERIAL.print(" ("); DEBUG_SERIAL.print(compass[i]); DEBUG_SERIAL.println(")");
        _rampTo(i, maxAll[i], 180, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
        delay(90);
        _rampTo(i, centerUs[i], 180, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
    }

    // 3. SLOW: unison sweep -- out to max, down through min, back to center.
    DEBUG_SERIAL.println("[FIN] Preflight slow sweep");
    _rampAllTo(maxAll, 550, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
    delay(200);
    _rampAllTo(minAll, 1200, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
    delay(200);
    _rampAllTo(centerUs, 750, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);

    // 4. FAST: quick synchronized flutter, a snappy confidence check. Full
    // +-SERVO_TEST_SWEEP_ANGLE_DEG amplitude (same maxAll/minAll as the slow
    // sweep above), not a small wiggle -- so it actually reads on camera at
    // this speed instead of looking like a twitch.
    DEBUG_SERIAL.println("[FIN] Preflight fast flutter");
    for (int rep = 0; rep < 3; rep++) {
        for (int i = 0; i < SERVO_NUM_CHANNELS; i++) _writeUs(i, maxAll[i], SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
        delay(90);
        for (int i = 0; i < SERVO_NUM_CHANNELS; i++) _writeUs(i, minAll[i], SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);
        delay(90);
    }

    // 5. Final settle at trimmed center.
    _rampAllTo(centerUs, 300, SERVO_RATED_MIN_US, SERVO_RATED_MAX_US);

    DEBUG_SERIAL.println("[FIN] Preflight sequence complete");
}

void FinController::nudge(uint8_t channel, bool positive) {
    if (channel < 1 || channel > SERVO_NUM_CHANNELS) return;
    uint8_t idx  = channel - 1;
    int32_t step = positive ? SERVO_TRIM_STEP_US : -SERVO_TRIM_STEP_US;
    _writeUs(idx, (int32_t)_liveUs[idx] + step);

    DEBUG_SERIAL.print("[FIN] ch"); DEBUG_SERIAL.print(channel);
    DEBUG_SERIAL.print(" nudged to "); DEBUG_SERIAL.print(_liveUs[idx]); DEBUG_SERIAL.println("us");
}

void FinController::centerAll() {
    DEBUG_SERIAL.println("[FIN] Centering all channels to raw SERVO_CENTER_US.");
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
        _writeUs(i, SERVO_CENTER_US);
    }
}

float FinController::liveCorrectionDeg(uint8_t channel) const {
    if (channel < 1 || channel > SERVO_NUM_CHANNELS) return 0.0f;
    uint8_t idx = channel - 1;
    // Inverse of setCorrectionDeg()'s us-per-degree scale, applied to the
    // post-clamp live value -- see the comment on _writeUs().
    return ((float)_liveUs[idx] - (float)SERVO_CENTER_US - (float)_trimUs[idx]) * 90.0f / 800.0f;
}

void FinController::saveCalibration() {
    FinCalibration cal;
    cal.magic = SERVO_CAL_MAGIC;
    for (int i = 0; i < SERVO_NUM_CHANNELS; i++) {
        cal.trimUs[i] = (int16_t)_liveUs[i] - (int16_t)SERVO_CENTER_US;
        _trimUs[i]    = cal.trimUs[i];
    }
    EEPROM.put(SERVO_CAL_EEPROM_ADDR, cal);
    DEBUG_SERIAL.println("[FIN] Calibration saved to EEPROM.");
}
