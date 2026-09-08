#include "PyroController.h"
#include "../config.h"
#include <Arduino.h>

// From schematic: PyroCHx_N labels give MCU pin N for each channel
const uint8_t PyroController::_firePins[3] = {
    PYRO_CH1_FIRE_PIN,   // pin 2  — parachute
    PYRO_CH2_FIRE_PIN,   // pin 3  — reserved (future booster/2nd stage)
    PYRO_CH3_FIRE_PIN,   // pin 4  — backup
};
const uint8_t PyroController::_contPins[3] = {
    PYRO_CH1_CONT_PIN,   // pin 40
    PYRO_CH2_CONT_PIN,   // pin 41
    PYRO_CH3_CONT_PIN,   // pin 39
};

bool PyroController::begin() {
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        pinMode(_firePins[i], OUTPUT);
        digitalWrite(_firePins[i], LOW);   // ensure all channels start OFF
        pinMode(_contPins[i], INPUT);      // continuity sense is analog-capable digital input
    }
    _armed = false;
    return true;
}

void PyroController::arm() {
    _armed = true;
    DEBUG_SERIAL.println("[PYRO] Armed");
}

void PyroController::disarm() {
    _armed = false;
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        digitalWrite(_firePins[i], LOW);
        _firing[i] = false;   // cancel any in-progress non-blocking fire timer too
    }
    DEBUG_SERIAL.println("[PYRO] Disarmed");
}

void PyroController::update() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        if (_firing[i] && (now - _fireStartMs[i] >= PYRO_FIRE_DURATION_MS)) {
            digitalWrite(_firePins[i], LOW);
            _firing[i] = false;
            DEBUG_SERIAL.print("[PYRO] ch"); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.println(" complete");
        }
    }
}

bool PyroController::continuityOk(uint8_t channel) const {
    if (channel < 1 || channel > PYRO_NUM_CHANNELS) return false;
    int raw = analogRead(_contPins[channel - 1]);
    return (raw > PYRO_CONT_THRESHOLD);
}

bool PyroController::fire(uint8_t channel) {
    if (!_safetyCheck(channel)) return false;

    uint8_t idx = channel - 1;
    DEBUG_SERIAL.print("[PYRO] Firing ch"); DEBUG_SERIAL.println(channel);

    digitalWrite(_firePins[idx], HIGH);
    _firing[idx]      = true;
    _fireStartMs[idx] = millis();   // update() turns it back off once PYRO_FIRE_DURATION_MS elapses
    return true;
}

bool PyroController::fireBackupUnconditional() {
    const uint8_t channel = PYRO_BACKUP;
    uint8_t idx = channel - 1;
    DEBUG_SERIAL.println("[PYRO] BACKUP AUTO-FIRE ch3 -- independent apogee evidence, bypassing arm check");

    digitalWrite(_firePins[idx], HIGH);
    _firing[idx]      = true;
    _fireStartMs[idx] = millis();   // update() turns it back off once PYRO_FIRE_DURATION_MS elapses
    return true;
}

bool PyroController::_safetyCheck(uint8_t ch) const {
    if (!_armed) {
        DEBUG_SERIAL.println("[PYRO] SAFETY: not armed");
        return false;
    }
    if (ch < 1 || ch > PYRO_NUM_CHANNELS) {
        DEBUG_SERIAL.println("[PYRO] SAFETY: invalid channel");
        return false;
    }
    return true;
}
