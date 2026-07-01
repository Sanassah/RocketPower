#include "PyroController.h"
#include "../config.h"
#include <Arduino.h>

// From schematic: PyroCHx_N labels give MCU pin N for each channel
const uint8_t PyroController::_firePins[3] = {
    PYRO_CH1_FIRE_PIN,   // pin 2  — ignition
    PYRO_CH2_FIRE_PIN,   // pin 3  — parachute
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
    Serial.println("[PYRO] Armed");
}

void PyroController::disarm() {
    _armed = false;
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        digitalWrite(_firePins[i], LOW);
    }
    Serial.println("[PYRO] Disarmed");
}

bool PyroController::continuityOk(uint8_t channel) const {
    if (channel < 1 || channel > PYRO_NUM_CHANNELS) return false;
    int raw = analogRead(_contPins[channel - 1]);
    return (raw > PYRO_CONT_THRESHOLD);
}

bool PyroController::fire(uint8_t channel) {
    if (!_safetyCheck(channel)) return false;

    uint8_t pin = _firePins[channel - 1];
    Serial.print("[PYRO] Firing ch"); Serial.println(channel);

    digitalWrite(pin, HIGH);
    delay(PYRO_FIRE_DURATION_MS);
    digitalWrite(pin, LOW);

    Serial.print("[PYRO] ch"); Serial.print(channel); Serial.println(" complete");
    return true;
}

bool PyroController::_safetyCheck(uint8_t ch) const {
    if (!_armed) {
        Serial.println("[PYRO] SAFETY: not armed");
        return false;
    }
    if (ch < 1 || ch > PYRO_NUM_CHANNELS) {
        Serial.println("[PYRO] SAFETY: invalid channel");
        return false;
    }
    return true;
}
