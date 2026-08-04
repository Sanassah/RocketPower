#pragma once
#include <Servo.h>
#include <stdint.h>

// Drives the 4 fin-actuation RC servos (Servo1_PWM..Servo4_PWM on the schematic,
// HighCurrentComponents.kicad_sch) via the Arduino Servo library. A custom
// FlexPWM/analogWrite implementation was tried here to chase a chatter issue
// (wrongly blamed on the Servo library's 50Hz rate -- the real cause was the
// wrong pulse-width center/range, since fixed via the BMS-101HV datasheet
// below) and it introduced a worse bug where only one of the four channels
// would actually respond after boot. Confirmed via an isolated single-purpose
// test project (ServoTest/, plain Servo library, nothing else running) that
// all 4 channels work correctly on this hardware -- so back to the Servo
// library here too. Each channel has a persisted trim (an offset from
// SERVO_CENTER_US in microseconds) set via the ground-station nudge buttons
// against a printed alignment jig, then saved to flash so the fins return to the
// calibrated zero on every boot -- no hand-turning or horn removal required.
class FinController {
public:
    bool begin();    // load trim from EEPROM, attach all 4 servos, drive to trimmed center

    // Move one channel (1-4) to an angle in degrees (0-180, 90 = center pulse width).
    void setAngle(uint8_t channel, uint8_t angleDeg);

    // Blocking sweep test: trimmed center -> min -> max -> trimmed center.
    void testSweep(uint8_t channel);

    // Blocking ~6.0s all-4-channel choreography (CH1=S, CH2=E, CH3=N, CH4=W):
    // settle -> FAST compass wave S->E->N->W -> SLOW unison sweep (max -> min
    // -> center) -> FAST synchronized flutter -> final settle. A known
    // "dance" to eyeball before flight -- if it doesn't look right, something's
    // wrong.
    void preflightSequence();

    // Nudge one channel's live position by +-SERVO_TRIM_STEP_US, clamped to
    // [SERVO_MIN_US, SERVO_MAX_US]. Not persisted until saveCalibration() runs.
    void nudge(uint8_t channel, bool positive);

    // Drive all 4 channels to raw SERVO_CENTER_US, ignoring any existing trim.
    // For remounting horns/fins at true electrical center before re-calibrating.
    // Not persisted until saveCalibration() runs.
    void centerAll();

    // Persist the current live position of all 4 channels as their new trim
    // (0 deg reference) to EEPROM. Applied automatically on the next boot.
    void saveCalibration();

private:
    static const uint8_t _pins[4];
    Servo    _servos[4];
    int16_t  _trimUs[4] = {0, 0, 0, 0};   // offset from SERVO_CENTER_US, persisted
    uint16_t _liveUs[4] = {0, 0, 0, 0};   // currently commanded pulse width per channel

    void _loadCalibration();
    void _writeUs(uint8_t idx, int32_t us);

    // Cosmetic ramps (linear, stepped) so choreography looks like intentional
    // motion instead of instant snaps. Purely visual -- no functional need.
    void _rampTo(uint8_t idx, int32_t targetUs, uint32_t durationMs);
    void _rampAllTo(const int32_t targets[4], uint32_t durationMs);
};
