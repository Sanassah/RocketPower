#pragma once
#include <Servo.h>
#include <stdint.h>
#include "../config.h"

// Drives the 4 fin-actuation RC servos (Servo1_PWM..Servo4_PWM,
// HighCurrentComponents.kicad_sch) via the Arduino Servo library -- a custom
// FlexPWM/analogWrite implementation was dropped (bench-confirmed: only 1 of
// 4 channels responded after boot; see ServoTest/ for the isolated re-test
// that cleared the Servo library instead). Each channel's trim (offset from
// SERVO_CENTER_US) is set via ground-station nudge buttons against a printed
// alignment jig and persisted to flash, so fins return to calibrated zero on
// every boot.
class FinController {
public:
    bool begin();    // load trim from EEPROM, attach all 4 servos, drive to trimmed center

    // Fast, direct deflection from the trimmed center (0 = trimmed-neutral,
    // + / - = deflection in each direction, degrees) -- for a real-time
    // control loop (see AttitudeController), not bench-test choreography.
    // Relative to the per-channel trim, and unlike testSweep()/
    // preflightSequence() there's no ramping -- it writes immediately, still
    // hard-clamped to SERVO_MIN_US/SERVO_MAX_US by _writeUs() same as
    // everything else.
    void setCorrectionDeg(uint8_t channel, float correctionDeg);

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

    // Last commanded correction (deg) for one channel, relative to trim,
    // AFTER the hard SERVO_MIN_US/MAX_US clamp in _writeUs() -- i.e. the real
    // value actually written to the servo this loop, not the pre-clamp
    // request. Pure read-only derivation from _liveUs/_trimUs, no state of
    // its own. Used by TelemetryManager's HITL_MODE report (see Packet.h's
    // HITLResponsePacket and Simulation/hitl/) so a closed-loop HITL run
    // sees exactly what the real control code decided, clamping included.
    float liveCorrectionDeg(uint8_t channel) const;

private:
    static const uint8_t _pins[SERVO_NUM_CHANNELS];
    Servo    _servos[SERVO_NUM_CHANNELS];
    int16_t  _trimUs[SERVO_NUM_CHANNELS] = {};   // offset from SERVO_CENTER_US, persisted
    uint16_t _liveUs[SERVO_NUM_CHANNELS] = {};   // currently commanded pulse width per channel

    void _loadCalibration();

    // loUs/hiUs default to the flight-authority SERVO_MIN_US/MAX_US, same as
    // always -- only testSweep() overrides them (to SERVO_RATED_MIN_US/MAX_US)
    // to reach further than flight ever commands for its bench diagnostic.
    void _writeUs(uint8_t idx, int32_t us, int32_t loUs = SERVO_MIN_US, int32_t hiUs = SERVO_MAX_US);

    // This channel's +-SERVO_TEST_SWEEP_ANGLE_DEG travel limits around its
    // trimmed center, in raw microseconds, bounded by SERVO_RATED_MIN_US/
    // MAX_US and clamped to the SAME distance from center both directions
    // (the smaller of the two available margins) so a nonzero trim can't
    // make it reach further one way than the other. Shared by testSweep()
    // and preflightSequence() so both bench diagnostics reach the same,
    // beyond-flight-authority range.
    void _symmetricSweepLimits(uint8_t idx, int32_t& loUs, int32_t& hiUs) const;

    // Same idea as _symmetricSweepLimits, but bounded by the flight-
    // authority SERVO_MIN_US/MAX_US instead of the wider rated range --
    // used by setCorrectionDeg() so real/HITL/demo commands also get
    // matched authority both directions, not just the bench tests above.
    void _symmetricFlightLimits(uint8_t idx, int32_t& loUs, int32_t& hiUs) const;

    // Cosmetic ramps (linear, stepped) so choreography looks like intentional
    // motion instead of instant snaps. Purely visual -- no functional need.
    // loUs/hiUs default to flight-authority SERVO_MIN_US/MAX_US, same as
    // _writeUs(); preflightSequence() overrides them to the wider rated range.
    void _rampTo(uint8_t idx, int32_t targetUs, uint32_t durationMs, int32_t loUs = SERVO_MIN_US, int32_t hiUs = SERVO_MAX_US);
    void _rampAllTo(const int32_t targets[SERVO_NUM_CHANNELS], uint32_t durationMs, int32_t loUs = SERVO_MIN_US, int32_t hiUs = SERVO_MAX_US);
};
