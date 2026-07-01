#pragma once
#include <stdint.h>

// Controls the 3 pyro channels on the custom MIMXRT1062 board.
// From schematic (HighCurrentComponents.kicad_sch):
//   CH1 (ignition):  fire=pin2, continuity=pin40
//   CH2 (parachute): fire=pin3, continuity=pin41
//   CH3 (backup):    fire=pin4, continuity=pin39
class PyroController {
public:
    bool begin();    // configure GPIO directions

    void arm();      // enable pyro logic (software arm)
    void disarm();   // disable pyro logic

    // Fire a channel for PYRO_FIRE_DURATION_MS ms.
    // Refuses to fire if not armed or continuity check fails.
    bool fire(uint8_t channel);

    // Returns true if continuity detected on channel (1-3)
    bool continuityOk(uint8_t channel) const;

    bool isArmed() const { return _armed; }

private:
    bool _armed = false;

    static const uint8_t _firePins[3];
    static const uint8_t _contPins[3];

    bool _safetyCheck(uint8_t ch) const;
};
