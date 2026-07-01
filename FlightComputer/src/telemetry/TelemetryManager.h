#pragma once
#include "LoRa.h"
#include "Packet.h"
#include "../sensors/SensorManager.h"

class StateMachine;
class PyroController;

class TelemetryManager {
public:
    TelemetryManager(LoRaRadio& lora, StateMachine& sm, PyroController& pyro)
        : _lora(lora), _sm(sm), _pyro(pyro) {}

    bool begin();

    // Call every loop. Sends telemetry at TELEMETRY_INTERVAL_MS,
    // checks for incoming commands and dispatches them.
    void update(const FlightData& d);

    // Returns true (once) when a CALIBRATE_BARO command arrived.
    // main.cpp polls this and calls sensors.calibrateBaro().
    bool calibrateRequested() { bool r = _calibrateRequested; _calibrateRequested = false; return r; }

private:
    LoRaRadio&      _lora;
    StateMachine&   _sm;
    PyroController& _pyro;
    uint32_t        _lastTxMs          = 0;
    bool            _calibrateRequested = false;

    void _sendTelemetry(const FlightData& d);
    void _handleCommand(const CommandPacket& cmd);
};
