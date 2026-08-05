#pragma once
#include "LoRa.h"
#include "Packet.h"
#include "../sensors/SensorManager.h"

class StateMachine;
class PyroController;
class FinController;
class CameraController;

class TelemetryManager {
public:
    TelemetryManager(LoRaRadio& lora, StateMachine& sm, PyroController& pyro,
                      FinController& fins, CameraController& camera)
        : _lora(lora), _sm(sm), _pyro(pyro), _fins(fins), _camera(camera) {}

    bool begin();

    // Call every loop. Sends telemetry at TELEMETRY_INTERVAL_MS,
    // checks for incoming commands and dispatches them.
    void update(const FlightData& d);

    // Returns true (once) when a CALIBRATE_BARO command arrived.
    // main.cpp polls this and calls sensors.calibrateBaro().
    bool calibrateRequested() { bool r = _calibrateRequested; _calibrateRequested = false; return r; }

private:
    LoRaRadio&        _lora;
    StateMachine&     _sm;
    PyroController&   _pyro;
    FinController&    _fins;
    CameraController& _camera;
    uint32_t        _lastTxMs          = 0;
    bool            _calibrateRequested = false;

    // -1 = no command processed yet (cmd.seq is uint8_t, 0-255, so this
    // sentinel is never reachable by a real command and the first one
    // always executes). Set to the seq of the last command actually
    // executed; a resend with the same seq (because its ack got lost, not
    // because the command itself was lost) is re-acked but not re-run.
    int16_t _lastProcessedSeq = -1;

    void _sendTelemetry(const FlightData& d);
    void _handleCommand(const CommandPacket& cmd);
    void _sendAck(const CommandPacket& cmd);
};
