#pragma once
#include "LoRa.h"
#include "Packet.h"
#include "../sensors/SensorManager.h"

class StateMachine;
class PyroController;
class FinController;
class CameraController;
class DataLogger;
class AttitudeController;

class TelemetryManager {
public:
    TelemetryManager(LoRaRadio& lora, StateMachine& sm, PyroController& pyro,
                      FinController& fins, CameraController& camera, DataLogger& logger,
                      AttitudeController& attitude)
        : _lora(lora), _sm(sm), _pyro(pyro), _fins(fins), _camera(camera), _logger(logger),
          _attitude(attitude) {
        for (size_t i = 0; i < _CMD_TYPE_SLOTS; i++) _lastProcessedSeqByType[i] = -1;
    }

    bool begin();

    // Call every loop. Sends telemetry over LoRa at TELEMETRY_INTERVAL_MS
    // (that cadence is airtime-limited, see config.h) and, independently,
    // mirrors it to USB at the much faster USB_TELEMETRY_INTERVAL_MS when
    // USB_SERIAL_BINARY_MIRROR is on -- a direct USB link has no airtime
    // constraint, so there's no reason to throttle it to the radio's rate.
    // Also checks for incoming commands and dispatches them.
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
    DataLogger&       _logger;
    AttitudeController& _attitude;
    uint32_t        _lastLoraTxMs      = 0;
    uint32_t        _lastUsbTxMs       = 0;
    bool            _calibrateRequested = false;

    // Last-processed seq, tracked PER command type (indexed by the raw
    // CommandType byte) rather than one global value. A single global last-
    // seq is wrong: if command A executes and then a *different* command B
    // executes before A's ack makes it back to the ground, a resend of A
    // (because its ack got lost, not because A itself was lost) no longer
    // matches the global "last seq" -- B's seq does -- so it looks new and
    // gets re-run. Per-type tracking means A's resend is only ever compared
    // against A's own last seq, regardless of what else ran in between.
    // -1 = no command of this type processed yet (cmd.seq is uint8_t,
    // 0-255, so this sentinel is never reachable by a real command).
    static constexpr size_t _CMD_TYPE_SLOTS = 32;   // headroom above the highest CommandType value
    int16_t _lastProcessedSeqByType[_CMD_TYPE_SLOTS];

    TelemetryPacket _buildPacket(const FlightData& d) const;
    void _handleCommand(const CommandPacket& cmd);
    void _sendAck(const CommandPacket& cmd);
};
