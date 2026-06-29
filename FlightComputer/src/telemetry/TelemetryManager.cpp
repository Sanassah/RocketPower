#include "TelemetryManager.h"
#include "../config.h"
#include "../states/StateMachine.h"
#include "../control/PyroController.h"

bool TelemetryManager::begin() {
    return _lora.begin();
}

void TelemetryManager::update(const FlightData& d) {
    uint32_t now = millis();

    // Send telemetry at configured interval
    if (now - _lastTxMs >= TELEMETRY_INTERVAL_MS) {
        _lastTxMs = now;
        _sendTelemetry(d);
    }

    // Check for incoming commands
    CommandPacket cmd;
    if (_lora.receiveCommand(cmd)) {
        _handleCommand(cmd);
    }
}

void TelemetryManager::_sendTelemetry(const FlightData& d) {
    TelemetryPacket pkt{};
    pkt.magic[0]      = TELEM_MAGIC_0;
    pkt.magic[1]      = TELEM_MAGIC_1;
    pkt.timestamp_ms  = d.timestamp_ms;
    pkt.state         = static_cast<uint8_t>(d.state);
    pkt.lat           = d.lat;
    pkt.lon           = d.lon;
    pkt.gps_alt_m     = d.gps_alt_m;
    pkt.gps_sats      = d.gps_sats;
    pkt.gps_fix       = d.gps_fix ? 1 : 0;
    pkt.baro_alt_m    = d.baro_alt_m;
    pkt.vert_vel_ms   = d.vert_vel_ms;
    pkt.accel_x_g     = d.highg_x_g;
    pkt.accel_y_g     = d.highg_y_g;
    pkt.accel_z_g     = d.highg_z_g;
    pkt.quat_w        = d.quat_w;
    pkt.quat_x        = d.quat_x;
    pkt.quat_y        = d.quat_y;
    pkt.quat_z        = d.quat_z;
    pkt.voltage_v     = d.voltage_v;
    pkt.current_ma    = d.current_ma;
    pkt.rssi          = (int8_t)_lora.getRSSI();

    _lora.send(pkt);
}

void TelemetryManager::_handleCommand(const CommandPacket& cmd) {
    switch (cmd.type) {
        case CommandType::ARM:
            Serial.println("[TELEM] CMD: ARM");
            _sm.onArm();
            break;

        case CommandType::DISARM:
            Serial.println("[TELEM] CMD: DISARM");
            _sm.onDisarm();
            break;

        case CommandType::FIRE_PYRO:
            Serial.print("[TELEM] CMD: FIRE_PYRO ch="); Serial.println(cmd.param);
            _pyro.fire(cmd.param);   // PyroController enforces armed + continuity checks
            break;

        case CommandType::PING:
            // Response will be sent next telemetry cycle
            break;

        default:
            Serial.println("[TELEM] CMD: unknown");
            break;
    }
}
