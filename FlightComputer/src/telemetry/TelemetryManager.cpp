#include "TelemetryManager.h"
#include "../config.h"
#include "../states/StateMachine.h"
#include "../control/PyroController.h"
#include "../control/FinController.h"
#include "../control/CameraController.h"
#include "../control/AttitudeController.h"
#include "../storage/DataLogger.h"
#include <math.h>

bool TelemetryManager::begin() {
    return _lora.begin();
}

void TelemetryManager::update(const FlightData& d) {
    uint32_t now = millis();

    // LoRa: airtime-limited, see TELEMETRY_INTERVAL_MS in config.h.
    if (now - _lastLoraTxMs >= TELEMETRY_INTERVAL_MS) {
        _lastLoraTxMs = now;
        _lora.send(_buildPacket(d));
    }

#if USB_SERIAL_BINARY_MIRROR
    // USB: no airtime constraint, so this runs on its own, much faster clock
    // instead of piggybacking on the LoRa cadence above. Untouched by
    // HITL_MODE -- still fully usable over the normal Serial/LoRa link at
    // the same time a HITL run is driving the rocket over SerialUSB1 below,
    // so a real GroundStation can watch it live.
    if (now - _lastUsbTxMs >= USB_TELEMETRY_INTERVAL_MS) {
        _lastUsbTxMs = now;
        _lora.sendUsbFast(_buildPacket(d));
    }
#endif

#ifdef HITL_MODE
    // Sent unconditionally, once per loop -- one HITLResponsePacket per
    // SensorInjectPacket SensorManager blocked on this loop (see
    // SensorManager.cpp), on the SAME SerialUSB1 link, so the bridge that's
    // driving this run gets exactly what the real control code just decided.
    {
        HITLResponsePacket resp{};
        resp.magic[0]     = HITL_RESP_MAGIC_0;
        resp.magic[1]     = HITL_RESP_MAGIC_1;
        resp.timestamp_ms = d.timestamp_ms;
        resp.state        = static_cast<uint8_t>(d.state);
        resp.fin_deg[0]   = _fins.liveCorrectionDeg(1);
        resp.fin_deg[1]   = _fins.liveCorrectionDeg(2);
        resp.fin_deg[2]   = _fins.liveCorrectionDeg(3);
        resp.fin_deg[3]   = _fins.liveCorrectionDeg(4);
        resp.attitude_status = (_attitude.controlEnabled() ? ATTITUDE_STATUS_CONTROL_ON : 0)
                              | (_attitude.demoEnabled()    ? ATTITUDE_STATUS_DEMO_ON    : 0);
        resp.checksum = packetChecksum(reinterpret_cast<const uint8_t*>(&resp), sizeof(resp) - 2);
        SerialUSB1.write(reinterpret_cast<const uint8_t*>(&resp), sizeof(resp));
    }
#endif

    // Check for incoming commands — USB has priority when a PC is connected
    CommandPacket cmd;
    bool gotCmd = false;
    if ((bool)Serial && _lora.receiveCommandFrom(Serial, cmd)) {
        Serial.println("[TELEM] CMD source: USB");
        gotCmd = true;
    } else if (_lora.receiveCommand(cmd)) {
        gotCmd = true;
    }

    if (gotCmd) {
        // Ack first, before executing -- so even a slow/blocking command
        // (e.g. SERVO_PREFLIGHT) is confirmed promptly and the ground stops
        // retrying right away instead of retrying into an in-progress command.
        _sendAck(cmd);

        uint8_t typeIdx = (uint8_t)cmd.type;
        bool isDuplicate = (typeIdx < _CMD_TYPE_SLOTS) &&
                            ((int16_t)cmd.seq == _lastProcessedSeqByType[typeIdx]);

        // Single-line, always-printed record of every command RX -- easy to
        // scan/grep for exactly what arrived and whether it actually ran.
        Serial.print("[CMD RX] seq="); Serial.print(cmd.seq);
        Serial.print(" type=0x"); Serial.print(typeIdx, HEX);
        Serial.print(" param="); Serial.print(cmd.param);
        Serial.println(isDuplicate ? "  -> DUPLICATE (re-acked only, not re-run)" : "  -> NEW (executing)");

        if (!isDuplicate) {
            if (typeIdx < _CMD_TYPE_SLOTS) _lastProcessedSeqByType[typeIdx] = cmd.seq;
            _handleCommand(cmd);
        }
    }
}

void TelemetryManager::_sendAck(const CommandPacket& cmd) {
    AckPacket ack{};
    ack.magic[0] = ACK_MAGIC_0;
    ack.magic[1] = ACK_MAGIC_1;
    ack.cmdSeq   = cmd.seq;
    ack.cmdType  = cmd.type;
    _lora.sendAck(ack);
}

TelemetryPacket TelemetryManager::_buildPacket(const FlightData& d) const {
    TelemetryPacket pkt{};
    pkt.magic[0]      = TELEM_MAGIC_0;
    pkt.magic[1]      = TELEM_MAGIC_1;
    pkt.timestamp_ms  = d.timestamp_ms;
    pkt.state         = static_cast<uint8_t>(d.state);
    pkt.lat           = (float)d.lat;
    pkt.lon           = (float)d.lon;
    pkt.gps_alt_dm    = (int16_t)lroundf(d.gps_alt_m * 10.0f);
    pkt.gps_sats      = d.gps_sats;
    pkt.gps_fix       = d.gps_fix ? 1 : 0;
    pkt.baro_alt_dm   = (int16_t)lroundf(d.baro_alt_m * 10.0f);
    pkt.vert_vel_cms  = (int16_t)lroundf(d.vert_vel_ms * 100.0f);
    pkt.accel_x_cg    = (int16_t)lroundf(d.highg_x_g * 100.0f);
    pkt.accel_y_cg    = (int16_t)lroundf(d.highg_y_g * 100.0f);
    pkt.accel_z_cg    = (int16_t)lroundf(d.highg_z_g * 100.0f);
    pkt.quat_w_i16    = (int16_t)lroundf(d.quat_w * 32767.0f);
    pkt.quat_x_i16    = (int16_t)lroundf(d.quat_x * 32767.0f);
    pkt.quat_y_i16    = (int16_t)lroundf(d.quat_y * 32767.0f);
    pkt.quat_z_i16    = (int16_t)lroundf(d.quat_z * 32767.0f);
    pkt.voltage_cv    = (int16_t)lroundf(d.voltage_v * 100.0f);
    pkt.current_ma    = (int16_t)lroundf(d.current_ma);
    pkt.rssi          = (int8_t)_lora.getRSSI();

    // Read continuity for each pyro channel so the ground station shows green/red
    pkt.pyro_cont[0]  = _pyro.continuityOk(1) ? 1 : 0;
    pkt.pyro_cont[1]  = _pyro.continuityOk(2) ? 1 : 0;
    pkt.pyro_cont[2]  = _pyro.continuityOk(3) ? 1 : 0;

    pkt.cam_recording = _camera.isRecording() ? 1 : 0;

    pkt.system_status = (d.imu_ok   ? SENSOR_HEALTH_IMU_OK   : 0)
                       | (d.baro_ok  ? SENSOR_HEALTH_BARO_OK  : 0)
                       | (d.accel_ok ? SENSOR_HEALTH_ACCEL_OK : 0)
                       | (d.gps_ok   ? SENSOR_HEALTH_GPS_OK   : 0)
                       | (d.power_ok ? SENSOR_HEALTH_POWER_OK : 0)
                       | (_logger.cardPresent() ? SD_STATUS_PRESENT   : 0)
                       | (_logger.isOpen()      ? SD_STATUS_RECORDING : 0);

    pkt.attitude_status = (_attitude.controlEnabled() ? ATTITUDE_STATUS_CONTROL_ON : 0)
                         | (_attitude.demoEnabled()    ? ATTITUDE_STATUS_DEMO_ON    : 0);

    return pkt;
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

        case CommandType::CALIBRATE_BARO:
            Serial.println("[TELEM] CMD: CALIBRATE_BARO");
            _calibrateRequested = true;
            break;

        case CommandType::SERVO_TEST:
            Serial.print("[TELEM] CMD: SERVO_TEST ch="); Serial.println(cmd.param);
            _fins.testSweep(cmd.param);
            break;

        case CommandType::CAM_TOGGLE:
            Serial.println("[TELEM] CMD: CAM_TOGGLE");
            _camera.toggleRecording();
            break;

        case CommandType::SERVO_NUDGE_POS:
            _fins.nudge(cmd.param, true);
            break;

        case CommandType::SERVO_NUDGE_NEG:
            _fins.nudge(cmd.param, false);
            break;

        case CommandType::SERVO_SAVE_CAL:
            Serial.println("[TELEM] CMD: SERVO_SAVE_CAL");
            _fins.saveCalibration();
            break;

        case CommandType::SERVO_CENTER_ALL:
            Serial.println("[TELEM] CMD: SERVO_CENTER_ALL");
            _fins.centerAll();
            break;

        case CommandType::SERVO_PREFLIGHT:
            Serial.println("[TELEM] CMD: SERVO_PREFLIGHT");
            _fins.preflightSequence();
            break;

        case CommandType::SD_START_RECORDING:
            Serial.println("[TELEM] CMD: SD_START_RECORDING");
            if (!_logger.open()) Serial.println("[TELEM] SD_START_RECORDING failed -- no card?");
            break;

        case CommandType::SD_STOP_RECORDING:
            Serial.println("[TELEM] CMD: SD_STOP_RECORDING");
            _logger.close();
            break;

        case CommandType::ATTITUDE_CONTROL_ENABLE:
            Serial.println("[TELEM] CMD: ATTITUDE_CONTROL_ENABLE");
            _attitude.setControlEnabled(true);
            break;

        case CommandType::ATTITUDE_CONTROL_DISABLE:
            Serial.println("[TELEM] CMD: ATTITUDE_CONTROL_DISABLE");
            _attitude.setControlEnabled(false);
            break;

        case CommandType::ATTITUDE_DEMO_ENABLE:
            Serial.println("[TELEM] CMD: ATTITUDE_DEMO_ENABLE");
            _attitude.setDemoEnabled(true);
            break;

        case CommandType::ATTITUDE_DEMO_DISABLE:
            Serial.println("[TELEM] CMD: ATTITUDE_DEMO_DISABLE");
            _attitude.setDemoEnabled(false);
            break;

        case CommandType::RESET:
            Serial.println("[TELEM] CMD: RESET");
            _resetRequested = true;
            break;

        case CommandType::PING:
            // Response will be sent next telemetry cycle
            break;

        default:
            Serial.println("[TELEM] CMD: unknown");
            break;
    }
}
