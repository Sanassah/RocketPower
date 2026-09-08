#include "CameraController.h"
#include "../config.h"
#include <Arduino.h>

// RunCam Device Protocol (RCDevice), used by Split-style cameras over UART.
// Frame: [0xCC][command][data...][crc8], crc8 = CRC-8/DVB-S2 (poly 0xD5) over
// all preceding bytes. Camera control (command 0x01) takes a 1-byte op code.
#define RUNCAM_HEADER                 0xCC
#define RUNCAM_CMD_GET_DEVICE_INFO    0x00
#define RUNCAM_CMD_CAMERA_CONTROL     0x01
// This camera's firmware doesn't act on the explicit START_RECORDING (0x03) /
// STOP_RECORDING (0x04) sub-ops documented for some RunCam firmware -- op 0x01
// (bench-confirmed: sent twice, started then stopped recording) is the only
// thing this unit reacts to, and it's a toggle, not an absolute state -- so
// there's no hardware ack telling us which way it actually went. _recording
// is only ever our own best guess, kept in sync as long as nothing else
// (the physical button, a full SD card, ...) touches the camera.
#define RUNCAM_OP_TOGGLE_RECORDING    0x01

static uint8_t crc8Dvb(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    return crc;
}

bool CameraController::begin() {
    CAM_SERIAL.begin(CAM_BAUD);
    _recording = false;
    _probeDeviceInfo();
    return true;
}

void CameraController::onStateChange(FlightState prev, FlightState next) {
    (void)prev;
    if (next == FlightState::ARMED && !_recording) {
        toggleRecording();
    } else if (next == FlightState::LANDED && _recording) {
        toggleRecording();
    }
}

void CameraController::toggleRecording() {
    _sendCameraControl(RUNCAM_OP_TOGGLE_RECORDING);
    _recording = !_recording;
    DEBUG_SERIAL.println(_recording ? "[CAM] Recording started" : "[CAM] Recording stopped");
}

void CameraController::_sendCameraControl(uint8_t op) {
    uint8_t frame[4];
    frame[0] = RUNCAM_HEADER;
    frame[1] = RUNCAM_CMD_CAMERA_CONTROL;
    frame[2] = op;
    frame[3] = crc8Dvb(frame, 3);
    CAM_SERIAL.write(frame, sizeof(frame));
    CAM_SERIAL.flush();
}

void CameraController::_probeDeviceInfo() {
    while (CAM_SERIAL.available()) CAM_SERIAL.read();

    uint8_t frame[3] = { RUNCAM_HEADER, RUNCAM_CMD_GET_DEVICE_INFO, 0 };
    frame[2] = crc8Dvb(frame, 2);
    CAM_SERIAL.write(frame, sizeof(frame));
    CAM_SERIAL.flush();

    uint8_t reply[16];
    uint8_t n = 0;
    uint32_t startMs = millis();
    while (millis() - startMs < 200 && n < sizeof(reply)) {
        if (CAM_SERIAL.available()) {
            reply[n++] = CAM_SERIAL.read();
        }
    }

    if (n == 0) {
        DEBUG_SERIAL.println("[CAM] Probe: NO RESPONSE from camera -- check UART wiring/power.");
    } else {
        DEBUG_SERIAL.print("[CAM] Probe: got "); DEBUG_SERIAL.print(n); DEBUG_SERIAL.print(" byte(s): ");
        for (uint8_t i = 0; i < n; i++) {
            if (reply[i] < 0x10) DEBUG_SERIAL.print('0');
            DEBUG_SERIAL.print(reply[i], HEX);
            DEBUG_SERIAL.print(' ');
        }
        DEBUG_SERIAL.println();
    }
}
