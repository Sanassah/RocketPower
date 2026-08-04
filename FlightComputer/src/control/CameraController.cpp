#include "CameraController.h"
#include "../config.h"
#include <Arduino.h>

// RunCam Device Protocol (RCDevice), used by Split-style cameras over UART.
// Frame: [0xCC][command][data...][crc8], crc8 = CRC-8/DVB-S2 (poly 0xD5) over
// all preceding bytes. Camera control (command 0x01) takes a 1-byte op code;
// the op code -- not the command byte -- is what selects start/stop recording.
#define RUNCAM_HEADER              0xCC
#define RUNCAM_CMD_CAMERA_CONTROL  0x01
#define RUNCAM_OP_START_RECORDING  0x03
#define RUNCAM_OP_STOP_RECORDING   0x04

bool CameraController::begin() {
    CAM_SERIAL.begin(CAM_BAUD);
    _recording = false;
    return true;
}

void CameraController::onStateChange(FlightState prev, FlightState next) {
    (void)prev;
    if (next == FlightState::ARMED) {
        startRecording();
    } else if (next == FlightState::LANDED) {
        stopRecording();
    }
}

void CameraController::startRecording() {
    if (_recording) return;
    _sendCameraControl(RUNCAM_OP_START_RECORDING);
    _recording = true;
    Serial.println("[CAM] Recording started");
}

void CameraController::stopRecording() {
    if (!_recording) return;
    _sendCameraControl(RUNCAM_OP_STOP_RECORDING);
    _recording = false;
    Serial.println("[CAM] Recording stopped");
}

void CameraController::_sendCameraControl(uint8_t op) {
    uint8_t frame[4];
    frame[0] = RUNCAM_HEADER;
    frame[1] = RUNCAM_CMD_CAMERA_CONTROL;
    frame[2] = op;
    // CRC8/DVB-S2 over bytes 0..2
    uint8_t crc = 0;
    for (int i = 0; i < 3; i++) {
        crc ^= frame[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    frame[3] = crc;
    CAM_SERIAL.write(frame, sizeof(frame));
}
