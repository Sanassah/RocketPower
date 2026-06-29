#include "CameraController.h"
#include "../config.h"
#include <Arduino.h>

// RunCam device protocol v2: 5-byte frame
// [0xCC] [COMMAND] [CRC]
// (simplified — full protocol has address byte and proper CRC8)
#define RUNCAM_HEADER   0xCC
#define RUNCAM_START_REC 0x01
#define RUNCAM_STOP_REC  0x02

bool CameraController::begin() {
    CAM_SERIAL.begin(CAM_BAUD);
    _recording = false;
    return true;
}

void CameraController::onStateChange(FlightState prev, FlightState next) {
    (void)prev;
    if (next == FlightState::ARMED && !_recording) {
        _startRecording();
    } else if (next == FlightState::LANDED && _recording) {
        _stopRecording();
    }
}

void CameraController::_startRecording() {
    _sendCommand(RUNCAM_START_REC);
    _recording = true;
    Serial.println("[CAM] Recording started");
}

void CameraController::_stopRecording() {
    _sendCommand(RUNCAM_STOP_REC);
    _recording = false;
    Serial.println("[CAM] Recording stopped");
}

void CameraController::_sendCommand(uint8_t action) {
    // RunCam Device Protocol v2 frame (minimal implementation)
    uint8_t frame[3];
    frame[0] = RUNCAM_HEADER;
    frame[1] = action;
    // CRC8/DVB-S2 over bytes 0..1
    uint8_t crc = 0;
    for (int i = 0; i < 2; i++) {
        crc ^= frame[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    frame[2] = crc;
    CAM_SERIAL.write(frame, sizeof(frame));
}
