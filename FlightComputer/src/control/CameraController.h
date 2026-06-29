#pragma once
#include "../states/FlightStates.h"

// Controls RunCam-compatible camera over Serial7 (UART).
// Cam_TX→pin28 (Serial7 RX), Cam_RX→pin29 (Serial7 TX).
// Starts recording on ARMED, stops on LANDED.
class CameraController {
public:
    bool begin();
    void onStateChange(FlightState prev, FlightState next);

    bool isRecording() const { return _recording; }

private:
    bool _recording = false;

    void _startRecording();
    void _stopRecording();

    // RunCam protocol: 5-byte frame
    void _sendCommand(uint8_t action);
};
