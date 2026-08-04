#pragma once
#include "../states/FlightStates.h"

// Controls RunCam-compatible camera over Serial7 (UART).
// Cam_TX→pin28 (Serial7 RX), Cam_RX→pin29 (Serial7 TX).
// Starts recording on ARMED, stops on LANDED.
class CameraController {
public:
    bool begin();
    void onStateChange(FlightState prev, FlightState next);

    // Manual triggers for bench testing / ground-station override.
    // Both are no-ops if already in the requested state.
    void startRecording();
    void stopRecording();

    bool isRecording() const { return _recording; }

private:
    bool _recording = false;

    void _sendCameraControl(uint8_t op);
};
