#pragma once
#include "../states/FlightStates.h"

// Controls RunCam-compatible camera over Serial7 (UART).
// Cam_TX→pin28 (Serial7 RX), Cam_RX→pin29 (Serial7 TX).
// Starts recording on ARMED, stops on LANDED.
class CameraController {
public:
    bool begin();
    void onStateChange(FlightState prev, FlightState next);

    // Manual trigger for bench testing / ground-station override. The
    // camera only exposes a toggle (see CameraController.cpp), so this
    // always flips whatever we believe the current state is.
    void toggleRecording();

    bool isRecording() const { return _recording; }

private:
    bool _recording = false;

    void _sendCameraControl(uint8_t op);

    // Sends GET_DEVICE_INFO and logs whatever comes back (or "no response")
    // to USB Serial, once at boot -- a cheap, silent confirmation that the
    // UART link to the camera is physically alive.
    void _probeDeviceInfo();
};
