#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <elapsedMillis.h>

#include "config.h"
#include "sensors/SensorManager.h"
#include "states/StateMachine.h"
#include "telemetry/LoRa.h"
#include "telemetry/TelemetryManager.h"
#include "storage/DataLogger.h"
#include "control/PyroController.h"
#include "control/CameraController.h"
#include "control/FinController.h"
#include "control/AttitudeController.h"
#include "control/StatusLED.h"
#include "safety/BackupDeploy.h"

// ===== Global objects =====
SensorManager      sensors;
PyroController     pyro;
StateMachine       fsm(pyro);
LoRaRadio          radio;
FinController      fins;
AttitudeController attitude;
CameraController   camera;
DataLogger         logger;
TelemetryManager   telem(radio, fsm, pyro, fins, camera, logger, attitude);
StatusLED          statusLed;
// Independent of fsm/pyro's armed state on purpose -- see BackupDeploy.h.
BackupDeploy        backupDeploy;

// ===== Timing =====
elapsedMillis loopTimer;     // tracks time since last loop start
uint32_t      loopCount    = 0;
uint32_t      loopMaxUs    = 0;
FlightState   prevState    = FlightState::IDLE;

// TEMPORARY diagnostic -- per-stage breakdown of loopMaxUs above, so a
// bench run pinpoints WHICH stage is stalling instead of just confirming
// that something is. See SensorManager::update()'s own breakdown for a
// further split of the sensors stage specifically (I2C reads across 5
// sensors on 3 Wire buses, one of which -- Wire1 -- is shared by baro/gps/
// power). Remove once the actual stall source is found and fixed.
uint32_t maxSensorsUs = 0, maxControlUs = 0, maxLogUs = 0, maxTelemUs = 0;

// ===== Helpers =====
static void printSensorReport(bool sdOk) {
    const FlightData& d = sensors.data();
    DEBUG_SERIAL.println("--- Sensor init report ---");
    DEBUG_SERIAL.print("  IMU   (BNO085 Wire  0x4A): "); DEBUG_SERIAL.println(d.imu_ok   ? "OK" : "FAIL");
    DEBUG_SERIAL.print("  Baro  (BMP390 Wire1 0x77): "); DEBUG_SERIAL.println(d.baro_ok  ? "OK" : "FAIL");
    DEBUG_SERIAL.print("  Accel (ADXL375 Wire2 0x1D): "); DEBUG_SERIAL.println(d.accel_ok ? "OK" : "FAIL");
    DEBUG_SERIAL.print("  GPS   (ZOEM8  Wire1 0x42): "); DEBUG_SERIAL.println(d.gps_ok   ? "OK" : "FAIL");
    DEBUG_SERIAL.print("  Power (INA260 Wire1 0x40): "); DEBUG_SERIAL.println(d.power_ok  ? "OK" : "FAIL");
    DEBUG_SERIAL.print("  SD    (BUILTIN_SDCARD):    "); DEBUG_SERIAL.println(sdOk ? "OK" : "FAIL");
    DEBUG_SERIAL.println("--------------------------");
}

// =================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);

    DEBUG_SERIAL.println("\n=================================");
    DEBUG_SERIAL.println(" RocketFlightComputer v0.1");
    DEBUG_SERIAL.println(" Custom MIMXRT1062 / Teensy 4.1");
    DEBUG_SERIAL.println("=================================\n");

    // ---- Status LEDs ----
    statusLed.begin();

    // ---- Sensors ----
    DEBUG_SERIAL.println("[INIT] Sensors...");
    bool sensorsOk = sensors.begin();

    // ---- SD card ---- (before the report below, so it reflects the real attempt --
    // logger.open() is self-contained and probes the card itself)
    DEBUG_SERIAL.println("[INIT] SD card...");
    if (!logger.open()) {
        DEBUG_SERIAL.println("[INIT] WARNING: SD card failed. Logging disabled.");
    }

    printSensorReport(logger.cardPresent());
    statusLed.update(sensors.data());
    if (!sensorsOk) {
        DEBUG_SERIAL.println("[INIT] CRITICAL: IMU or barometer failed. Halting.");
        while (true) { statusLed.update(sensors.data()); delay(50); }
    }

    // ---- Calibrate barometer at launch site ----
    DEBUG_SERIAL.println("[INIT] Calibrating barometer (2s)...");
    sensors.calibrateBaro();
    DEBUG_SERIAL.println("[INIT] Baro calibrated.");

    // ---- Pyro ----
    DEBUG_SERIAL.println("[INIT] Pyro channels...");
    pyro.begin();
    for (int ch = 1; ch <= PYRO_NUM_CHANNELS; ch++) {
        DEBUG_SERIAL.print("  CH"); DEBUG_SERIAL.print(ch);
        DEBUG_SERIAL.print(" continuity: ");
        DEBUG_SERIAL.println(pyro.continuityOk(ch) ? "OK" : "OPEN (no ematch?)");
    }

    // ---- LoRa / Telemetry ----
    DEBUG_SERIAL.println("[INIT] LoRa radio...");
    if (!telem.begin()) {
        DEBUG_SERIAL.println("[INIT] WARNING: LoRa init failed.");
    }

    // ---- Camera ----
    camera.begin();

    // ---- Fin servos ----
    DEBUG_SERIAL.println("[INIT] Fin servos...");
    fins.begin();

    DEBUG_SERIAL.println("\n[INIT] Boot complete. State: IDLE\n");
    loopTimer = 0;
}

// =================================================================
void loop() {
    uint32_t loopStart = micros();

    // ---- 1. Read all sensors ----
    sensors.update();
    sensors.setState(fsm.state());   // stamp this loop's state onto the snapshot for logging/telemetry
    const FlightData& d = sensors.data();
    statusLed.update(d);
    pyro.update();   // non-blocking fire-pin timeout -- see PyroController.h
    uint32_t tSensors = micros();   // TEMPORARY diagnostic, see maxSensorsUs above

    // ---- 2. Update state machine ----
    fsm.update(d);
    FlightState newState = fsm.state();

    // ---- 2a. Independent backup apogee monitor ----
    // Deliberately unconditional -- no state or armed-flag check here. See
    // BackupDeploy.h for why: if fsm/pyro's state got reset mid-flight
    // (neither survives a reboot), this is the only thing that can still
    // notice the rocket already went through a real boost and reached
    // apogee, and fire the backup channel anyway.
    backupDeploy.update(d, pyro);

    // ---- 2b. Active attitude control ----
    // Two independent reasons to run this, from two independent ground-
    // commanded runtime flags -- see AttitudeController.h for why they're
    // kept separate:
    //   - Real in-flight control (attitude.controlEnabled()): only
    //     meaningful with real airflow over the fins, so POWERED_ASCENT/
    //     COAST only -- not before liftoff, not once the chute's out.
    //   - Ground demo/bench-validation (attitude.demoEnabled()): IDLE or
    //     ARMED, for hand-rotating the airframe and watching the fin
    //     response -- deliberately doesn't require ARMED (pyro doesn't need
    //     to be hot just to watch the fins move), and never runs once real
    //     liftoff moves the state machine past ARMED either way.
    // A no-op entirely when both are false (the default at every boot).
    bool wantAttitudeControl =
        (attitude.controlEnabled() && (newState == FlightState::POWERED_ASCENT ||
                                        newState == FlightState::COAST)) ||
        (attitude.demoEnabled()    && (newState == FlightState::IDLE ||
                                        newState == FlightState::ARMED));
    if (wantAttitudeControl) {
        attitude.update(d, fins);
    }

    // ---- 2c. Raw gyro + absolute tilt + fin-response debug print, DEMO only ----
    // Exists for exactly one purpose: the bench axis-mapping/allocation-sign
    // calibration procedure in config.h/AttitudeController.h. gyro_x/y/z
    // aren't in TelemetryPacket at all (kept out deliberately -- see its
    // size comment), so there's otherwise no live way to see them.
    //
    // The angle terms are the same exact tilt-from-true-vertical computation
    // AttitudeController::update() now uses for its own roll/pitch angle
    // error (atan2/asin off the absolute quaternion via gravity -- no
    // reference latch, see AttitudeController.cpp) -- computed fresh here
    // too rather than read back from the controller, since this print needs
    // to work even when it isn't the one driving the fins. Labeled by raw
    // sensor axis (X/Y), not N/S/E/W or roll/pitch -- figuring out which raw
    // axis IS which compass direction is exactly what hand-tilting the
    // airframe toward each direction and watching these numbers move is for.
    //
    // Gated on demoEnabled() specifically (not wantAttitudeControl, which is
    // also true during real controlEnabled() flight) so this never fires
    // outside an explicit bench session, and rate-limited so it's readable
    // at a glance while slowly rotating the airframe by hand, not a wall of
    // scrolling text. The binary USB telemetry mirror self-suppresses while
    // demoEnabled() (see TelemetryManager.cpp), so this prints clean in a
    // plain serial monitor without any config.h change needed.
    if (attitude.demoEnabled()) {
        static uint32_t lastGyroPrintMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastGyroPrintMs >= 200) {
            lastGyroPrintMs = nowMs;

            constexpr float kRadToDeg = 57.29577951308232f;
            float qw = d.quat_w, qx = d.quat_x, qy = d.quat_y, qz = d.quat_z;
            float sinTiltY = 2.0f * (qw * qy - qz * qx);
            sinTiltY = sinTiltY > 1.0f ? 1.0f : (sinTiltY < -1.0f ? -1.0f : sinTiltY);
            float tiltXdeg = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * kRadToDeg;
            float tiltYdeg = asinf(sinTiltY) * kRadToDeg;
            float spinZdeg = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * kRadToDeg;

            Serial.print("[AXIS CAL] gyro rad/s  x=");
            Serial.print(d.gyro_x, 3);
            Serial.print("  y=");
            Serial.print(d.gyro_y, 3);
            Serial.print("  z=");
            Serial.print(d.gyro_z, 3);
            Serial.print("   |   tilt deg (absolute, from level)  X=");
            Serial.print(tiltXdeg, 1);
            Serial.print(" Y=");
            Serial.print(tiltYdeg, 1);
            Serial.print(" spin=");
            Serial.print(spinZdeg, 1);
            Serial.print("   |   fin deg  S=");
            Serial.print(fins.liveCorrectionDeg(1), 1);
            Serial.print(" E=");
            Serial.print(fins.liveCorrectionDeg(2), 1);
            Serial.print(" N=");
            Serial.print(fins.liveCorrectionDeg(3), 1);
            Serial.print(" W=");
            Serial.println(fins.liveCorrectionDeg(4), 1);
        }
    }

    // ---- 3. Handle state transitions ----
    if (newState != prevState) {
        camera.onStateChange(prevState, newState);
        logger.flush();   // force flush on state change to minimise data loss

        // Arm pyro on ARM, disarm on IDLE or LANDED
        if (newState == FlightState::ARMED) {
            pyro.arm();
            // Re-calibrate baro at the actual launch site, not wherever the FCC booted.
            // The rocket should be stationary on the pad when ARM is sent.
            DEBUG_SERIAL.println("[CALIB] Re-calibrating baro at launch site...");
            sensors.calibrateBaro();
            DEBUG_SERIAL.println("[CALIB] Done.");
        }
        if (newState == FlightState::IDLE || newState == FlightState::LANDED) {
            pyro.disarm();
            attitude.reset();   // force both attitude modes back off, nothing carries between sessions
        }

        // Close log after landing
        if (newState == FlightState::LANDED) logger.close();

        prevState = newState;
    }
    uint32_t tControl = micros();   // TEMPORARY diagnostic, see maxSensorsUs above

    // ---- 4. Log data ----
    logger.update(d);
    uint32_t tLog = micros();   // TEMPORARY diagnostic, see maxSensorsUs above

    // ---- 5. Telemetry (send + receive commands) ----
    telem.update(d);
    uint32_t tTelem = micros();   // TEMPORARY diagnostic, see maxSensorsUs above

    // ---- 5b. Execute calibration if requested by ground station ----
    if (telem.calibrateRequested()) {
        DEBUG_SERIAL.println("[CALIB] Calibrating barometer...");
        sensors.calibrateBaro();
        DEBUG_SERIAL.println("[CALIB] Done.");
    }

    // ---- 5c. Soft reset if requested by ground station ----
    // Fresh-boot-equivalent IDLE without an actual power cycle -- see
    // Packet.h's RESET for why this exists (repeated bench/HITL test
    // flights). Touches every piece of RAM-only flight/detection state this
    // file itself owns references to; TelemetryManager only flags the
    // request since it doesn't hold a BackupDeploy reference.
    if (telem.resetRequested()) {
        DEBUG_SERIAL.println("[RESET] Soft reset requested -- clearing all flight/detection state.");
        fsm.reset();
        pyro.disarm();
        attitude.reset();
        backupDeploy.reset();
        if (camera.isRecording()) camera.toggleRecording();
        logger.close();
        if (logger.cardPresent()) logger.open();   // fresh log file for the next run, same as a real reboot would get
        prevState = FlightState::IDLE;             // keep this loop's own change-detection var in sync with fsm's new state
        DEBUG_SERIAL.println("[RESET] Done. State: IDLE");
    }

    // ---- 6. Loop timing diagnostics ----
    uint32_t elapsed = micros() - loopStart;
    if (elapsed > loopMaxUs) loopMaxUs = elapsed;
    loopCount++;

    // TEMPORARY diagnostic -- see maxSensorsUs above.
    uint32_t sensorsUs = tSensors - loopStart;
    uint32_t controlUs = tControl - tSensors;
    uint32_t logUs     = tLog     - tControl;
    uint32_t telemUs    = tTelem   - tLog;
    if (sensorsUs > maxSensorsUs) maxSensorsUs = sensorsUs;
    if (controlUs > maxControlUs) maxControlUs = controlUs;
    if (logUs     > maxLogUs)     maxLogUs     = logUs;
    if (telemUs   > maxTelemUs)   maxTelemUs   = telemUs;

    // Print loop stats every 5 seconds
    static uint32_t lastStatMs = 0;
    if (millis() - lastStatMs >= 5000) {
        lastStatMs = millis();
        float hz = loopCount / 5.0f;
        // Printed in ms (3 decimals = full microsecond precision, just
        // relabeled -- e.g. 6952us prints as 6.952ms) rather than raw us.
        DEBUG_SERIAL.print("[LOOP] "); DEBUG_SERIAL.print(hz, 1); DEBUG_SERIAL.print(" Hz | ");
        DEBUG_SERIAL.print("max="); DEBUG_SERIAL.print(loopMaxUs / 1000.0f, 3); DEBUG_SERIAL.print("ms | ");
        DEBUG_SERIAL.print("state="); DEBUG_SERIAL.println(flightStateName(fsm.state()));
        DEBUG_SERIAL.print("[LOOP BREAKDOWN] sensors="); DEBUG_SERIAL.print(maxSensorsUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms control="); DEBUG_SERIAL.print(maxControlUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms log="); DEBUG_SERIAL.print(maxLogUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms telem="); DEBUG_SERIAL.print(maxTelemUs / 1000.0f, 3);
        DEBUG_SERIAL.println("ms  (each is a max over the same 5s window)");
        loopCount  = 0;
        loopMaxUs  = 0;
        maxSensorsUs = maxControlUs = maxLogUs = maxTelemUs = 0;

        sensors.gps().printDebug();
    }

    // Enforce 100 Hz minimum: if this loop ran faster than 10ms, yield the remainder.
    // elapsedMillis-based approach: do NOT delay if sensors/comms already took > 10ms.
    if (loopTimer < 10) {
        // busy-wait the remainder so we don't skip a sensor poll cycle
        while (loopTimer < 10);
    }
    loopTimer = 0;
}
