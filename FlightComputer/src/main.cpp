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

// ===== Global objects =====
SensorManager   sensors;
PyroController  pyro;
StateMachine    fsm(pyro);
LoRaRadio       radio;
TelemetryManager telem(radio, fsm, pyro);
DataLogger      logger;
CameraController camera;

// ===== Timing =====
elapsedMillis loopTimer;     // tracks time since last loop start
uint32_t      loopCount    = 0;
uint32_t      loopMaxUs    = 0;
FlightState   prevState    = FlightState::IDLE;

// ===== Helpers =====
static void printSensorReport() {
    const FlightData& d = sensors.data();
    Serial.println("--- Sensor init report ---");
    Serial.print("  IMU   (BNO085 Wire  0x4A): "); Serial.println(d.imu_ok   ? "OK" : "FAIL");
    Serial.print("  Baro  (BMP390 Wire1 0x77): "); Serial.println(d.baro_ok  ? "OK" : "FAIL");
    Serial.print("  Accel (ADXL375 Wire2 0x1D): "); Serial.println(d.accel_ok ? "OK" : "FAIL");
    Serial.print("  GPS   (ZOEM8  Wire1 0x42): "); Serial.println(d.gps_ok   ? "OK" : "FAIL");
    Serial.print("  Power (INA260 Wire1 0x40): "); Serial.println(d.power_ok  ? "OK" : "FAIL");
    Serial.print("  SD    (BUILTIN_SDCARD):    "); Serial.println(SD.begin(SD_CS_PIN) ? "OK" : "FAIL");
    Serial.println("--------------------------");
}

// =================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);

    Serial.println("\n=================================");
    Serial.println(" RocketFlightComputer v0.1");
    Serial.println(" Custom MIMXRT1062 / Teensy 4.1");
    Serial.println("=================================\n");

    // ---- Sensors ----
    Serial.println("[INIT] Sensors...");
    bool sensorsOk = sensors.begin();
    printSensorReport();
    if (!sensorsOk) {
        Serial.println("[INIT] CRITICAL: IMU or barometer failed. Halting.");
        while (true) { delay(1000); }
    }

    // ---- Calibrate barometer at launch site ----
    Serial.println("[INIT] Calibrating barometer (2s)...");
    sensors.calibrateBaro();
    Serial.println("[INIT] Baro calibrated.");

    // ---- Pyro ----
    Serial.println("[INIT] Pyro channels...");
    pyro.begin();
    for (int ch = 1; ch <= PYRO_NUM_CHANNELS; ch++) {
        Serial.print("  CH"); Serial.print(ch);
        Serial.print(" continuity: ");
        Serial.println(pyro.continuityOk(ch) ? "OK" : "OPEN (no ematch?)");
    }

    // ---- SD card ----
    Serial.println("[INIT] SD card...");
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[INIT] WARNING: SD card failed. Logging disabled.");
    } else {
        logger.open();
    }

    // ---- LoRa / Telemetry ----
    Serial.println("[INIT] LoRa radio...");
    if (!telem.begin()) {
        Serial.println("[INIT] WARNING: LoRa init failed.");
    }

    // ---- Camera ----
    camera.begin();

    Serial.println("\n[INIT] Boot complete. State: IDLE\n");
    loopTimer = 0;
}

// =================================================================
void loop() {
    uint32_t loopStart = micros();

    // ---- 1. Read all sensors ----
    sensors.update();
    const FlightData& d = sensors.data();

    // ---- 2. Inject current state into data (read-modify-write via mutable ref) ----
    // StateMachine owns the state; we inject it into the FlightData snapshot
    // for logging and telemetry by const_casting the immutable ref.
    // (In a refactor, SensorManager would hold a state pointer — fine for now.)
    const_cast<FlightData&>(d).state = fsm.state();

    // ---- 3. Update state machine ----
    fsm.update(d);
    FlightState newState = fsm.state();

    // ---- 4. Handle state transitions ----
    if (newState != prevState) {
        camera.onStateChange(prevState, newState);
        logger.flush();   // force flush on state change to minimise data loss

        // Arm pyro on ARM, disarm on IDLE or LANDED
        if (newState == FlightState::ARMED) {
            pyro.arm();
            // Re-calibrate baro at the actual launch site, not wherever the FCC booted.
            // The rocket should be stationary on the pad when ARM is sent.
            Serial.println("[CALIB] Re-calibrating baro at launch site...");
            sensors.calibrateBaro();
            Serial.println("[CALIB] Done.");
        }
        if (newState == FlightState::IDLE || newState == FlightState::LANDED) pyro.disarm();

        // Close log after landing
        if (newState == FlightState::LANDED) logger.close();

        prevState = newState;
    }

    // ---- 5. Log data ----
    logger.update(d);

    // ---- 6. Telemetry (send + receive commands) ----
    telem.update(d);

    // ---- 6b. Execute calibration if requested by ground station ----
    if (telem.calibrateRequested()) {
        Serial.println("[CALIB] Calibrating barometer...");
        sensors.calibrateBaro();
        Serial.println("[CALIB] Done.");
    }

    // ---- 7. Loop timing diagnostics ----
    uint32_t elapsed = micros() - loopStart;
    if (elapsed > loopMaxUs) loopMaxUs = elapsed;
    loopCount++;

    // Print loop stats every 5 seconds
    static uint32_t lastStatMs = 0;
    if (millis() - lastStatMs >= 5000) {
        lastStatMs = millis();
        float hz = loopCount / 5.0f;
        Serial.print("[LOOP] "); Serial.print(hz, 1); Serial.print(" Hz | ");
        Serial.print("max="); Serial.print(loopMaxUs); Serial.print("us | ");
        Serial.print("state="); Serial.println(flightStateName(fsm.state()));
        loopCount  = 0;
        loopMaxUs  = 0;
    }

    // Enforce 100 Hz minimum: if this loop ran faster than 10ms, yield the remainder.
    // elapsedMillis-based approach: do NOT delay if sensors/comms already took > 10ms.
    if (loopTimer < 10) {
        // busy-wait the remainder so we don't skip a sensor poll cycle
        while (loopTimer < 10);
    }
    loopTimer = 0;
}
