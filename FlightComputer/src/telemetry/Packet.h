#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../states/FlightStates.h"

// Magic bytes identify valid packets and their direction
#define TELEM_MAGIC_0  0xAA
#define TELEM_MAGIC_1  0x55
#define CMD_MAGIC_0    0xBB
#define CMD_MAGIC_1    0x44
#define ACK_MAGIC_0    0xAC
#define ACK_MAGIC_1    0x4B

// System status bitfield (TelemetryPacket.system_status). The SENSOR_HEALTH_*
// bits are set when that sensor produced a successful reading within
// SENSOR_HEALTH_TIMEOUT_MS of this packet being sent -- i.e. "alive right
// now", not "passed its boot check" (see SensorManager). The SD_* bits
// reflect DataLogger's last explicit presence check (boot, a ground
// SD_START_RECORDING command, or write success/failure while recording) --
// see DataLogger::cardPresent() for why this isn't continuously re-polled.
#define SENSOR_HEALTH_IMU_OK    (1 << 0)
#define SENSOR_HEALTH_BARO_OK   (1 << 1)
#define SENSOR_HEALTH_ACCEL_OK  (1 << 2)
#define SENSOR_HEALTH_GPS_OK    (1 << 3)
#define SENSOR_HEALTH_POWER_OK  (1 << 4)
#define SD_STATUS_PRESENT       (1 << 5)   // a card responded recently
#define SD_STATUS_RECORDING     (1 << 6)   // a log file is currently open

// Attitude control status bitfield (TelemetryPacket.attitude_status) --
// separate from system_status since this is a commanded MODE, not sensor/
// hardware health. Set/cleared by the ATTITUDE_CONTROL_ENABLE/DISABLE and
// ATTITUDE_DEMO_ENABLE/DISABLE commands below; see AttitudeController.
#define ATTITUDE_STATUS_CONTROL_ON  (1 << 0)
#define ATTITUDE_STATUS_DEMO_ON     (1 << 1)

// ===== Rocket → Ground telemetry =====
// Deliberately compact (52 bytes) -- the LoRa link is stuck at a slow
// factory-default air data rate (2.4kbps) and reconfiguring the radios is off
// the table for now, so packet size is the only remaining lever to reduce
// airtime per packet. Most fields are scaled fixed-point instead of float:
// GPS accuracy is only ~2-5m, so lat/lon as float (not double) already loses
// no real accuracy; the rest use a resolution well beyond what's physically
// meaningful for this vehicle (see comments per field).
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  magic[2];       // TELEM_MAGIC_0, TELEM_MAGIC_1

    uint16_t seq;            // rolling sequence number
    uint32_t timestamp_ms;   // millis() since boot
    uint8_t  state;          // FlightState

    // GPS
    float    lat;
    float    lon;
    int16_t  gps_alt_dm;     // decimeters (0.1m resolution), +-3276m range
    uint8_t  gps_sats;
    uint8_t  gps_fix;

    // Barometer
    int16_t  baro_alt_dm;    // decimeters (0.1m resolution), relative to launch site
    int16_t  vert_vel_cms;   // cm/s (0.01 m/s resolution), positive = up

    // TEMPORARY: sourced from the BNO085's gravity-removed linear
    // acceleration (m/s², converted to g), NOT the ADXL375 -- see
    // IMU_LIFTOFF_ACCEL_THRESHOLD_MS2 in config.h for why. Reads ~0g at
    // rest on all 3 axes now (gravity subtracted), unlike the ADXL375's
    // ~1g-on-one-axis-at-rest. Field name/scale (centi-g, int16) kept as-is
    // for GroundStation compatibility -- REVERT to highg_x/y/z_g once the
    // ADXL375 hardware fault is resolved.
    int16_t  accel_x_cg;
    int16_t  accel_y_cg;
    int16_t  accel_z_cg;

    // IMU quaternion -- components are always in [-1,1], scaled to int16
    int16_t  quat_w_i16;
    int16_t  quat_x_i16;
    int16_t  quat_y_i16;
    int16_t  quat_z_i16;

    // Power
    int16_t  voltage_cv;     // centi-volts (0.01V resolution)
    int16_t  current_ma;     // milliamps, integer precision is plenty

    // Link quality
    int8_t   rssi;

    uint8_t  pyro_cont[3];   // continuity per channel: 1=OK 0=open (index 0=CH1, 1=CH2, 2=CH3)

    // Camera recording state as last commanded by CameraController. The RunCam
    // UART link has no ack, so this is what the FC believes it told the camera,
    // not a hardware-confirmed state -- still enough to catch a missed command
    // (e.g. sent while out of range) from the ground without touching a cable.
    uint8_t  cam_recording;  // 1=recording, 0=stopped

    // Live sensor + SD card status, re-derived every loop -- see the bit
    // #defines above. Distinct from a one-time boot check: this reflects
    // "still true right now," so a mid-flight dropout (or a card pulled/
    // inserted on the bench) shows up here.
    uint8_t  system_status;

    // Current attitude-control mode, as last set by a ground command -- see
    // the ATTITUDE_STATUS_* bits above. Not sensor health; this is "what did
    // the ground tell it to do," so the operator can confirm a command
    // actually landed instead of guessing from silence.
    uint8_t  attitude_status;

    uint16_t checksum;       // simple sum of all preceding bytes
};
#pragma pack(pop)

// ===== Ground → Rocket commands =====
enum class CommandType : uint8_t {
    ARM            = 0x01,
    DISARM         = 0x02,
    FIRE_PYRO      = 0x03,   // param = channel (1-3)
    PING           = 0x04,
    CALIBRATE_BARO = 0x05,   // re-zero barometer altitude at current ground level
    SERVO_TEST     = 0x06,   // param = fin channel (1-4); sweeps center->min->max->center
    CAM_TOGGLE     = 0x07,   // manually toggle camera recording (the camera only supports a toggle)
    SERVO_NUDGE_POS = 0x09,  // param = fin channel (1-4); +SERVO_TRIM_STEP_US, not persisted
    SERVO_NUDGE_NEG = 0x0A,  // param = fin channel (1-4); -SERVO_TRIM_STEP_US, not persisted
    SERVO_SAVE_CAL  = 0x0B,  // param unused; persists all 4 channels' live position as new trim
    SERVO_CENTER_ALL = 0x0C, // param unused; drives all 4 to raw center, ignoring trim, not persisted
    SERVO_PREFLIGHT  = 0x0D, // param unused; blocking ~6s all-4 choreography, see FinController
    SD_START_RECORDING = 0x0E, // param unused; opens a new log file (no-op if already recording)
    SD_STOP_RECORDING  = 0x0F, // param unused; flushes and closes the current log file

    // Runtime attitude-control mode toggles -- param unused. See
    // AttitudeController and config.h for what each mode actually does and
    // the checklist to clear before ever sending ATTITUDE_CONTROL_ENABLE for
    // a real flight. Both default off at boot and reset off on DISARM.
    ATTITUDE_CONTROL_ENABLE  = 0x10,
    ATTITUDE_CONTROL_DISABLE = 0x11,
    ATTITUDE_DEMO_ENABLE     = 0x12,
    ATTITUDE_DEMO_DISABLE    = 0x13,

    // Soft reset -- param unused. Puts the flight computer back to a fresh-
    // boot-equivalent IDLE without an actual power cycle: disarms, clears
    // StateMachine's and BackupDeploy's detection state (including
    // BackupDeploy's own "fires at most once" latch -- see its reset()),
    // resets AttitudeController, and closes/reopens the SD log so the next
    // run gets its own file. Exists for repeated bench/HITL test flights --
    // see main.cpp's handling for exactly what it touches. Deliberately
    // does NOT reset PyroController's continuity reads (those are live,
    // not latched) or anything HITL-specific (that's TeensyBridge/
    // SensorManager's own concern, this only touches real flight-logic
    // state that persists in RAM across a would-be flight).
    RESET = 0x14,
};

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t     magic[2];    // CMD_MAGIC_0, CMD_MAGIC_1
    CommandType type;
    uint8_t     param;       // FIRE_PYRO: channel number; others: 0
    uint8_t     seq;         // ground-assigned, echoed back in AckPacket so
                              // the ground knows exactly which send this was
                              // -- also used on this end to deduplicate a
                              // resent command (lost ack, not lost command)
                              // from a genuinely new one.
    uint16_t    checksum;    // simple sum of preceding bytes
};
#pragma pack(pop)

// ===== Rocket -> Ground: acknowledges a received command =====
// Sent immediately on receipt/validation, before the command is actually
// executed -- so even a slow, blocking command (e.g. SERVO_PREFLIGHT) still
// gets acked promptly and the ground stops retrying right away instead of
// retrying into an already-executing sequence.
#pragma pack(push, 1)
struct AckPacket {
    uint8_t     magic[2];    // ACK_MAGIC_0, ACK_MAGIC_1
    uint8_t     cmdSeq;      // echoes CommandPacket.seq being acknowledged
    CommandType cmdType;     // echoes CommandPacket.type, handy for logging
    uint16_t    checksum;    // simple sum of preceding bytes
};
#pragma pack(pop)

// Checksum helpers
inline uint16_t packetChecksum(const uint8_t* data, size_t len) {
    uint16_t s = 0;
    for (size_t i = 0; i < len; ++i) s += data[i];
    return s;
}

// ===== HITL sensor injection (simulation -> rocket) =====
// HITL_MODE builds only (see platformio.ini env:rocketpower_hitl). Carried on
// SerialUSB1 -- a SECOND Teensy USB virtual serial port, separate from the
// normal Serial -- so this protocol never contends with the existing command
// RX / telemetry mirror on Serial, which stays fully functional (a real
// GroundStation can watch a HITL run live over Serial at the same time the
// simulation drives it over SerialUSB1).
//
// SensorManager::update() (HITL_MODE branch) blocks reading exactly one of
// these per loop iteration -- that block IS this build's loop pacing, in
// place of the real build's I2C reads. See SensorManager.cpp and
// Simulation/hitl/TeensyBridge.m for the field-by-field derivation (notably:
// lin_accel is gravity-COMPENSATED like the real BNO085 "linear acceleration"
// report, highg is NOT -- raw specific force, reads ~1g at rest, like the
// real ADXL375).
#define HITL_SENSOR_MAGIC_0 0xC5
#define HITL_SENSOR_MAGIC_1 0x17
#define HITL_RESP_MAGIC_0   0xC6
#define HITL_RESP_MAGIC_1   0x18

#pragma pack(push, 1)
struct SensorInjectPacket {
    uint8_t  magic[2];   // HITL_SENSOR_MAGIC_0, HITL_SENSOR_MAGIC_1
    uint32_t seq;         // bridge-side bookkeeping only -- not echoed back, this
                           // link is strict single-outstanding-request lockstep

    // IMU (BNO085-equivalent)
    float quat_w, quat_x, quat_y, quat_z;
    float gyro_x, gyro_y, gyro_z;                 // rad/s, body frame
    float lin_accel_x, lin_accel_y, lin_accel_z;  // m/s^2, body frame, gravity-compensated

    // High-g accelerometer (ADXL375-equivalent) -- RAW specific force,
    // gravity included (reads ~1g magnitude at rest, same as the real part).
    float highg_x, highg_y, highg_z;   // g, body frame

    // Barometer-equivalent (BMP390)
    float baro_alt_m;    // m, already pad-relative (bridge zeroes this, not the firmware)
    float vert_vel_ms;   // m/s, positive = up, world frame

    uint16_t checksum;   // simple sum of all preceding bytes
};
#pragma pack(pop)

// ===== HITL response (rocket -> simulation) =====
// Sent unconditionally once per loop by TelemetryManager::update()'s
// HITL_MODE block, immediately after the real AttitudeController/
// FinController have processed this loop's injected sample -- reports
// exactly what the real flight code decided, so the simulation can apply the
// real commanded fin deflections to its own actuator/aero model and close
// the loop for real.
#pragma pack(push, 1)
struct HITLResponsePacket {
    uint8_t  magic[2];   // HITL_RESP_MAGIC_0, HITL_RESP_MAGIC_1
    uint32_t timestamp_ms;
    uint8_t  state;       // FlightState

    // Real, post-allocation, post-clamp commanded fin deflection (deg) per
    // channel -- CH1=S, CH2=E, CH3=N, CH4=W (see AttitudeController.h). This
    // is what actually got written to the servo this loop, clamp included.
    float fin_deg[4];

    uint8_t  attitude_status;   // see ATTITUDE_STATUS_* above

    uint16_t checksum;
};
#pragma pack(pop)
