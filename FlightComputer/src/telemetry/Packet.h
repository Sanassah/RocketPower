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

// ===== Rocket → Ground telemetry =====
// Deliberately compact (49 bytes, was 81) -- the LoRa link is stuck at a slow
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

    // High-g accelerometer (ADXL375 is +-200g rated -- centi-g gives +-327g range)
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
