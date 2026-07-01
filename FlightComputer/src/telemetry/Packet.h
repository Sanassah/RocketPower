#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../states/FlightStates.h"

// Magic bytes identify valid packets and their direction
#define TELEM_MAGIC_0  0xAA
#define TELEM_MAGIC_1  0x55
#define CMD_MAGIC_0    0xBB
#define CMD_MAGIC_1    0x44

// ===== Rocket → Ground telemetry =====
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  magic[2];       // TELEM_MAGIC_0, TELEM_MAGIC_1

    uint16_t seq;            // rolling sequence number
    uint32_t timestamp_ms;   // millis() since boot
    uint8_t  state;          // FlightState

    // GPS
    double   lat;
    double   lon;
    float    gps_alt_m;
    uint8_t  gps_sats;
    uint8_t  gps_fix;

    // Barometer
    float    baro_alt_m;     // relative to launch site
    float    vert_vel_ms;    // m/s, positive = up

    // High-g accelerometer
    float    accel_x_g;
    float    accel_y_g;
    float    accel_z_g;

    // IMU quaternion
    float    quat_w;
    float    quat_x;
    float    quat_y;
    float    quat_z;

    // Power
    float    voltage_v;
    float    current_ma;

    // Link quality
    int8_t   rssi;

    uint8_t  pyro_cont[3];   // continuity per channel: 1=OK 0=open (index 0=CH1, 1=CH2, 2=CH3)

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
};

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t     magic[2];    // CMD_MAGIC_0, CMD_MAGIC_1
    CommandType type;
    uint8_t     param;       // FIRE_PYRO: channel number; others: 0
    uint16_t    checksum;    // simple sum of preceding bytes
};
#pragma pack(pop)

// Checksum helpers
inline uint16_t packetChecksum(const uint8_t* data, size_t len) {
    uint16_t s = 0;
    for (size_t i = 0; i < len; ++i) s += data[i];
    return s;
}
