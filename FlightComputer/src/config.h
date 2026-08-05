#pragma once

// =============================================================
// config.h — All pin definitions and flight constants
// Derived from KiCad schematics (main.kicad_sch and sub-sheets)
// Board: Custom MIMXRT1062 (Teensy 4.1 compatible)
// =============================================================

// ===== I2C Bus Assignments =====
// Wire  (LPI2C1, pins 18 SDA / 19 SCL): sensors with no bus-number suffix
// Wire1 (LPI2C3, pins 17 SDA / 16 SCL): sensors labelled _SDA1 / _SCL1
// Wire2 (LPI2C4, pins 25 SDA / 24 SCL): sensors labelled _SDA2 / _SCL2

#define BNO085_I2C_BUS   Wire
#define BNO085_I2C_ADDR  0x4A   // SA0 pulled LOW (BNO085_SCL/SDA, no suffix)

#define BMP390_I2C_BUS   Wire1
#define BMP390_I2C_ADDR  0x77   // SDO pulled HIGH (BMP390_SCL1/SDA1)

#define ZOEM8_I2C_BUS    Wire1
#define ZOEM8_I2C_ADDR   0x42   // fixed u-blox DDC address (ZOEM8_SCL1/SDA1)

// Echo raw NMEA sentences to Serial as they're read (verbose — GPS bring-up only)
#define GPS_DEBUG_RAW_NMEA 0

#define INA260_I2C_BUS   Wire1
#define INA260_I2C_ADDR  0x40   // A0=A1=GND (INA260_SCL1/SDA1)

#define ADXL375_I2C_BUS  Wire2
#define ADXL375_I2C_ADDR 0x1D   // CS tied HIGH, ADDR tied HIGH (ADXL375_SCL2/SDA2)

// ===== UART Assignments =====
// From schematic text annotations: "0 (RX1)", "1 (TX1)", "RX 28", "TX 29"
#define LORA_SERIAL      Serial1   // Lora_TX→pin0 (RX1), Lora_RX→pin1 (TX1)
#define LORA_BAUD        9600

#define CAM_SERIAL       Serial7   // Cam_TX→pin28 (RX7), Cam_RX→pin29 (TX7)
#define CAM_BAUD         115200

// ===== LoRa E22-900T22D Control =====
// M0 and M1 are NOT connected to MCU on this board (4-pin JST-GH connector).
// Module is hardwired in Mode 0 (transparent UART) — M0=GND, M1=GND on PCB.
#define LORA_M0_PIN      -1   // not connected
#define LORA_M1_PIN      -1   // not connected
#define LORA_AUX_PIN     -1   // not connected
#define LORA_CHANNEL     0    // 900.125 MHz + channel * 1 MHz

// ===== SD Card =====
// MicroSD via SDIO (Hirose DM3D-SF, SD_B0 bus)
#define SD_CS_PIN        BUILTIN_SDCARD

// ===== Pyro Channels =====
// From HighCurrentComponents.kicad_sch hierarchical labels
// PyroCHx_N  → low-side MOSFET gate, MCU pin N (active HIGH to fire)
// Pyrox_Test_N → continuity sense (analog input), MCU pin N
#define PYRO_CH1_FIRE_PIN   2     // PyroCH1_2  — ignition
#define PYRO_CH2_FIRE_PIN   3     // PyroCH2_3  — parachute
#define PYRO_CH3_FIRE_PIN   4     // PyroCH3_4  — backup

#define PYRO_CH1_CONT_PIN   40    // Pyro1_Test_40
#define PYRO_CH2_CONT_PIN   41    // Pyro2_Test_41
#define PYRO_CH3_CONT_PIN   39    // Pyro3_Test_39

#define PYRO_NUM_CHANNELS   3
#define PYRO_FIRE_DURATION_MS  500     // pulse width when firing
#define PYRO_CONT_THRESHOLD    512     // ADC counts — above = continuity OK

// ===== Pyro channel aliases =====
#define PYRO_IGNITION   1   // CH1 — motor igniter, fired by ground command
#define PYRO_PARACHUTE  2   // CH2 — recovery parachute, fired at apogee
#define PYRO_BACKUP     3   // CH3 — backup charge, fired manually if CH2 fails

// ===== Camera =====
// JST-GH 4-pin, Serial7 at 115200 baud (RunCam Split 4-25 compatible)

// ===== Fin actuation servos =====
// From HighCurrentComponents.kicad_sch hierarchical labels Servo1_PWM..Servo4_PWM.
// No closed-loop control yet -- pins only, driven manually for bench testing.
#define SERVO1_PIN   36
#define SERVO2_PIN   8
#define SERVO3_PIN   7
#define SERVO4_PIN   9
#define SERVO_NUM_CHANNELS 4

// Blue Bird BMS-101HV (6.0-8.4V digital HV micro servo) datasheet: neutral
// 1520us, rated travel 1100-1900us (90 deg total, ~8.9us/deg). We deliberately
// command a much smaller window than the full rated range for now.
#define SERVO_CENTER_US     1520   // neutral (datasheet value, not 1500)
#define SERVO_MAX_ANGLE_DEG 25     // max deflection each side of center (rated travel is +-45deg)
#define SERVO_MIN_US        (SERVO_CENTER_US - (800L * SERVO_MAX_ANGLE_DEG) / 90)
#define SERVO_MAX_US        (SERVO_CENTER_US + (800L * SERVO_MAX_ANGLE_DEG) / 90)
#define SERVO_TEST_STEP_MS 500    // dwell time at each position during a test sweep

// Per-channel trim (offset from SERVO_CENTER_US) is set by nudging against a
// printed alignment jig and saved to flash -- see FinController.
#define SERVO_TRIM_STEP_US    1      // pulse-width change per nudge -- 1us is the finest step our
                                      // integer-microsecond tracking can express (~0.11 deg on the
                                      // BMS-101HV's 800us/90deg scale)

// Gap between commanding each channel's very first position at boot, so the 4
// servos don't all snap to center in the same instant -- spreads out inrush
// current on the shared, unregulated servo rail instead of stacking it.
// 60ms wasn't enough to actually separate the events (a full board power
// cycle still reproduced the failure); 400ms is closer to the real-world
// separation a manual one-at-a-time reconnect gives each servo, which does
// recover it.
#define SERVO_INIT_STAGGER_MS 400

#define SERVO_CAL_EEPROM_ADDR 0
#define SERVO_CAL_MAGIC       0xC1   // bump if FinCalibration's layout ever changes

// ===== BENCH DIAGNOSTIC =====
// Set to 1-4 to leave that one channel unattached, isolating the other three
// for one-at-a-time testing. 0 = attach all 4 normally.
#define FIN_TEST_SKIP_CHANNEL 0

// ===== Flight Constants =====
#define LIFTOFF_ACCEL_THRESHOLD    2.5f   // g — triggers POWERED_ASCENT
#define LIFTOFF_CONFIRM_MS         100    // must hold for this long
#define BURNOUT_ACCEL_THRESHOLD    1.0f   // g — drop below = COAST
#define APOGEE_DETECTION_WINDOW_MS 200    // vertical-velocity zero-crossing window
#define LANDED_STABLE_MS           10000  // altitude stable for this long = LANDED
#define LANDED_ALT_TOLERANCE_M     2.0f   // ±m to count as "stable"

// Requesting 200ms (5Hz) outran what the LoRa link could actually clear over
// the air, leaving the radio almost continuously transmitting -- observed
// real throughput was 0.8-2.3Hz, and since the link is half-duplex, a nearly
// always-busy radio also had little time free to receive inbound commands.
// 400ms still wasn't enough idle/listening time for reliable command
// reception. Confirmed empirically with the isolated TelemetryTest rig
// (dual-comms build): 1000ms (1Hz) gave a clean, reliable command hit rate
// alongside continuous telemetry. Slower downlink, but a link that actually
// works both ways beats a faster one that doesn't.
#define TELEMETRY_INTERVAL_MS      1000   // 1 Hz downlink (was 400ms/2.5Hz)
#define LOG_INTERVAL_MS            10     // 100 Hz SD logging

// Complementary filter weight for fused vertical velocity (SensorManager).
// 0.98 = IMU integration dominates above ~0.5 Hz, baro corrects drift below that.
#define VERT_VEL_ALPHA  0.98f

// ===== Sea-level pressure for altitude reference =====
#define SEA_LEVEL_HPA 1013.25f
