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
#define PYRO_CH1_FIRE_PIN   2     // PyroCH1_2  — drogue
#define PYRO_CH2_FIRE_PIN   3     // PyroCH2_3  — main
#define PYRO_CH3_FIRE_PIN   4     // PyroCH3_4  — aux / backup

#define PYRO_CH1_CONT_PIN   40    // Pyro1_Test_40
#define PYRO_CH2_CONT_PIN   41    // Pyro2_Test_41
#define PYRO_CH3_CONT_PIN   39    // Pyro3_Test_39

#define PYRO_NUM_CHANNELS   3
#define PYRO_FIRE_DURATION_MS  500     // pulse width when firing
#define PYRO_CONT_THRESHOLD    512     // ADC counts — above = continuity OK

// ===== Pyro channel aliases =====
#define PYRO_DROGUE  1
#define PYRO_MAIN    2
#define PYRO_AUX     3

// ===== Camera =====
// JST-GH 4-pin, Serial7 at 115200 baud (RunCam Split 4-25 compatible)

// ===== Flight Constants =====
#define LIFTOFF_ACCEL_THRESHOLD    2.5f   // g — triggers POWERED_ASCENT
#define LIFTOFF_CONFIRM_MS         100    // must hold for this long
#define BURNOUT_ACCEL_THRESHOLD    1.0f   // g — drop below = COAST
#define APOGEE_DETECTION_WINDOW_MS 200    // vertical-velocity zero-crossing window
#define LANDED_STABLE_MS           10000  // altitude stable for this long = LANDED
#define LANDED_ALT_TOLERANCE_M     2.0f   // ±m to count as "stable"

#define TELEMETRY_INTERVAL_MS      200    // 5 Hz downlink
#define LOG_INTERVAL_MS            10     // 100 Hz SD logging

// ===== Sea-level pressure for altitude reference =====
#define SEA_LEVEL_HPA 1013.25f
