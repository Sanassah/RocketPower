#pragma once

// =============================================================
// All pin definitions and flight constants derived from schematics
// ===== I2C Bus Assignments =====
// Wire  (LPI2C1, pins 18 SDA / 19 SCL)
// Wire1 (LPI2C3, pins 17 SDA / 16 SCL)
// Wire2 (LPI2C4, pins 25 SDA / 24 SCL)
// =============================================================

#define BNO085_I2C_BUS   Wire
#define BNO085_I2C_ADDR  0x4A   // SA0 pulled LOW

#define INA260_I2C_BUS   Wire1
#define INA260_I2C_ADDR  0x40   // A0=A1=GND

#define ZOEM8_I2C_BUS    Wire1
#define ZOEM8_I2C_ADDR   0x42   // fixed u-blox DDC address

#define BMP390_I2C_BUS   Wire1
#define BMP390_I2C_ADDR  0x77   // SDO pulled HIGH

#define ADXL375_I2C_BUS  Wire2
#define ADXL375_I2C_ADDR 0x1D   // CS tied HIGH, ADDR tied HIGH


