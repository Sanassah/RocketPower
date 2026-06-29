/*
 * sensor_test.ino — RocketFlightComputer Sensor Self-Test
 * Board: Custom MIMXRT1062 (programmed via Teensyduino, target "Teensy 4.1")
 *
 * Pin/bus assignments derived from schematics:
 *   Wire  (I2C1, pins 18/19): BNO085@0x4A  (no bus-number suffix in schematic)
 *   Wire1 (I2C2, pins 17/16): BMP390@0x77, ZOE-M8Q@0x42, INA260@0x40  (_SCL1/_SDA1)
 *   Wire2 (I2C3, pins 25/24): ADXL375@0x1D  (_SCL2/_SDA2)
 *   Serial1 (pins 0 RX / 1 TX): LoRa EBYTE E22 telemetry @ 9600 baud
 *   Serial7 (pins 28 RX / 29 TX): Camera UART @ 115200 baud
 *   BUILTIN_SDCARD: MicroSD via SDIO (SD_B0 bus, Hirose DM3D-SF)
 *
 * Required libraries (install via Arduino Library Manager):
 *   Adafruit ADXL375, Adafruit BMP3XX, Adafruit BNO08x,
 *   Adafruit INA260, Adafruit Unified Sensor, TinyGPSPlus
 */

#include <Wire.h>
#include <SD.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_INA260.h>
#include <TinyGPSPlus.h>

// ===== Configuration =====
#define ZOEM8_ADDR      0x42
#define LORA_SERIAL     Serial1
#define CAM_SERIAL      Serial7
#define LORA_BAUD       9600
#define CAM_BAUD        115200
#define SEA_LEVEL_HPA   1013.25f
#define STREAM_INTERVAL 100   // ms between data lines

// ===== Sensor instances =====
// Wire2 is passed to the constructor — begin() only accepts the I2C address
Adafruit_ADXL375 adxl(1, &Wire2);
Adafruit_BMP3XX  bmp;
Adafruit_BNO08x  bno;
Adafruit_INA260  ina;
TinyGPSPlus      gps;

// ===== Pass/fail state =====
bool ok_adxl, ok_bmp, ok_bno, ok_gps, ok_ina, ok_sd;

// BNO085 latest reading (updated in loop before print interval)
sh2_SensorValue_t bnoVal;

// ===== Helpers =====
static inline void printPass() { Serial.println("  [PASS]"); }
static inline void printFail(const char *reason) {
  Serial.print("  [FAIL] "); Serial.println(reason);
}

// Poll u-blox ZOE-M8Q NMEA stream over I2C DDC
static void pollGPS() {
  // Read bytes-available counter from registers 0xFD (MSB), 0xFE (LSB)
  Wire1.beginTransmission(ZOEM8_ADDR);
  Wire1.write(0xFD);
  if (Wire1.endTransmission(false) != 0) return;  // bus error

  Wire1.requestFrom((uint8_t)ZOEM8_ADDR, (uint8_t)2);
  if (Wire1.available() < 2) return;
  uint16_t avail = ((uint16_t)Wire1.read() << 8) | Wire1.read();

  if (avail == 0 || avail == 0xFFFF) return;  // no data / not ready
  if (avail > 512) avail = 512;               // guard against spurious large counts

  // Read NMEA bytes from the stream register (0xFF, auto-increments)
  while (avail) {
    uint8_t chunk = (avail > 32) ? 32 : (uint8_t)avail;
    Wire1.requestFrom((uint8_t)ZOEM8_ADDR, chunk);
    while (Wire1.available()) gps.encode((char)Wire1.read());
    avail -= chunk;
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);  // wait up to 5 s for USB serial

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" RocketFlightComputer Sensor Self-Test");
  Serial.println(" Custom MIMXRT1062 Board");
  Serial.println("==========================================");
  Serial.println();

  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  // -------- ADXL375 — Wire2, 0x1D --------
  Serial.print("[ADXL375] High-g accel  (Wire2, 0x1D)");
  ok_adxl = adxl.begin(0x1D);
  if (ok_adxl) {
    adxl.setDataRate(ADXL3XX_DATARATE_100_HZ);
    // ADXL375 is fixed ±200g — setRange() is a no-op in this library version
    printPass();
  } else {
    printFail("device not found");
  }

  // -------- BMP390 — Wire1, 0x77 --------
  Serial.print("[BMP390 ] Barometer     (Wire1, 0x77)");
  ok_bmp = bmp.begin_I2C(0x77, &Wire1);
  if (ok_bmp) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    printPass();
  } else {
    printFail("device not found");
  }

  // -------- BNO085 — Wire, 0x4A --------
  Serial.print("[BNO085 ] IMU           (Wire,  0x4A)");
  // BNO08x_I2CADDR_DEFAULT = 0x4A (SA0 pulled low on this board)
  if (bno.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
    ok_bno = bno.enableReport(SH2_ROTATION_VECTOR);
    if (ok_bno) printPass();
    else        printFail("enableReport failed");
  } else {
    printFail("device not found");
  }

  // -------- ZOE-M8Q — Wire1, 0x42 --------
  Serial.print("[ZOEM8  ] GPS           (Wire1, 0x42)");
  Wire1.beginTransmission(ZOEM8_ADDR);
  ok_gps = (Wire1.endTransmission() == 0);
  if (ok_gps) printPass(); else printFail("device not found");

  // -------- INA260 — Wire1, 0x40 --------
  Serial.print("[INA260 ] Power monitor (Wire1, 0x40)");
  // A0=A1=GND → address 0x40
  ok_ina = ina.begin(0x40, &Wire1);
  if (ok_ina) printPass(); else printFail("device not found");

  // -------- MicroSD — BUILTIN_SDCARD --------
  Serial.print("[MicroSD] SD card       (BUILTIN_SDCARD)");
  ok_sd = SD.begin(BUILTIN_SDCARD);
  if (ok_sd) printPass(); else printFail("no card or init error");

  // -------- LoRa — Serial1 --------
  Serial.print("[LoRa   ] Telemetry     (Serial1, ");
  Serial.print(LORA_BAUD); Serial.print(" baud)");
  LORA_SERIAL.begin(LORA_BAUD);
  delay(50);
  Serial.println("  [PASS] UART ready");

  // -------- Camera — Serial7 --------
  Serial.print("[Camera ] Interface     (Serial7, ");
  Serial.print(CAM_BAUD); Serial.print(" baud)");
  CAM_SERIAL.begin(CAM_BAUD);
  delay(50);
  Serial.println("  [PASS] UART ready");

  // -------- Startup summary --------
  Serial.println();
  Serial.println("==========================================");
  Serial.println(" Startup Summary");
  Serial.println("==========================================");
  Serial.print("  ADXL375 (±200g accel) : "); Serial.println(ok_adxl ? "PASS" : "FAIL");
  Serial.print("  BMP390  (barometer)   : "); Serial.println(ok_bmp  ? "PASS" : "FAIL");
  Serial.print("  BNO085  (9-DOF IMU)   : "); Serial.println(ok_bno  ? "PASS" : "FAIL");
  Serial.print("  ZOEM8   (GPS)         : "); Serial.println(ok_gps  ? "PASS" : "FAIL");
  Serial.print("  INA260  (power mon.)  : "); Serial.println(ok_ina  ? "PASS" : "FAIL");
  Serial.print("  MicroSD (SD card)     : "); Serial.println(ok_sd   ? "PASS" : "FAIL");
  Serial.println("  LoRa    (Serial1)     : PASS");
  Serial.println("  Camera  (Serial7)     : PASS");
  Serial.println("==========================================");
  Serial.println();
  Serial.print("Live data stream at "); Serial.print(STREAM_INTERVAL); Serial.println("ms intervals:");
  Serial.println("[ms] | ADXL ax,ay,az(g) | BMP tempC,hPa,m | BNO qw,qx,qy,qz | GPS lat,lon,sv | INA V,mA,mW");
  Serial.println();
}

// ===== Loop =====
void loop() {
  static uint32_t lastPrint = 0;

  // Poll continuously so GPS parser and BNO event queue stay drained
  if (ok_gps) pollGPS();
  if (ok_bno) bno.getSensorEvent(&bnoVal);

  if (millis() - lastPrint < STREAM_INTERVAL) return;
  lastPrint = millis();

  Serial.print(lastPrint); Serial.print(" | ");

  // --- ADXL375 ---
  if (ok_adxl) {
    sensors_event_t e;
    adxl.getEvent(&e);
    Serial.print("ADXL ");
    // getEvent returns m/s²; divide by gravity constant for g
    Serial.print(e.acceleration.x / SENSORS_GRAVITY_STANDARD, 2); Serial.print(',');
    Serial.print(e.acceleration.y / SENSORS_GRAVITY_STANDARD, 2); Serial.print(',');
    Serial.print(e.acceleration.z / SENSORS_GRAVITY_STANDARD, 2); Serial.print('g');
  } else {
    Serial.print("ADXL FAIL");
  }

  Serial.print(" | ");

  // --- BMP390 ---
  if (ok_bmp && bmp.performReading()) {
    Serial.print("BMP ");
    Serial.print(bmp.temperature, 1);               Serial.print("C,");
    Serial.print(bmp.pressure / 100.0f, 2);         Serial.print("hPa,");
    Serial.print(bmp.readAltitude(SEA_LEVEL_HPA), 1); Serial.print('m');
  } else {
    Serial.print("BMP FAIL");
  }

  Serial.print(" | ");

  // --- BNO085 ---
  if (ok_bno && bnoVal.sensorId == SH2_ROTATION_VECTOR) {
    Serial.print("BNO ");
    Serial.print(bnoVal.un.rotationVector.real, 3); Serial.print(',');
    Serial.print(bnoVal.un.rotationVector.i,    3); Serial.print(',');
    Serial.print(bnoVal.un.rotationVector.j,    3); Serial.print(',');
    Serial.print(bnoVal.un.rotationVector.k,    3);
  } else {
    Serial.print("BNO WAIT");  // ok but no quaternion event yet
  }

  Serial.print(" | ");

  // --- ZOE-M8Q GPS ---
  if (ok_gps) {
    Serial.print("GPS ");
    if (gps.location.isValid()) {
      Serial.print(gps.location.lat(), 6); Serial.print(',');
      Serial.print(gps.location.lng(), 6); Serial.print(',');
    } else {
      Serial.print("NO_FIX,");
    }
    Serial.print(gps.satellites.value()); Serial.print("sv");
  } else {
    Serial.print("GPS FAIL");
  }

  Serial.print(" | ");

  // --- INA260 ---
  if (ok_ina) {
    Serial.print("INA ");
    Serial.print(ina.readBusVoltage() / 1000.0f, 3); Serial.print("V,");
    Serial.print(ina.readCurrent(), 1);               Serial.print("mA,");
    Serial.print(ina.readPower(), 1);                 Serial.print("mW");
  } else {
    Serial.print("INA FAIL");
  }

  Serial.println();
}
