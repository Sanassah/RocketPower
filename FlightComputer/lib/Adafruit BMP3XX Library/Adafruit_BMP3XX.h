/*!
 * @file Adafruit_BMP3XX.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 * ---------------------------------------------------------------------
 * RocketPower fork (vendored into this project's lib/ so PlatformIO's
 * local-library priority shadows the registry copy in lib_deps --
 * .pio/libdeps gets wiped on a clean rebuild, this won't be). Only change
 * from upstream 2.1.6: added enableNormalMode()/_normalModeActive/
 * _normalSensorComp, and a data-ready (DRDY) check in performReading()'s
 * normal-mode path -- see Adafruit_BMP3XX.cpp for why. Diff against
 * upstream before bumping this fork if Adafruit's version ever needs
 * pulling in again.
 * ---------------------------------------------------------------------
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"

#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
/*=========================================================================*/
#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

/** Adafruit_BMP3XX Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

class Adafruit_BMP3XX {
public:
  Adafruit_BMP3XX();

  bool begin_I2C(uint8_t addr = BMP3XX_DEFAULT_ADDRESS,
                 TwoWire *theWire = &Wire);
  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 uint32_t frequency = BMP3XX_DEFAULT_SPIFREQ);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, uint32_t frequency = BMP3XX_DEFAULT_SPIFREQ);
  uint8_t chipID(void);
  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);

  /// Perform a reading in blocking mode
  bool performReading(void);

  /// RocketPower fork: one-time switch from the default forced (one-shot,
  /// trigger-and-wait-for-a-fresh-conversion-every-call) mode to normal
  /// (continuous background sampling at the configured ODR) mode. Call once
  /// after the oversampling/filter/ODR setters above; every
  /// performReading() after this just fetches the latest already-ready
  /// sample instead of re-triggering -- see the .cpp for the bench numbers
  /// that motivated this.
  bool enableNormalMode(void);

  /// Temperature (Celsius) assigned after calling performReading()
  double temperature;
  /// Pressure (Pascals) assigned after calling performReading()
  double pressure;

private:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  bool _init(void);

  bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;
  uint8_t _i2caddr;
  int32_t _sensorID;
  int8_t _cs;
  unsigned long _meas_end;

  /// RocketPower fork: set by enableNormalMode(), read by performReading()
  /// -- see enableNormalMode() above.
  bool _normalModeActive = false;
  uint8_t _normalSensorComp = 0;

  uint8_t spixfer(uint8_t x);

  struct bmp3_dev the_sensor;
};

#endif
