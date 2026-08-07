#include "PowerMonitor.h"
#include "../config.h"

bool PowerMonitor::begin() {
    if (!_ina.begin(INA260_I2C_ADDR, &INA260_I2C_BUS)) return false;
    _data.valid = true;
    return true;
}

bool PowerMonitor::update() {
    // The Adafruit INA260 driver's read calls don't report I2C failures
    // themselves (they just return whatever the register read produced), so
    // probe the address directly to know whether the chip is still there.
    INA260_I2C_BUS.beginTransmission(INA260_I2C_ADDR);
    if (INA260_I2C_BUS.endTransmission() != 0) return false;

    _data.voltage_v  = _ina.readBusVoltage() / 1000.0f;   // mV → V
    _data.current_ma = _ina.readCurrent();                 // already mA
    _data.power_mw   = _ina.readPower();                   // already mW
    return true;
}
