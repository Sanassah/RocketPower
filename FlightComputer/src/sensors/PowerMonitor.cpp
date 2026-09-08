#include "PowerMonitor.h"
#include "../config.h"
#include "../DebugPrint.h"

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

    // Bench diagnostic (DEBUG_SERIAL only -- separate USB port from LoRa
    // telemetry, see config.h's DEBUG_SERIAL macro -- never costs airtime).
    {
        static uint32_t lastPrintMs = 0;
        if (debugPrintReady(lastPrintMs)) {
            DEBUG_SERIAL.print("[POWER RAW] voltage_v="); DEBUG_SERIAL.print(_data.voltage_v, 3);
            DEBUG_SERIAL.print(" current_ma="); DEBUG_SERIAL.print(_data.current_ma, 1);
            DEBUG_SERIAL.print(" power_mw="); DEBUG_SERIAL.println(_data.power_mw, 1);
        }
    }

    return true;
}
