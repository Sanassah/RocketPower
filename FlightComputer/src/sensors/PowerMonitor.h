#pragma once
#include <Adafruit_INA260.h>

struct PowerData {
    float voltage_v;
    float current_ma;
    float power_mw;
    bool  valid;
};

class PowerMonitor {
public:
    bool begin();
    bool update();
    const PowerData& data() const { return _data; }

private:
    Adafruit_INA260 _ina;
    PowerData       _data{};
};
