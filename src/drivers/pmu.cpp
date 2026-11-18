#define XPOWERS_CHIP_AXP2101
#include "pmu.h"
#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include "config.h"

namespace {

XPowersPMU g_pmu;
bool g_online = false;

} // namespace

bool pmu_begin()
{
    Wire.begin(IIC_SDA, IIC_SCL, 400000);
    if (!g_pmu.begin(Wire, AXP2101_ADDR, IIC_SDA, IIC_SCL)) {
        g_online = false;
        return false;
    }
    g_online = true;
    return true;
}

bool pmu_read(float *vbatt_v, uint8_t *pct)
{
    if (!g_online) {
        *vbatt_v = 0;
        *pct = 255;
        return false;
    }
    *vbatt_v = g_pmu.getBattVoltage() / 1000.0f;
    if (g_pmu.isBatteryConnect()) {
        int p = g_pmu.getBatteryPercent();
        if (p < 0) p = 0;
        if (p > 100) p = 100;
        *pct = (uint8_t)p;
    } else {
        *pct = 255;   // alimentation USB uniquement
    }
    return true;
}
