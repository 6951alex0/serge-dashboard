#pragma once
// PMU AXP2101 : tension batterie + niveau (tellemetrie + autonomie).

#include <cstdint>

bool pmu_begin();
bool pmu_read(float *vbatt_v, uint8_t *pct);
