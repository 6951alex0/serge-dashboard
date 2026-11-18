#pragma once
// Chrono GPS : passage de la ligne départ/arrivée définie dans les paramètres.

#include <cstdint>
#include "data_model.h"

void lap_timer_tick(uint32_t nowMs, const GpsFix &gps);
void lap_triggerManual(uint32_t nowMs);   // bouton tactile (secours)
