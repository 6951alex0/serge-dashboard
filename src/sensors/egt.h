#pragma once
// EGT : thermocouple K + MAX6675.
// Le MAX6675 met 220 ms a convertir : le thread d'acquisition demande une
// lecture toutes les EGT_READ_PERIOD_MS et la valeur est interpolee en continu.

#include <cstdint>

bool egt_begin();                 // false si pas de thermocouple detecte
float egt_current();              // valeur interpolee courante (deg C)
void egt_tick();                  // declenche les lectures asynchrones
