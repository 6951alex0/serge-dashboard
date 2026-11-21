#pragma once
// Régime moteur - reprise allumage MHR via optocoupleur H11L1.
// 1 front / tour (mono 2 temps) => RPM = 60 / periode.

#include <cstdint>

void rpm_begin();                 // configure l'interruption
uint16_t rpm_read();              // RPM lissé (0 si moteur arrête)
