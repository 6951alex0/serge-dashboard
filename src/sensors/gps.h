#pragma once
#include <cstdint>
// GPS u-blox NEO - lecture NMEA via TinyGPS++ et configuration 20 Hz (UBX).

void gps_begin();       // lance la tache de lecture (uart + configuration NEO)
bool gps_timeReady();   // horloge GPS valide (pour horodater les sessions)
uint16_t gps_year();    // 0 si pas de fix
uint8_t gps_month();
uint8_t gps_day();
uint8_t gps_hour();
uint8_t gps_minute();
uint8_t gps_second();
