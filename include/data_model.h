#pragma once
// ---------------------------------------------------------------------------
// Modèle de données partagé entre les tâches (acquisition, UI, BLE, log SD).
// Un seul écrivain par rubrique, lecteurs multiples via verrou.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// ------------------------------ Télémétrie --------------------------------

struct TelemetryFrame {
    uint32_t t_ms;              // ms depuis le démarrage de la session

    float speed_kmh;            // GPS brute
    float speed_mps;            // filtrée (calculs)
    float egt_c;                // EGT interpolée
    float egt_max_c;            // maximum mémorisé de la session
    uint16_t rpm;               // régime moteur

    float g_long;               // accélération longitudinale [g]
    float g_lat;                // accélération latérale [g]
    float g_vert;

    float bank_deg;             // inclinaison gauche/droite (+: droite)
    float pitch_deg;            // pente / wheelie-estimée (+: pente montante)

    float power_kw;             // puissance estimée à la roue
    float torque_nm;            // couple moteur estimé

    int32_t lat_e7, lon_e7;     // position GPS * 1e7 (0,0 si pas de fix)
    float altitude_m;
    float heading_deg;
    int8_t sats;
    bool fix;

    float vbatt_v;
    uint8_t batt_pct;           // 0..100, 255 si PMU absent
    bool recording;
};

// ------------------------------ Chronométrage -------------------------------

struct ChronoState {
    bool running;
    uint32_t lap_start_ms;
    uint32_t current_lap_ms;    // ms depuis le début du tour courant
    uint32_t last_lap_ms;       // 0 = aucun
    uint32_t best_lap_ms;       // 0 = aucun
    uint16_t lap_count;
};

// ------------------------------ Paramètres ---------------------------------

struct VehicleSettings {
    float mass_kg;
    float cxsa;                 // Cx * S [m^2]
    float eta;                  // rendement transmission
    float crr;                  // résistance roulement
    double lap_lat;             // ligne départ/arrivée (degrés)
    double lap_lon;
    float lap_radius_m;
    char device_name[16];
};

struct GpsFix {
    bool valid;
    int32_t lat_e7, lon_e7;     // deg * 1e7
    float altitude_m;
    float heading_deg;
    float speed_mps;
    int8_t sats;
    float hdop;                 // dilution de precision (< 1 excellent, > 2 moyen)
    uint16_t year;
    uint8_t month, day, hour, minute, second;
};

// --------------------------------- API --------------------------------------

void store_init();

// Télémétrie : un écrivain (tâche d'acquisition)
void telemetry_commit(const TelemetryFrame &f);
TelemetryFrame telemetry_get();

// GPS (écrivain : tâche GPS)
void gps_commit(const GpsFix &fix);
GpsFix gps_get();

// Chrono (écrivain : tâche d'acquisition)
void chrono_commit(const ChronoState &c);
ChronoState chrono_get();

// Paramètres (lecture copiée, écriture + persistance NVS)
VehicleSettings settings_get();
void settings_update(const VehicleSettings &s);   // persiste en NVS
void settings_lapLineHere(const GpsFix &pos);     // "ligne ici" = position GPS courante

// Commandes croisées (UI / BLE -> tâches)
// Record ; pop() retournera false si aucune demande en attente.
void cmds_requestRecord(bool on);
bool cmds_popRecordRequest(bool &start);
void cmds_requestLap();
bool cmds_popLapRequest();
