#include "data_model.h"
#include <Arduino.h>
#include <Preferences.h>

namespace {

SemaphoreHandle_t g_mutex = nullptr;

TelemetryFrame g_tlm = {};
ChronoState g_chrono = {};
GpsFix g_gps = {};
VehicleSettings g_settings = {};

volatile bool g_recReqFlag = false;
volatile bool g_recReqValue = false;
volatile bool g_lapReqFlag = false;

constexpr char NVS_NS[] = "serge";

} // namespace

void store_init()
{
    g_mutex = xSemaphoreCreateMutex();
    g_tlm.egt_c = 0;
    g_tlm.egt_max_c = 0;

    g_settings.mass_kg = MASS_KG_DEFAULT;
    g_settings.cxsa = CXSA_DEFAULT;
    g_settings.eta = ETA_DEFAULT;
    g_settings.crr = CRR_DEFAULT;
    g_settings.lap_lat = 0.0;
    g_settings.lap_lon = 0.0;
    g_settings.lap_radius_m = LAP_RADIUS_DEFAULT_M;
    snprintf(g_settings.device_name, sizeof(g_settings.device_name), "%s", BLE_DEVICE_NAME);

    Preferences prefs;
    prefs.begin(NVS_NS, true);
    if (prefs.isKey("mass")) g_settings.mass_kg = prefs.getFloat("mass", g_settings.mass_kg);
    if (prefs.isKey("cxsa")) g_settings.cxsa = prefs.getFloat("cxsa", g_settings.cxsa);
    if (prefs.isKey("eta")) g_settings.eta = prefs.getFloat("eta", g_settings.eta);
    if (prefs.isKey("crr")) g_settings.crr = prefs.getFloat("crr", g_settings.crr);
    if (prefs.isKey("lapen")) {
        g_settings.lap_lat = prefs.getDouble("laplat", g_settings.lap_lat);
        g_settings.lap_lon = prefs.getDouble("laplon", g_settings.lap_lon);
        g_settings.lap_radius_m = prefs.getFloat("lapr", g_settings.lap_radius_m);
    }
    prefs.end();
}

static void lock() { xSemaphoreTake(g_mutex, portMAX_DELAY); }
static void unlock() { xSemaphoreGive(g_mutex); }

// ------------------------------- Télémétrie --------------------------------

void telemetry_commit(const TelemetryFrame &f)
{
    lock();
    g_tlm = f;
    unlock();
}

TelemetryFrame telemetry_get()
{
    lock();
    TelemetryFrame f = g_tlm;
    unlock();
    return f;
}

// ---------------------------------- GPS ------------------------------------

void gps_commit(const GpsFix &fix)
{
    lock();
    g_gps = fix;
    unlock();
}

GpsFix gps_get()
{
    lock();
    GpsFix f = g_gps;
    unlock();
    return f;
}

// --------------------------------- Chrono ----------------------------------

void chrono_commit(const ChronoState &c)
{
    lock();
    g_chrono = c;
    unlock();
}

ChronoState chrono_get()
{
    lock();
    ChronoState c = g_chrono;
    unlock();
    return c;
}

// ------------------------------- Paramètres --------------------------------

VehicleSettings settings_get()
{
    lock();
    VehicleSettings s = g_settings;
    unlock();
    return s;
}

void settings_update(const VehicleSettings &s)
{
    lock();
    g_settings = s;
    unlock();

    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.putFloat("mass", s.mass_kg);
    prefs.putFloat("cxsa", s.cxsa);
    prefs.putFloat("eta", s.eta);
    prefs.putFloat("crr", s.crr);
    prefs.putFloat("lapr", s.lap_radius_m);
    if (s.lap_lat != 0.0 || s.lap_lon != 0.0) {
        prefs.putBool("lapen", true);
        prefs.putDouble("laplat", s.lap_lat);
        prefs.putDouble("laplon", s.lap_lon);
    }
    prefs.end();
}

void settings_lapLineHere(const GpsFix &pos)
{
    if (!pos.valid) return;
    VehicleSettings s = settings_get();
    s.lap_lat = pos.lat_e7 / 1e7;
    s.lap_lon = pos.lon_e7 / 1e7;
    settings_update(s);
}

// ------------------------------- Commandes ---------------------------------

void cmds_requestRecord(bool on)
{
    g_recReqValue = on;
    g_recReqFlag = true;
}

bool cmds_recordRequestActive()
{
    return g_recReqFlag;
}

bool cmds_recordRequestValue()
{
    return g_recReqValue;
}

bool cmds_popRecordRequest(bool &start)
{
    if (!g_recReqFlag) return false;
    g_recReqFlag = false;
    start = g_recReqValue;
    return true;
}

void cmds_requestLap()
{
    g_lapReqFlag = true;
}

bool cmds_popLapRequest()
{
    if (!g_lapReqFlag) return false;
    g_lapReqFlag = false;
    return true;
}

