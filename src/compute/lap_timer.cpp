#include "lap_timer.h"
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "data_model.h"

namespace {

constexpr float EARTH_R_M = 6371000.0f;
constexpr float DEG = 3.14159265f / 180.0f;

bool g_armed = false;
uint32_t g_lastCrossMs = 0;

float distMeters(const GpsFix &a, double lat2, double lon2)
{
    // approximation equirectangulaire, largement suffisante < 100 m
    float lat1 = (float)(a.lat_e7 / 1e7);
    float dLat = (float)(lat2 - lat1);
    float dLon = (float)(lon2 - a.lon_e7 / 1e7) * cosf(lat1 * DEG);
    return sqrtf(dLat * dLat + dLon * dLon) * DEG * EARTH_R_M;
}

void crossLine(uint32_t nowMs)
{
    ChronoState c = chrono_get();
    uint32_t lapMs = nowMs - c.lap_start_ms;

    if (c.running) {
        if (lapMs > 5000 && (!c.best_lap_ms || lapMs < c.best_lap_ms)) {
            c.best_lap_ms = lapMs;
        }
        c.last_lap_ms = lapMs;
        c.lap_count++;
    }
    c.running = true;
    c.lap_start_ms = nowMs;
    c.current_lap_ms = 0;
    chrono_commit(c);

    g_lastCrossMs = nowMs;
    g_armed = false;
}

} // namespace

void lap_triggerManual(uint32_t nowMs)
{
    crossLine(nowMs);
}

void lap_timer_tick(uint32_t nowMs, const GpsFix &gps)
{
    VehicleSettings s = settings_get();
    if (s.lap_lat == 0.0 && s.lap_lon == 0.0) return;
    if (!gps.valid || gps.speed_mps < LAP_MIN_SPEED_MPS) return;

    float d = distMeters(gps, s.lap_lat, s.lap_lon);
    float rearm = s.lap_radius_m * LAP_REARM_FACTOR;

    if (!g_armed) {
        if (d > rearm) g_armed = true;
        return;
    }

    if (d <= s.lap_radius_m && nowMs - g_lastCrossMs > 5000) {
        crossLine(nowMs);
    }
}
