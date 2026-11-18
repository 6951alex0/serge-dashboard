#include "rpm.h"
#include <Arduino.h>
#include <esp_timer.h>
#include "config.h"

// Interruption minimale : on horodate chaque front descendant.
// La conversion periode -> RPM se fait hors interruption.

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_prevUs = 0;
volatile uint32_t g_lastUs = 0;
volatile uint32_t g_count = 0;

float g_smoothRpm = 0;
uint32_t g_lastCountSeen = 0;
uint32_t g_lastNonZeroMs = 0;

void IRAM_ATTR rpmEdge()
{
    uint32_t now = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);
    portENTER_CRITICAL_ISR(&g_mux);
    // Saturer la difference theorique si wrap (improbable : 71 min)
    g_prevUs = g_lastUs;
    g_lastUs = now;
    g_count++;
    portEXIT_CRITICAL_ISR(&g_mux);
}

} // namespace

void rpm_begin()
{
    pinMode(RPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmEdge, FALLING);
}

uint16_t rpm_read()
{
    uint32_t prev, last, cnt;
    portENTER_CRITICAL(&g_mux);
    prev = g_prevUs;
    last = g_lastUs;
    cnt = g_count;
    portEXIT_CRITICAL(&g_mux);

    uint32_t nowMs = millis();

    if (cnt == g_lastCountSeen && g_smoothRpm > 0) {
        // Aucun front depuis le dernier appel : decroissance vers 0
        if (nowMs - g_lastNonZeroMs > 600) {
            g_smoothRpm *= 0.80f;
        }
        if (g_smoothRpm < RPM_IDLE_SPEED * 0.5f) g_smoothRpm = 0;
        return (uint16_t)(g_smoothRpm + 0.5f);
    }
    g_lastCountSeen = cnt;

    float periodUs = 0;
    if (cnt >= 2 && last > prev) periodUs = (float)(last - prev);
    // antirebond/plausibilite : bornes 300..24000 tr/min
    if (periodUs < 2500.0f || periodUs > 200000.0f) {
        return (uint16_t)(g_smoothRpm + 0.5f);
    }
    float rpm = 60.0e6f / periodUs;
    if (g_smoothRpm <= 0) {
        g_smoothRpm = rpm;
    } else {
        // lissage plus rapide a la montee (reponse moteur)
        float a = (rpm > g_smoothRpm) ? 0.6f : 0.3f;
        g_smoothRpm = g_smoothRpm * (1.0f - a) + rpm * a;
    }
    g_lastNonZeroMs = nowMs;
    if (g_smoothRpm > RPM_MAX_SPEED) g_smoothRpm = RPM_MAX_SPEED;
    return (uint16_t)(g_smoothRpm + 0.5f);
}
