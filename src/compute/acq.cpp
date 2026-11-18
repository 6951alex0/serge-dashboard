#include "acq.h"
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "data_model.h"
#include "sensors/imu.h"
#include "sensors/rpm.h"
#include "sensors/egt.h"
#include "sensors/gps.h"
#include "compute/lap_timer.h"
#include "storage/sd_logger.h"
#include "drivers/pmu.h"

namespace {

float g_speedFilt = 0;        // m/s
float g_gLatFilt = 0;
float g_bankFilt = 0;
float g_pitchFilt = 0;
float g_powerFilt = 0;
float g_measAxOffset = 0.0f;  // centrage gravimétrique (calibration à plat)
float g_measAyOffset = 0.0f;

float g_prevSpeedMps = 0;

float g_gLongDeriv = 0;       // accel. longitudinale derivee du GPS [g]

float pitchFromAcc(float ax, float ay, float az)
{
    // Carte montée verticale : extrait la composante longitudinale de la gravité.
    float horiz = ax - g_measAxOffset;
    float v = sqrtf(ay * ay + az * az);
    if (v < 0.05f && fabsf(horiz) < 0.05f) return 0;
    return atan2f(-horiz, v) * 57.2958f;
}

float bankFromAcc(float ay, float az)
{
    float ayC = ay - g_measAyOffset;
    return atan2f(ayC, az) * 57.2958f;
}

} // namespace

void acqTask(void *param)
{
    (void)param;
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastAcqMs = millis();
    bool recording = false;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ACQ_PERIOD_MS));
        uint32_t nowMs = millis();
        float dt = (nowMs - lastAcqMs) / 1000.0f;
        if (dt <= 0.001f) dt = 0.001f;
        lastAcqMs = nowMs;

        // ---------------- Capteurs ----------------
        float ax, ay, az, gx, gy, gz;
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);
        (void)gx; (void)gy; (void)gz;      // gyro : lecture réservée aux extensions

        uint16_t rpm = rpm_read();
        egt_tick();
        float egt = egt_current();

        GpsFix gps = gps_get();

        // ---------------- Dérivées / filtres ----------------
        float speedMps = gps.valid ? gps.speed_mps : 0;
        g_speedFilt += (speedMps - g_speedFilt) * SPEED_FILTER_ALPHA;

        if (dt > 0.001f) {
            float deriv = (g_speedFilt - g_prevSpeedMps) / dt / 9.80665f;
            g_prevSpeedMps = g_speedFilt;
            // passe-bas sur la dérivée pour cantonner le bruit GPS
            g_gLongDeriv += (deriv - g_gLongDeriv) * G_FILTER_ALPHA;
        }

        g_gLatFilt += ((ay - g_measAyOffset) - g_gLatFilt) * G_FILTER_ALPHA;

        g_bankFilt += (bankFromAcc(ay, az) - g_bankFilt) * 0.30f;
        g_pitchFilt += (pitchFromAcc(ax, ay, az) - g_pitchFilt) * 0.20f;

        // ---------------- Puissance / couple ----------------
        VehicleSettings settings = settings_get();
        float powerW = 0;
        if (g_speedFilt > 0.5f) {
            float aMs2 = g_gLongDeriv * 9.80665f;
            float fInertia = settings.mass_kg * aMs2;
            float fSlope = settings.mass_kg * 9.80665f * sinf(g_pitchFilt * 0.0174533f);
            float fAero = 0.5f * AIR_DENSITY * settings.cxsa * g_speedFilt * g_speedFilt;
            float fRoll = settings.crr * settings.mass_kg * 9.80665f *
                          cosf(g_pitchFilt * 0.0174533f);
            float driveForce = fInertia + fSlope + fAero + fRoll;
            powerW = driveForce * g_speedFilt / settings.eta;
        }
        if (powerW < 0) powerW = 0;
        g_powerFilt += ((powerW / 1000.0f) - g_powerFilt) * POWER_FILTER_ALPHA;

        float torqueNm = 0;
        if (rpm > 200) {
            float omega = 2.0f * 3.14159265f * rpm / 60.0f;
            torqueNm = (g_powerFilt * 1000.0f / omega) * settings.eta;
        }

        // ---------------- EGT max ----------------
        static float egtMax = 0;
        if (egt > egtMax) egtMax = egt;

        // ---------------- Chrono GPS ----------------
        lap_timer_tick(nowMs, gps);

        // ---------------- Commandes croisées ----------------
        bool reqStart = false;
        if (cmds_popRecordRequest(reqStart)) {
            if (reqStart && !recording) {
                if (sd_logger_startSession(gps_timeReady())) {
                    recording = true;
                }
            } else if (!reqStart && recording) {
                sd_logger_stopSession();
                recording = false;
            }
            ChronoState c = chrono_get();
            c.running = recording;
            if (recording) c.lap_start_ms = nowMs;
            chrono_commit(c);
        }
        if (cmds_popLapRequest()) {
            lap_triggerManual(nowMs);
        }

        ChronoState c2 = chrono_get();
        if (c2.running) {
            c2.current_lap_ms = nowMs - c2.lap_start_ms;
            chrono_commit(c2);
        }

        // ---------------- Batterie ----------------
        float vbatt = 0;
        uint8_t bpct = 255;
        pmu_read(&vbatt, &bpct);

        // ---------------- Publication ----------------
        TelemetryFrame f{};
        f.t_ms = nowMs;
        f.speed_kmh = g_speedFilt * 3.6f;
        f.speed_mps = g_speedFilt;
        f.egt_c = egt;
        f.egt_max_c = egtMax;
        f.rpm = rpm;
        f.g_long = g_gLongDeriv;
        f.g_lat = g_gLatFilt;
        f.g_vert = az - 1.0f;
        f.bank_deg = g_bankFilt;
        f.pitch_deg = g_pitchFilt;
        f.power_kw = g_powerFilt;
        f.torque_nm = torqueNm;
        f.lat_e7 = gps.lat_e7;
        f.lon_e7 = gps.lon_e7;
        f.altitude_m = gps.altitude_m;
        f.heading_deg = gps.heading_deg;
        f.sats = gps.sats;
        f.fix = gps.valid;
        f.vbatt_v = vbatt;
        f.batt_pct = bpct;
        f.recording = recording;
        telemetry_commit(f);

        if (recording) {
            sd_logger_write(f);
        }
    }
}

void acq_start()
{
    xTaskCreatePinnedToCore(acqTask, "acq", 8192, nullptr, 3, nullptr, 1);
}

void acq_calibrateFlat()
{
    // Recentre la gravité mesurée à l'arrêt (moto verticale, moteur coupé).
    float sx = 0, sy = 0;
    float ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 20; i++) {
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);
        sx += ax;
        sy += ay;
        delay(20);
    }
    g_measAxOffset = sx / 20.0f;
    g_measAyOffset = sy / 20.0f;

    g_speedFilt = 0;
    g_prevSpeedMps = 0;
    g_gLongDeriv = 0;
    g_powerFilt = 0;
}
