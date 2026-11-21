#include <Arduino.h>
#include "config.h"
#include "data_model.h"
#include "drivers/screen.h"
#include "drivers/pmu.h"
#include "sensors/rpm.h"
#include "sensors/egt.h"
#include "sensors/gps.h"
#include "compute/acq.h"
#include "storage/sd_logger.h"
#include "ble/ble_server.h"
#include "ui/ui.h"

// ---------------------------------------------------------------------------
// Serge Dashboard - ESP32-S3, tableau de bord de telemetrie 2 temps.
//
// Philosophie : l'ESP32-S3 est autonome (mesure, calculs, affichage, log SD).
// Le Pixel 5 est un complement via BLE (live 20 Hz + export des sessions).
//
// Repartition FreeRTOS :
//   coeur 0 : UI LVGL + BLE (live 20 Hz, transfert sessions)
//   coeur 1 : acquisition 20 Hz + GPS + log microSD
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.printf("Serge Dashboard v%s\n", FIRMWARE_VERSION);

    store_init();
    pmu_begin();
    rpm_begin();
    egt_begin();
    sd_begin();

    if (!screen_begin()) {
        Serial.println("[setup] arret : ecran indisponible");
        while (true) delay(1000);
    }

    ble_start();     // serveur GATT (coeur 0)
    acq_start();     // acquisition 20 Hz (coeur 1)
    gps_begin();     // parse NMEA + config NEO 20 Hz (coeur 1)
    ui_start();      // LVGL + 5 pages (coeur 1)

    Serial.println("[setup] OK");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
