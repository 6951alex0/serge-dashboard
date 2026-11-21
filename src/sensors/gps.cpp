#include "gps.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "data_model.h"
#include "config.h"

namespace {

TinyGPSPlus g_parser;
HardwareSerial &g_serial = Serial1;

uint16_t g_year = 0;
uint8_t g_month = 0, g_day = 0, g_hour = 0, g_minute = 0, g_second = 0;

// Configuration UBX-CFG-RATE : periode de mesure 50 ms = 20 Hz.
// Un NEO-6M au-dela de sa limite ignorera ou rejettera la trame :
// le code reste fonctionnel a sa cadence maximale.
const uint8_t UBX_CFG_RATE_20HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0x32, 0x00,        // measRate = 50 ms
    0x01, 0x00,        // navRate = 1
    0x01, 0x00,        // timeRef = GPS
    0x48, 0x69         // checksum (CK_A, CK_B) pour measRate=50, nav=1, ref=GPS
};

void commitFromParser(uint32_t msNow)
{
    (void)msNow;
    GpsFix fix{};
    fix.valid = g_parser.location.isValid() && g_parser.location.age() < 500;
    if (g_parser.location.isValid()) {
        fix.lat_e7 = (int32_t)(g_parser.location.lat() * 1e7);
        fix.lon_e7 = (int32_t)(g_parser.location.lng() * 1e7);
    }
    fix.altitude_m = g_parser.altitude.isValid() ? (float)g_parser.altitude.meters() : 0;
    fix.heading_deg = g_parser.course.isValid() ? (float)g_parser.course.deg() : 0;
    fix.speed_mps = g_parser.speed.isValid() ? (float)g_parser.speed.mps() : 0;
    fix.sats = g_parser.satellites.isValid() ? (int8_t)g_parser.satellites.value() : 0;
    fix.hdop = g_parser.hdop.isValid() ? (float)g_parser.hdop.value() / 100.0f : 99.9f;
    if (g_parser.date.isValid() && g_parser.time.isValid()) {
        g_year = g_parser.date.year();
        g_month = g_parser.date.month();
        g_day = g_parser.date.day();
        g_hour = g_parser.time.hour();
        g_minute = g_parser.time.minute();
        g_second = g_parser.time.second();
    }
    gps_commit(fix);
}

} // namespace

bool gps_timeReady() { return g_year != 0; }
uint16_t gps_year() { return g_year; }
uint8_t gps_month() { return g_month; }
uint8_t gps_day() { return g_day; }
uint8_t gps_hour() { return g_hour; }
uint8_t gps_minute() { return g_minute; }
uint8_t gps_second() { return g_second; }

void gpsTask(void *param)
{
    (void)param;

    g_serial.begin(GPS_BAUD_DEFAULT, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);

    g_serial.write(UBX_CFG_RATE_20HZ, sizeof(UBX_CFG_RATE_20HZ));
    g_serial.flush();
    delay(100);

    uint32_t lastCommitMs = 0;

    for (;;) {
        while (g_serial.available()) {
            g_parser.encode((char)g_serial.read());
        }
        uint32_t now = millis();
        if ((g_parser.location.isUpdated() || g_parser.speed.isUpdated()) &&
            now - lastCommitMs >= 40) {   // ~20 Hz max
            lastCommitMs = now;
            commitFromParser(now);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void gps_begin()
{
    xTaskCreatePinnedToCore(gpsTask, "gps", 8192, nullptr, 2, nullptr, 1);
}
