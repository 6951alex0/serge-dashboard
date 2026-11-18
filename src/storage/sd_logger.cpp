#include "sd_logger.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "config.h"
#include "sensors/gps.h"

namespace {

SPIClass g_spiSD(HSPI);
bool g_sdOk = false;
File g_file;
bool g_recording = false;
uint32_t g_lastFlushMs = 0;
uint32_t g_lines = 0;

void csvHeader(File &f)
{
    f.println("t_ms,speed_kmh,rpm,egt_c,egt_max_c,g_long,g_lat,bank_deg,pitch_deg,"
              "power_kw,torque_nm,lat_e7,lon_e7,altitude_m,heading_deg,sats,fix,vbatt_v");
}

void csvLine(File &f, const TelemetryFrame &fr)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "%lu,%.2f,%u,%.1f,%.1f,%.3f,%.3f,%.1f,%.1f,"
             "%.3f,%.2f,%ld,%ld,%.1f,%.1f,%d,%d,%.2f",
             (unsigned long)fr.t_ms, fr.speed_kmh, fr.rpm, fr.egt_c, fr.egt_max_c,
             fr.g_long, fr.g_lat, fr.bank_deg, fr.pitch_deg,
             fr.power_kw, fr.torque_nm, (long)fr.lat_e7, (long)fr.lon_e7,
             fr.altitude_m, fr.heading_deg, fr.sats, fr.fix ? 1 : 0, fr.vbatt_v);
    f.println(buf);
}

void makeSessionName(char *out, size_t len, bool gpsTime)
{
    if (gpsTime && gps_timeReady()) {
        snprintf(out, len, "/sessions/SESS_%04u%02u%02u_%02u%02u%02u.csv",
                 gps_year(), gps_month(), gps_day(),
                 gps_hour(), gps_minute(), gps_second());
    } else {
        snprintf(out, len, "/sessions/SESS_%lu.csv", (unsigned long)millis());
    }
}

} // namespace

bool sd_begin()
{
    g_spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    g_sdOk = SD.begin(SD_CS, g_spiSD, SD_SPI_FREQ_HZ);
    if (g_sdOk && !SD.exists("/sessions")) {
        SD.mkdir("/sessions");
    }
    return g_sdOk;
}

bool sd_ready() { return g_sdOk; }

bool sd_logger_startSession(bool gpsTimeAvailable)
{
    if (!g_sdOk || g_recording) return g_recording;
    char name[48];
    makeSessionName(name, sizeof(name), gpsTimeAvailable);
    g_file = SD.open(name, FILE_WRITE);
    if (!g_file) return false;
    csvHeader(g_file);
    g_file.flush();
    g_recording = true;
    g_lines = 0;
    return true;
}

void sd_logger_stopSession()
{
    if (g_file) {
        g_file.flush();
        g_file.close();
    }
    g_recording = false;
}

bool sd_logger_isRecording() { return g_recording; }

void sd_logger_write(const TelemetryFrame &f)
{
    if (!g_recording || !g_file) return;
    csvLine(g_file, f);
    g_lines++;
    if (millis() - g_lastFlushMs > 2000) {   // flush toutes les 2 s
        g_lastFlushMs = millis();
        g_file.flush();
    }
}

std::vector<std::string> sd_listSessions()
{
    std::vector<std::string> out;
    if (!g_sdOk) return out;
    File dir = SD.open("/sessions");
    if (!dir) return out;
    File entry;
    while ((entry = dir.openNextFile())) {
        if (!entry.isDirectory()) {
            out.push_back(std::string(entry.name()));
        }
        entry.close();
    }
    dir.close();
    return out;
}

uint32_t sd_fileSize(const std::string &name)
{
    if (!g_sdOk) return 0;
    std::string path = name.rfind("/sessions/", 0) == 0 ? name : "/sessions/" + name;
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) return 0;
    uint32_t sz = f.size();
    f.close();
    return sz;
}

bool sd_readFileChunk(const std::string &name, uint32_t offset,
                      uint8_t *buf, uint32_t bufLen, uint32_t *outLen)
{
    *outLen = 0;
    if (!g_sdOk) return false;
    std::string path = name.rfind("/sessions/", 0) == 0 ? name : "/sessions/" + name;
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) return false;
    if (offset >= f.size()) {
        f.close();
        return false;
    }
    f.seek(offset);
    *outLen = f.read(buf, bufLen);
    f.close();
    return *outLen > 0;
}
