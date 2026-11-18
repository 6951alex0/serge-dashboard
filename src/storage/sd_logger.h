#pragma once
// Enregistrement de sessions sur microSD (CSV 20 Hz) + listing pour BLE.

#include <cstdint>
#include <vector>
#include <string>
#include "data_model.h"

bool sd_begin();
bool sd_ready();   // microSD presente depuis le demarrage
bool sd_logger_startSession(bool gpsTimeAvailable);   // cree le fichier CSV
void sd_logger_stopSession();
bool sd_logger_isRecording();
void sd_logger_write(const TelemetryFrame &f);        // appele a 20 Hz par acq

// Pour la page Sessions et le transfert BLE
std::vector<std::string> sd_listSessions();
bool sd_readFileChunk(const std::string &name, uint32_t offset,
                      uint8_t *buf, uint32_t bufLen, uint32_t *outLen);
uint32_t sd_fileSize(const std::string &name);
