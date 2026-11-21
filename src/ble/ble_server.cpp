#include "ble_server.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"
#include "data_model.h"
#include "storage/sd_logger.h"

namespace {

constexpr size_t BLE_MTU_SAFE = 180;

BLEServer *g_server = nullptr;
BLECharacteristic *g_charLive = nullptr;
BLECharacteristic *g_charCtrl = nullptr;
BLECharacteristic *g_charFile = nullptr;
volatile bool g_clientConnected = false;

struct TransferState {
    bool active;
    std::string name;
    uint32_t size;
    uint32_t offset;
    uint32_t seq;
} g_transfer = {};

SemaphoreHandle_t g_transferMutex = nullptr;

void notifyFile(const char *data, size_t len)
{
    g_charFile->setValue((uint8_t *)data, len);
    g_charFile->notify();
}

void transferStart(const std::string &name)
{
    xSemaphoreTake(g_transferMutex, portMAX_DELAY);
    g_transfer.active = false;
    g_transfer.name = name;
    g_transfer.size = sd_fileSize(name);
    g_transfer.offset = 0;
    g_transfer.seq = 0;
    if (g_transfer.size > 0) {
        g_transfer.active = true;
        char hdr[96];
        int n = snprintf(hdr, sizeof(hdr), "FHDR %lu %s",
                         (unsigned long)g_transfer.size, name.c_str());
        if (n > 0) notifyFile(hdr, (size_t)n);
    }
    xSemaphoreGive(g_transferMutex);
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *s) override
    {
        g_clientConnected = true;
        s->getAdvertising()->start();
    }
    void onDisconnect(BLEServer *s) override
    {
        g_clientConnected = false;
        xSemaphoreTake(g_transferMutex, portMAX_DELAY);
        g_transfer.active = false;
        xSemaphoreGive(g_transferMutex);
        s->getAdvertising()->start();
    }
};

void handleCtrl(const String &cmd)
{
    if (cmd == "REC") {
        cmds_requestRecord(true);
    } else if (cmd == "STOP") {
        cmds_requestRecord(false);
    } else if (cmd == "LAP") {
        cmds_requestLap();
    } else if (cmd == "LIST") {
        auto files = sd_listSessions();
        std::string batch;
        for (auto &f : files) {
            std::string line = "F " + f + " " + std::to_string(sd_fileSize(f)) + "\n";
            if (batch.size() + line.size() > BLE_MTU_SAFE) {
                notifyFile(batch.data(), batch.size());
                batch.clear();
                vTaskDelay(pdMS_TO_TICKS(15));
            }
            batch += line;
        }
        batch += "FLIST_END\n";
        notifyFile(batch.data(), batch.size());
    } else if (cmd.startsWith("GET ")) {
        transferStart(std::string(cmd.c_str() + 4));
    }
}

class CtrlCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override
    {
        String v = c->getValue();
        if (v.length() > 0) handleCtrl(v);
    }
};

size_t packLiveFrame(uint8_t *buf)   // 70 octets little-endian
{
    TelemetryFrame f = telemetry_get();
    ChronoState ch = chrono_get();
    size_t o = 0;
    buf[o++] = 'S';
    buf[o++] = 'G';
    auto putU16 = [&](uint16_t v) { memcpy(buf + o, &v, 2); o += 2; };
    auto putU32 = [&](uint32_t v) { memcpy(buf + o, &v, 4); o += 4; };
    auto putI32 = [&](int32_t v) { memcpy(buf + o, &v, 4); o += 4; };
    auto putF = [&](float v) { memcpy(buf + o, &v, 4); o += 4; };

    putU16(f.rpm);
    putF(f.speed_kmh);
    putF(f.egt_c);
    putF(f.egt_max_c);
    putF(f.g_long);
    putF(f.g_lat);
    putF(f.bank_deg);
    putF(f.power_kw);
    putF(f.torque_nm);
    putU32(f.t_ms);
    putI32(f.lat_e7);
    putI32(f.lon_e7);
    putF(f.altitude_m);
    buf[o++] = (uint8_t)f.sats;
    buf[o++] = f.fix ? 1 : 0;
    buf[o++] = f.batt_pct;
    buf[o++] = f.recording ? 1 : 0;
    putU32(ch.current_lap_ms);
    putU32(ch.best_lap_ms);
    putU32(ch.last_lap_ms);
    putU16(ch.lap_count);
    return o;   // = 70
}

void bleLiveTask(void *param)
{
    (void)param;
    uint8_t buf[70];
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ACQ_PERIOD_MS));
        if (!g_clientConnected) continue;
        size_t n = packLiveFrame(buf);
        g_charLive->setValue(buf, n);
        g_charLive->notify();
    }
}

void bleTransferTask(void *param)
{
    (void)param;
    uint8_t buf[BLE_FILE_CHUNK + 8];
    uint32_t sinceAck = 0;

    for (;;) {
        bool sent = false;
        xSemaphoreTake(g_transferMutex, portMAX_DELAY);
        if (g_transfer.active && g_clientConnected) {
            uint32_t outLen = 0;
            if (sd_readFileChunk(g_transfer.name, g_transfer.offset,
                                 buf + 8, BLE_FILE_CHUNK, &outLen)) {
                buf[0] = 'D';
                buf[1] = 'A';
                buf[2] = 'T';
                buf[3] = ' ';
                memcpy(buf + 4, &g_transfer.seq, 4);
                g_charFile->setValue(buf, 8 + BLE_FILE_CHUNK);
                g_charFile->notify();
                g_transfer.offset += outLen;
                g_transfer.seq++;
                sent = true;
                sinceAck++;
                if (g_transfer.offset >= g_transfer.size) {
                    g_transfer.active = false;
                    notifyFile("FEND", 4);
                }
            } else {
                g_transfer.active = false;
            }
        }
        xSemaphoreGive(g_transferMutex);
        if (!sent) {
            sinceAck = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
        } else if (sinceAck >= BLE_FILE_ACK_EVERY) {
            vTaskDelay(pdMS_TO_TICKS(120));   // respiration : laisse le ACK passer
            sinceAck = 0;
        } else {
            vTaskDelay(pdMS_TO_TICKS(12));
        }
    }
}

} // namespace

void ble_start()
{
    g_transferMutex = xSemaphoreCreateMutex();

    BLEDevice::init(BLE_DEVICE_NAME);
    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    BLEService *svc = g_server->createService(BLE_SERVICE_UUID);

    g_charLive = svc->createCharacteristic(
        BLE_CHAR_LIVE_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    g_charLive->addDescriptor(new BLE2902());

    g_charCtrl = svc->createCharacteristic(
        BLE_CHAR_CTRL_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    g_charCtrl->setCallbacks(new CtrlCallbacks());

    g_charFile = svc->createCharacteristic(
        BLE_CHAR_FILE_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    g_charFile->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->start();

    xTaskCreatePinnedToCore(bleLiveTask, "bleLive", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(bleTransferTask, "bleXfer", 4096, nullptr, 1, nullptr, 0);
}

bool ble_connected()
{
    return g_clientConnected;
}
