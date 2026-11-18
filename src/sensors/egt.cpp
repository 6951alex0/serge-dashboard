#include "egt.h"
#include <Arduino.h>
#include "config.h"

// Lecture SPI logicielle du MAX6675 (16 bits, horloge < 4.3 MHz - on est a ~1 kHz).
// Format: D15..D3 = temperature * 0.25 C en 12 bits, D2 = thermocouple ouvert.

namespace {

bool g_online = false;
uint16_t g_raw = 0;       // 12-bit raw (0.25 C, soit degC = raw / 4)
uint32_t g_rawAtMs = 0;
float g_prevDeg = 0;

uint16_t shiftIn16()
{
    pinMode(EGT_MISO, INPUT);
    pinMode(EGT_SCK, OUTPUT);
    digitalWrite(EGT_SCK, LOW);

    uint16_t word = 0;
    for (int i = 0; i < 16; i++) {
        digitalWrite(EGT_SCK, HIGH);
        delayMicroseconds(1);
        word <<= 1;
        if (digitalRead(EGT_MISO)) word |= 1;
        digitalWrite(EGT_SCK, LOW);
        delayMicroseconds(1);
    }
    return word;
}

void startConversion()
{
    pinMode(EGT_CS, OUTPUT);
    pinMode(EGT_SCK, OUTPUT);
    pinMode(EGT_MISO, INPUT);
    delayMicroseconds(5);
    digitalWrite(EGT_CS, LOW);
    delayMicroseconds(5);
    digitalWrite(EGT_CS, HIGH);      // CS haut => demarre une nouvelle conversion
}

} // namespace

bool egt_begin()
{
    pinMode(EGT_CS, OUTPUT);
    digitalWrite(EGT_CS, HIGH);
    pinMode(EGT_SCK, OUTPUT);
    digitalWrite(EGT_SCK, LOW);
    pinMode(EGT_MISO, INPUT);
    delay(300);                      // premiere conversion
    digitalWrite(EGT_CS, LOW);
    uint16_t w = shiftIn16();
    digitalWrite(EGT_CS, HIGH);
    bool open = w & 0x0004;          // D2 : thermocouple ouvert
    g_online = !open;
    if (g_online) {
        g_raw = (w >> 3) & 0x0FFF;
        g_rawAtMs = millis();
        g_prevDeg = g_raw * 0.25f;
    }
    return g_online;
}

float egt_current()
{
    // Interpolation lineaire entre deux conversions (220 ms)
    if (!g_online) return -1.0f;
    uint32_t now = millis();
    float frac = (float)(now - g_rawAtMs) / (float)EGT_READ_PERIOD_MS;
    if (frac > 1.0f) frac = 1.0f;
    float prev = g_prevDeg;
    float nowDeg = g_raw * 0.25f;
    return prev + (nowDeg - prev) * frac;
}

void egt_tick()
{
    // Le MAX6675 a besoin de 220 ms pour convertir : on lit a 250 ms.
    uint32_t now = millis();
    if (now - g_rawAtMs < EGT_READ_PERIOD_MS + 30) return;
    digitalWrite(EGT_CS, LOW);
    delayMicroseconds(5);
    uint16_t w = shiftIn16();
    digitalWrite(EGT_CS, HIGH);      // CS haut = relance une nouvelle conversion
    if (w & 0x0004) return;          // thermocouple ouvert : garder la derniere valeur
    g_prevDeg = g_raw * 0.25f;
    g_raw = (w >> 3) & 0x0FFF;
    g_rawAtMs = now;
}
