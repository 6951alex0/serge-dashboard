#pragma once
// ---------------------------------------------------------------------------
// Serge Dashboard — configuration matérielle et véhicule
// Carte : Waveshare ESP32-S3-Touch-AMOLED-2.16 (ESP32-S3R8, 8 Mo PSRAM)
// ---------------------------------------------------------------------------

#define FIRMWARE_VERSION "0.1.0"

// ------------------------------- Écran AMOLED ------------------------------
// CO5300 en QSPI, 480x480, aucune broche de rétro-éclairage (commande de
// luminosité via gfx->setBrightness()).
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK 38
#define LCD_CS 12
#define LCD_RESET 39
#define LCD_WIDTH 480
#define LCD_HEIGHT 480
#define LCD_BRIGHTNESS 200

// ------------------------------ Bus I2C interne ----------------------------
#define IIC_SDA 15
#define IIC_SCL 14

// Tactile CST9220
#define TP_INT 11
#define TP_RST 40

// PMU AXP2101 (batterie MX1.25)
#define AXP2101_ADDR 0x34

// ------------------------------ MicroSD (SPI) ------------------------------
#define SD_SCK 2
#define SD_MOSI 1
#define SD_MISO 3
#define SD_CS 41
#define SD_SPI_FREQ_HZ 10000000UL

// --------------------------- Capteurs externes -----------------------------
// Broches libres de la carte (à condition de renoncer à l'audio ES8311/ES7210).

// EGT : thermocouple K + MAX6675 (SPI logiciel).
// MAX6675 : conversion 220 ms max => lecture asynchrone toutes les 250 ms.
#define EGT_SCK 42
#define EGT_CS 16
#define EGT_MISO 45
#define EGT_READ_PERIOD_MS 250

// Régime moteur : impulsions de l'allumage MHR via optocoupleur H11L1
// (sortie collecteur ouvert, front descendant à chaque étincelle
// => 1 impulsion / tour sur un mono 2 temps).
#define RPM_PIN 10
#define RPM_DEBOUNCE_US 800
#define RPM_IDLE_SPEED 300
#define RPM_MAX_SPEED 16000

// GPS : module u-blox NEO (M8N recommandé à 20 Hz, NEO-6M accepté à 10 Hz).
#define GPS_RX_PIN 8   // TX du NEO -> ESP32
#define GPS_TX_PIN 9   // RX du NEO <- ESP32
#define GPS_BAUD_DEFAULT 9600L
#define GPS_BAUD_HIGH 115200L

// Bouton utilisateur Key3 (option : manual lap trigger)
#define USER_BTN_PIN 18

// ------------------------- Montage / orientation ---------------------------
// L'écran est monté pivoté de 90° vers la GAUCHE (boutons physiques à gauche,
// USB à droite, microSD à l'avant). LVGL tourne alors le rendu ET les
// coordonnées tactiles automatiquement (sw_rotate).
// Si après flash le sens s'avère inversé (test réel), mettre 0 pour revenir
// à l'orientation portrait.
#define UI_ROTATED_90CCW 1

// Calibration tactile brute (implémentation officielle Waveshare, révision
// actuelle de la carte) : ajuster si le toucher est inversé en X/Y.
#define TOUCH_SWAP_XY 1
#define TOUCH_MIRROR_X 1
#define TOUCH_MIRROR_Y 0

// ------------------------------ Alarme EGT ---------------------------------
// Clignotement du fond d'écran (orange) quand l'EGT dépasse ce seuil.
// Zone ROUGE de la jauge = 600 °C : l'alarme signifie "passage en dur danger".
#define EGT_ALARM_C 650.0f
#define EGT_ALARM_HYST_C 15.0f   // retour sous (seuil - hystérésis) pour stopper
#define EGT_ALARM_PERIOD_MS 250

// ------------------------------ Acquisition -------------------------------
#define ACQ_PERIOD_MS 50            // 20 Hz
#define IMU_ODR_HZ 250              // ODR QMI8658 (>= fréquence d'échantillonnage)
#define EGT_INTERP_POLY_ORDER 1
#define POWER_FILTER_ALPHA 0.20f    // passe-bas du 1er ordre (0..1)
#define SPEED_FILTER_ALPHA 0.35f
#define G_FILTER_ALPHA 0.45f

// --------------------------- Paramètres véhicule ---------------------------
// Estimation puissance / couple : P = (m*a + m*g*sin(pitch) + 0.5*rho*SCx*v^2)*v/eta
// Valeurs par défaut à ajuster dans la page Paramètres.
#define MASS_KG_DEFAULT 155.0f      // moto + pilote + inertie tournante ramenée
#define CXSA_DEFAULT 0.42f          // Cx * surface frontale (m^2)
#define ETA_DEFAULT 0.88f           // transmission (0.9 chaîne, ~0.85-0.90)
#define CRR_DEFAULT 0.015f          // résistance au roulement
#define AIR_DENSITY 1.184f          // kg/m^3 (25 C, ~0 m)

// ------------------------------ Chrono / tours -----------------------------
#define LAP_RADIUS_DEFAULT_M 15.0f  // rayon de la ligne départ/arrivée
#define LAP_REARM_FACTOR 1.6f       // doit sortir du rayon pour ré-armer
#define LAP_MIN_SPEED_MPS 3.0f      // ignore les validations à l'arrêt

// --------------------------------- BLE ------------------------------------
#define BLE_DEVICE_NAME "Serge-Dash"
// Service GATT custom. Cadence : notify 20 Hz (~64 octets) ~ 1.3 kB/s.
#define BLE_SERVICE_UUID "a5f1d800-5b3a-4b7c-9c1e-2f6d4a8e0100"
#define BLE_CHAR_LIVE_UUID "a5f1d801-5b3a-4b7c-9c1e-2f6d4a8e0100"   // notify <LIVE frame>
#define BLE_CHAR_CTRL_UUID "a5f1d802-5b3a-4b7c-9c1e-2f6d4a8e0100"   // write  (commandes)
#define BLE_CHAR_FILE_UUID "a5f1d803-5b3a-4b7c-9c1e-2f6d4a8e0100"   // notify (transferts)
#define BLE_FILE_CHUNK 232          // octets de données par notification
#define BLE_FILE_ACK_EVERY 16       // ACK client requis tous les N paquets
