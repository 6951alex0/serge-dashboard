# Câblage des périphériques externes

Tous les organes internes de la carte (écran, tactile, IMU QMI8658, PMU AXP2101,
microSD) sont déjà raccordés en interne — rien à souder dessus.

Les périphériques externes se branchent sur les **broches audio/désaffectées**
de cette carte, car l'audio ES8311/ES7210 n'est pas utilisé ici.

## 1. Régime moteur — allumage MHR (recommandé)

Une impulsion par tour (mono 2 temps) est extraite du harnais d'allumage MHR.
Le primaire peut monter à plusieurs centaines de volts : **protection
obligatoire**.

```
Fil primaire allumage ──[ 100 kΩ ]──●──[ H11L1 : entrée opto ]──── GND moteur
                                    │
                                     (masse moteur seulement côté allumage)
H11L1 sortie (collecteur ouvert) :
    VCC 3V3 carte ──[ 10 kΩ pull-up ]──●── GPIO10 (ESP32-S3)
                                       │
                                     H11L1.OUT
    GND carte ── GND H11L1
```

- Le H11L1 (comparateur Schmitt intégré) sort des fronts propres.
- Front descendant à chaque étincelle → 1 front/tour.
- Optionnellement : zener 5,1 V + condensateur 10 nF en plus côté opto si le MHR est bruyant.
- Logiciel : interruption FALLING + timer 32 bits (`src/sensors/rpm.cpp`).

Alternative acceptée : capteur Hall + aimant sur rotor (même code).

## 2. EGT — thermocouple K + MAX6675 (SPI logiciel)

| Signal MAX6675 | GPIO carte |
|---|---|
| SCK | **GPIO42** |
| CS  | **GPIO16** |
| SO  (MISO) | **GPIO45** |
| VCC | 3V3 |
| GND | GND |

- Thermocouple K : point le plus chaud du pot (≈ 5 cm de la sortie), fil jaune = +.
- Conversion 220 ms maximum → l'acquisition lit toutes les 250 ms et interpole.
- Si le thermocouple est débranché, bit D2 du registre = 1 : la valeur
  précédente est conservée (plafonnée à la dernière valeur valide).

## 3. GPS — module u-blox NEO (UART)

| Signal NEO | GPIO ESP32-S3 |
|---|---|
| TX | **GPIO8** (GPS_RX_PIN) |
| RX | **GPIO9** (GPS_TX_PIN) |
| VCC | 3V3 |
| GND | GND |

- Au boot, l'ESP32 envoie la trame UBX-CFG-RATE pour passer le module à 20 Hz.
- Un NEO-6M n'accepte pas 20 Hz réels : il reste fonctionnel à sa cadence (5–10 Hz),
  le chrono GPS diminue pas ...
- NEO-M8N/M9N/M10 recommandés pour 20 Hz.

Attention : si ta carte expose bien les broches audio 8/9, elles sont ici
**réaffectées** (plus d'audio). Le code n'inclut pas ES8311/ES7210.

## 4. Bouton utilisateur (optionnel)

- **GPIO18** (Key3 de la carte) : libre, tiré au 3V3, à la masse au pressé.
  Réservé à un futur trigger manuel de tour au guidon.

## 5. Alimentation

- Batterie Li-ion 3,7 V sur connecteur **MX1.25 2 broches** (PMU AXP2101 ;
  charge/décharge gérées, niveau de batterie lu dans le dashboard).
- Ne jamais alimenter l'ESP32 directement en dessous de 3,3 V pour le
  thermocouple : le MAX6675 a besoin de 3,0 V min pour fonctionner correctement.
