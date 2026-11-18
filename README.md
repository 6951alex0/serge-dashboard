# Serge Dashboard

Tableau de bord de télémétrie **autonome** pour prototype 2 temps.
Carte : **Waveshare ESP32-S3-Touch-AMOLED-2.16** (ESP32-S3R8, AMOLED rond 480x480
tactile, IMU 6 axes QMI8658, PMU AXP2101, slot microSD, BLE 5).

Le Pixel 5 est un **complément**, pas une dépendance : l'appareil
mesure, calcule, affiche et enregistre seule. Le téléphone ne fait que
visualiser le live et récupérer les sessions.

## Fonctions

| Fonction | Détail |
|---|---|
| EGT | Thermocouple K + MAX6675, jauge arc à 3 zones (vert / orange / rouge), 0–700 °C |
| Vitesse | GPS, affichage numérique très lisible |
| Chrono | Ligne départ/arrivée GPS (configurable) + bouton de secours ; tour courant, meilleur, précédent |
| EGT max | Mémorisé toute la session |
| G-Force | Longitudinale (dérivée GPS filtrée) + latérale (IMU), échelle 2 g |
| Inclinaison | Angle gauche/droite barre -60° :+60°, issue de la gravité (IMU) |
| Puissance / couple | Estimation : force d'inertie + pente + aéro + roulement, filtre bas-pass |
| Régime | Reprise allumage MHR via H11L1 (1 impulsion/tour) |
| Enregistrement | CSV 20 Hz sur microSD, horodaté GPS |
| Pixel 5 | BLE live 20 Hz + export de sessions (voir `docs/protocole-ble.md`) |

## Pages écran (5)

1. **COURSE** — jauge EGT, vitesse, 3 lignes de chrono, indicateur REC + GPS/BAT
2. **TÉLÉMÉTRIE** — cercle G-Force (lat/long), inclinaison
3. **MOTEUR** — régime, EGT max, puissance estimée, couple estimé
4. **PARAMÈTRES** — masse, Cx·S, rendement transmission, rayon ligne, bouton *LIGNE ICI* (prend la position GPS courante) et calibrage IMU
5. **SESSIONS** — enregistrement start/stop, listing microSD, export via BLE

Navigation : swipe gauche/droite sur n'importe quelle page.

## Acquisition

- Loops acquisition : 20 Hz (50 ms), imu QMI8658 à 250/112 Hz interne
- EGT : lecture MAX6675 toutes les 250 ms (limite matérielle de conversion), interpolée linéairement => valeur "lisse" à 20 Hz
- RPM : front par étincelle ; RPM = 60 / période (timer 32 bits + interruption)

## Compilation

```bash
pip install platformio
pio run -e serge216          # compile
pio run -e serge216 -t upload # flash via USB
pio run -e serge216 -t monitor
```

Le fichier `include/lv_conf.h` est inclus au projet (LVGL 8.4, buffer dans la
RAM interne, PSRAM activée OPI — flash 16 Mo, QSPI écran).

## Câblage externe

Voir `docs/cablage.md` : interface allumage MHR (H11L1), EGT MAX6675, module
NEO GPS, et les broches de la carte récupérées (audio non utilisé).

## Structure

```
include/   config.h (toute la config), data_model.h, lv_conf.h
src/
  main.cpp          setup + allocation des tâches
  data_model.cpp    état partagé (télémétrie, chrono, GPS, settings NVS)
  sensors/          rpm (H11L1), egt (MAX6675), imu (QMI8658), gps (NEO NMEA+UBX)
  compute/          acq.cpp (boucle 20 Hz, G, puissance), lap_timer.cpp (chrono GPS)
  storage/          sd_logger.cpp (CSV, listing, chunks pour BLE)
  ble/              ble_server.cpp (GATT : LIVE 20 Hz, CTRL, FILE)
  drivers/          screen.cpp (CO5300 + CST9220), pmu.cpp (AXP2101)
  ui/               ui.cpp : 5 pages LVGL
```

## Et après

- Café, essai au stand, calibrage réaliste de masse / Cx·S / rendement.
- Puis réglage du seuil EGT max (alarme si dépassement).
