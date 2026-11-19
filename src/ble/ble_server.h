#pragma once
// Serveur BLE : trame LIVE 20 Hz + commandes + transfert de sessions.
// Protocole detaille dans docs/protocole-ble.md (cote Pixel 5).

void ble_start();
bool ble_connected();
