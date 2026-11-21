# Protocole BLE — Serge Dashboard ↔ Pixel 5

## Découverte

- Nom du périphérique : **Serge-Dash**
- Service : `a5f1d800-5b3a-4b7c-9c1e-2f6d4a8e0100`

| Caractéristique | UUID | Mode |
|---|---|---|
| LIVE | `...d801` | Notify (20 Hz) |
| CTRL | `...d802` | Write (commandes texte) |
| FILE | `...d803` | Notify (listes + transferts de sessions) |

## 1. Trame LIVE (notify, 20 Hz, 70 octets, little-endian)

| Décalage | Taille | Champ |
|---|---|---|
| 0 | 2 | magic `'S'`,`'G'` |
| 2 | u16 | régime (tr/min) |
| 4 | f32 | vitesse km/h |
| 8 | f32 | EGT °C (-1 = pas de sonde) |
| 12 | f32 | EGT max °C |
| 16 | f32 | G longitudinale (g) |
| 20 | f32 | G latérale (g) |
| 24 | f32 | inclinaison ° |
| 28 | f32 | puissance kW (estimée) |
| 32 | f32 | couple Nm (estimé) |
| 36 | u32 | ms depuis début session |
| 40 | i32 | GPS lat × 1e7 (0 si pas de fix) |
| 44 | i32 | GPS lon × 1e7 |
| 48 | f32 | altitude m |
| 52 | u8 | satellites |
| 53 | u8 | fix (0/1) |
| 54 | u8 | batterie % (255 = absent) |
| 55 | u8 | enregistrement (0/1) |
| 56 | u32 | tour courant ms |
| 60 | u32 | meilleur tour ms (0 = aucun) |
| 64 | u32 | tour précédent ms (0) |
| 68 | u16 | n° de tour |

## 2. Commandes (écriture sur CTRL)

Commandes texte ASCII :

| Commande | Effet |
|---|---|
| `REC` | démarre l'enregistrement microSD |
| `STOP` | arrête l'enregistrement |
| `LAP` | déclenche un passage de tour (trigger manuel) |
| `LIST` | renvoie la liste des sessions sur FILE |
| `GET <nom>` | démarre le transfert du fichier (FILE) |

## 3. Transfert de fichier (FILE, notify)

- `LIST` → notifications `F <nom> <taille>\n` (lots), puis `FLIST_END\n`
- `GET <nom>` → :
  1. une notification `FHDR <taille> <nom>`
  2. des paquets `DAT` + 4 octets séquence + 232 octets de données
  3. une notification finale `FEND`

Débit contraint (pause de 120 ms tous les 16 paquets) : ~15 Ko/s utiles.
Un fichier de 20 min ≈ 2 Mo → ≈ 2 à 3 min de transfert.

## 4. Exemple Python (pixel 5 / PC) avec bleak

```python
import asyncio, struct
from bleak import BleakClient

SERVICE = "a5f1d800-5b3a-4b7c-9c1e-2f6d4a8e0100"
LIVE = "a5f1d801-5b3a-4b7c-9c1e-2f6d4a8e0100"
CTRL = "a5f1d802-5b3a-4b7c-9c1e-2f6d4a8e0100"

def parse_live(data):
    assert data[:2] == b"SG"
    rpm = struct.unpack_from("<H", data, 2)[0]
    speed = struct.unpack_from("<f", data, 4)[0]
    egt = struct.unpack_from("<f", data, 8)[0]
    g_long = struct.unpack_from("<f", data, 16)[0]
    g_lat = struct.unpack_from("<f", data, 20)[0]
    return rpm, speed, egt, g_long, g_lat

async def main():
    async with BleakClient("Serge-Dash") as cl:
        def on_live(_, data):
            rpm, speed, egt, glong, glat = parse_live(data)
            print(f"{speed:6.1f} km/h  RPM {rpm:5d}  EGT {egt:5.0f} °C  G {glong:+.2f}/{glat:+.2f}")
        await cl.start_notify(LIVE, on_live)
        await asyncio.sleep(30)   # live pendant 30 s

asyncio.run(main())
```

## 5. Format CSV des sessions (microSD)

```
t_ms,speed_kmh,rpm,egt_c,egt_max_c,g_long,g_lat,bank_deg,pitch_deg,
power_kw,torque_nm,lat_e7,lon_e7,altitude_m,heading_deg,sats,fix,vbatt_v
```

20 Hz → ≈ 2 Mo pour 20 minutes de roulage.
