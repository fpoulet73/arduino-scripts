# pompe.ino — Contrôle de pompe Hayward via ESP8266

Script Arduino pour piloter une **pompe de piscine Hayward** (3 vitesses) à distance via une API HTTP, grâce à un module **ESP8266** (NodeMCU / Wemos D1).

## Matériel requis

| Composant | Détail |
|-----------|--------|
| Microcontrôleur | ESP8266 (NodeMCU / Wemos D1 Mini) |
| Relais | 4 canaux (actif LOW) |
| Pompe | Hayward 3 vitesses |

## Câblage des relais

| Broche ESP8266 | Relais | Fonction |
|----------------|--------|----------|
| D4 | RELAIS_ONOFF | Alimentation pompe (ON/OFF) |
| D1 | RELAIS_V1 | Vitesse 1 (lente) |
| D2 | RELAIS_V2 | Vitesse 2 (moyenne) |
| D3 | RELAIS_V3 | Vitesse 3 (rapide) |

> Les relais sont actifs à l'état **LOW** — toutes les sorties sont mises à HIGH au démarrage par sécurité.

## Configuration WiFi

Modifier les constantes en haut du fichier :

```cpp
const char* ssid     = "TON_SSID";
const char* password = "TON_MOT_DE_PASSE";
```

Si la connexion échoue après 15 secondes, l'ESP8266 redémarre automatiquement.

## API HTTP

Une fois connecté, l'ESP expose un serveur web sur le port **80**. L'adresse IP est affichée dans le moniteur série.

| Méthode | Endpoint | Description |
|---------|----------|-------------|
| GET | `/` | Vérification — retourne `Pompe Hayward API OK` |
| GET | `/pompe/on` | Démarre la pompe en vitesse 1 |
| GET | `/pompe/off` | Arrête complètement la pompe |
| GET | `/pompe/vitesse?v=1\|2\|3` | Sélectionne une vitesse (1, 2 ou 3) |
| GET | `/pompe/status` | Retourne l'état JSON de la pompe |

### Exemple de réponse `/pompe/status`

```json
// Pompe allumée en vitesse 2
{ "etat": "on", "vitesse": 2 }

// Pompe éteinte
{ "etat": "off" }
```

## Logique de sécurité

- Lors d'un **changement de vitesse**, tous les relais de vitesse sont coupés avant d'activer le nouveau, avec un délai de 300 ms.
- Lors de l'**arrêt**, les vitesses sont coupées avant de couper l'alimentation (délai 200 ms).

## Dépendances Arduino

- `ESP8266WiFi.h` (inclus dans le board package ESP8266)
- `ESP8266WebServer.h` (inclus dans le board package ESP8266)

**Board manager URL :** `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
