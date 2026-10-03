# Communication Protocol — Foxibot
## Raspberry Pi 5 ↔ STM32 Motor Controller

## Objectif

Ce document spécifie le contrat de communication entre le Raspberry Pi 5 et le STM32 du prototype actuel de Foxibot.

Le protocole reste indépendant de ROS, de la cinématique et du comportement haut niveau.

---

# I. Décisions d’architecture

## 1. Framing

Format retenu :

```text
[START][TYPE][VERSION][LEN][PAYLOAD][CRC]
```

Le récepteur cherche `START`, lit l’en-tête, vérifie que `LEN` est plausible, lit le payload et le CRC, puis valide la trame.

Le byte `START` peut apparaître dans le payload. La resynchronisation repose donc sur :

```text
START détecté
   ↓
LEN plausible
   ↓
CRC valide
```

Si la trame candidate est invalide, elle est rejetée et le décodeur recommence à chercher `START`.

---

## 2. Structure de trame

### START
Début d’une trame candidate.

### TYPE
Identifie le type de message.

Messages V1 :
- `SERVO_COMMAND`
- `ARM_STATE`
- `HEARTBEAT`
- `STATUS`

### VERSION
Version du `TYPE` concerné.

Exemples :

```text
SERVO_COMMAND V1
SERVO_COMMAND V2
STATUS V1
STATUS V2
```

### LEN
Longueur du payload en octets.

### PAYLOAD
Données propres à `TYPE + VERSION`.

### CRC
CRC 16 bits calculé sur :

```text
TYPE + VERSION + LEN + PAYLOAD
```

`START` et le champ CRC lui-même sont exclus du calcul.

---

## 3. Représentation des données

### Endianness

Tous les champs multi-octets sont transmis en **little-endian**.

Exemple :

```text
0x1234 → 0x34 0x12
```

### Angle servo

```text
Type  : int16_t
Échelle : 0.01°
Plage : -327.68° à +327.67°
```

Exemples :

```text
100    → 1.00°
12345  → 123.45°
-9000  → -90.00°
```

---

## 4. CRC

CRC retenu :

```text
CRC-16/CCITT-FALSE
```

Paramètres :

```text
Width  : 16 bits
Poly   : 0x1021
Init   : 0xFFFF
RefIn  : false
RefOut : false
Check  : 0x29B1
```

Polynôme :

```text
x^16 + x^12 + x^5 + 1
```

Vecteur de test :

```text
"123456789" → 0x29B1
```

---

## 5. Messages V1

### SERVO_COMMAND V1

Direction :

```text
Raspberry Pi → STM32
```

Période :

```text
20 ms = 50 Hz
```

Payload :

```text
[ID_SERVO][SENS][ANGLE]
```

Avec :

```text
ID_SERVO : identifiant du servo
SENS     : 0 = positif, 1 = négatif
ANGLE    : int16_t, échelle 0.01°, little-endian
```

---

### ARM_STATE V1

Direction :

```text
Raspberry Pi → STM32
```

Payload :

```text
arm_state : uint8_t
```

Valeurs :

```text
0 = DISARMED
1 = ARMED
```

Le STM32 démarre en `DISARMED`.

Les trames peuvent être reçues et décodées en état `DISARMED`, mais elles ne doivent pas provoquer de mouvement tant qu’un armement explicite n’a pas été reçu.

---

### HEARTBEAT V1

Direction :

```text
Raspberry Pi ↔ STM32
```

Payload vide :

```text
LEN = 0
```

Période :

```text
200 ms = 5 Hz
```

Le heartbeat signifie uniquement :

```text
"je suis vivant"
```

---

### STATUS V1

Direction :

```text
STM32 → Raspberry Pi
```

Période nominale :

```text
500 ms = 2 Hz
```

Un `STATUS` est aussi envoyé immédiatement après un changement d’état important ou une erreur protocole.

Payload :

```text
ARM_STATE            : uint8_t
WATCHDOG_STATE       : uint8_t
CRC_ERROR_COUNT      : uint16_t
PROTOCOL_ERROR_COUNT : uint8_t
PROTOCOL_ERROR_CODE  : uint8_t
PROTOCOL_ERROR_VALUE : uint8_t
```

Taille actuelle :

```text
7 octets
```

`PROTOCOL_ERROR_COUNT` est saturant :

```text
0 → ... → 255 → 255
```

Il ne reboucle pas à 0.

Erreurs prévues :

```text
UNKNOWN_TYPE
UNKNOWN_VERSION
```

`PROTOCOL_ERROR_VALUE` contient la valeur reçue ayant provoqué l’erreur.

Exemples :

```text
UNKNOWN_TYPE    + TYPE reçu
UNKNOWN_VERSION + VERSION reçue
```

---

### Exclus de V1

```text
ACK / NACK
messages capteurs
champs brushless futurs
placeholders inutilisés
```

---

## 6. Timing, watchdog et erreurs

### Fréquences

```text
SERVO_COMMAND : 20 ms  = 50 Hz
HEARTBEAT     : 200 ms = 5 Hz
STATUS        : 500 ms = 2 Hz
WATCHDOG      : 600 ms
```

### Perte du heartbeat Pi côté STM32

Après `600 ms` sans heartbeat Pi valide :

```text
timeout heartbeat
      ↓
WATCHDOG_FAULT
      ↓
DEFAULT_POSE locale
```

La pose par défaut doit être stockée localement dans le STM32.

Pas de reprise automatique.

---

### Perte du heartbeat STM32 côté Pi

Après `600 ms` sans heartbeat STM32 valide :

```text
MCU OFFLINE affiché dans l’UI
      ↓
arrêt des commandes normales
      ↓
5 commandes DEFAULT_POSE
espacées de 20 ms
      ↓
défaut latched
```

La reprise est volontaire.

L’utilisateur doit soit :
- éteindre/rallumer Foxibot ;
- relancer le launch/script côté Pi.

Au redémarrage :

```text
communication relancée
      ↓
DISARMED
      ↓
heartbeat valide
      ↓
armement explicite
      ↓
reprise des mouvements
```

---

### CRC invalide

```text
trame rejetée
CRC_ERROR_COUNT++
heartbeat watchdog non réarmé
```

Le nombre d’erreurs CRC est purement diagnostique.

Il n’existe pas de seuil CRC provoquant directement l’arrêt du robot.

---

### LEN invalide

```text
trame rejetée
resynchronisation
retour SEARCH_START
```

---

### TYPE inconnu

```text
trame rejetée
PROTOCOL_ERROR_COUNT++
PROTOCOL_ERROR_CODE  = UNKNOWN_TYPE
PROTOCOL_ERROR_VALUE = TYPE reçu
STATUS immédiat
```

---

### VERSION inconnue

```text
trame rejetée
PROTOCOL_ERROR_COUNT++
PROTOCOL_ERROR_CODE  = UNKNOWN_VERSION
PROTOCOL_ERROR_VALUE = VERSION reçue
STATUS immédiat
```

Le payload ne doit pas être interprété avec une autre version.

---

## 7. Sérialisation

Choix retenu :

```text
sérialisation champ par champ
```

Pas de struct packée envoyée directement avec `memcpy`.

Exemple conceptuel :

```text
write(TYPE)
write(VERSION)
write(LEN)
write(PAYLOAD fields)
write(CRC)
```

Même principe côté décodage.

Les champs multi-octets sont écrits et lus explicitement en little-endian.

---

## 8. Décodeur

Choix retenu :

```text
machine à états incrémentale
```

Exemple :

```text
SEARCH_START
    ↓
READ_HEADER
    ↓
READ_PAYLOAD
    ↓
READ_CRC
    ↓
VALIDATE
    ↓
DISPATCH
```

En cas d’erreur :

```text
RESYNC → SEARCH_START
```

Cette FSM UART est distincte de la machine à états globale du STM32 (`DISARMED`, `ARMED`, `WATCHDOG_FAULT`, etc.).

---

# II. Résumé rapide

| # | Décision | Choix |
|---|---|---|
| 1 | Framing | Start byte + length |
| 2 | Structure | `[START][TYPE][VERSION][LEN][PAYLOAD][CRC]` |
| 3 | Représentation | Little-endian, angle `int16_t` à 0.01° |
| 4 | CRC | CRC-16/CCITT-FALSE |
| 5 | Messages V1 | SERVO_COMMAND, ARM_STATE, HEARTBEAT, STATUS |
| 6 | Timing / sécurité | 50 Hz / 5 Hz / 2 Hz, watchdog 600 ms |
| 7 | Sérialisation | Champ par champ |
| 8 | Décodeur | FSM incrémentale |

---

# III. Tests minimum

- CRC `"123456789" → 0x29B1`
- encode → decode
- little-endian
- trame valide
- CRC invalide
- trame tronquée
- LEN invalide
- TYPE inconnu
- VERSION inconnue
- resynchronisation
- watchdog
- saturation du compteur protocole
- STATUS immédiat après erreur

# IV. Constantes numériques encore à fixer dans le code

Les décisions d’architecture sont closes. Il reste seulement à attribuer quelques valeurs numériques d’implémentation :

- valeur du byte `START`
- identifiants numériques des `TYPE`
- identifiants numériques des `PROTOCOL_ERROR_CODE`
- encodage/taille concret de `ID_SERVO`