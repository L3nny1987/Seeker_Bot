# Seeker_Bot – Verkabelung

## Komponenten

- ESP32 (esp32dev / WROOM-32)
- L298N Dual-H-Bridge Motortreiber
- 2× DC-Motor (Panzerketten-Antrieb)
- FHL-LD20 LiDAR
- Motor-Netzteil
- 5-V-Versorgung für LiDAR/ESP32

---

## Kinematik

Der Roboter hat **keinen** klassischen Differentialantrieb:

| Motor | L298N-Kanal | Funktion |
|---|---|---|
| Motor A | ENA / IN1 / IN2 / OUT1+OUT2 | **Fahrt** vorwärts / rückwärts |
| Motor B | ENB / IN3 / IN4 / OUT3+OUT4 | **Drehung** links / rechts |

Beide Achsen sind unabhängig und können gleichzeitig angesteuert werden
(Kurvenfahrt = Fahrt ≠ 0 **und** Drehung ≠ 0).

Bestätigte Richtungen anhand des funktionierenden Motortests:
- Vorwärts: IN1 HIGH, IN2 LOW; rückwärts umgekehrt.
- Links: IN3 HIGH, IN4 LOW; rechts umgekehrt.
- API: Fahrt positiv = vorwärts, Drehung negativ = links.
    Deshalb ist `SEEKER_TURN_INVERTED = 1` gesetzt.
- Die Firmware regelt ENA/ENB weiterhin per PWM; bei 100 % dauerhaft HIGH.
    Stopps und Not-Aus schalten beide PWM-Ausgänge ab.

---

## 1. ESP32 → L298N

| ESP32 | L298N | Funktion |
|---|---|---|
| GPIO 25 | ENA | Geschwindigkeit Fahrmotor (PWM) |
| GPIO 26 | IN1 | Richtung Fahrmotor |
| GPIO 27 | IN2 | Richtung Fahrmotor |
| GPIO 32 | IN3 | Richtung Drehmotor |
| GPIO 33 | IN4 | Richtung Drehmotor |
| GPIO 14 | ENB | Geschwindigkeit Drehmotor (PWM) |
| GND | GND | Gemeinsame Masse |

### ENA / ENB

Für Geschwindigkeitsregelung über den ESP32:

- Jumper auf **ENA entfernen**
- Jumper auf **ENB entfernen**
- ENA → GPIO 25
- ENB → GPIO 14

---

## 2. Motoren → L298N

- Fahrmotor: Kabel 1 → **OUT1**, Kabel 2 → **OUT2**
- Drehmotor: Kabel 1 → **OUT3**, Kabel 2 → **OUT4**

Falls ein Motor falsch herum läuft: Kabel tauschen **oder**
`SEEKER_DRIVE_INVERTED` / `SEEKER_TURN_INVERTED` in
`components/seeker_config/include/seeker_pins.h` auf `1` setzen.

---

## 3. Motorversorgung → L298N

- Netzteil PLUS → **L298N VIN / +12V**
- Netzteil MINUS → **L298N GND**

Die DC-Motoren werden **nicht über den ESP32** versorgt.

---

## 4. FHL-LD20 LiDAR

| LiDAR-Kabel | Anschluss |
|---|---|
| Schwarz | +5 V |
| Grün | GND |
| Weiß | GPIO 18, **S**-Reihe – Datenleitung (LiDAR TX → ESP32 RX) |
| Rot | **OFFEN LASSEN** – PWM-Eingang |

### Korrektur anhand der Herstellerunterlagen

- Quelle: https://wiki.youyeetoo.com/en/Lidar/LD20, Abschnitt 3.1.1.
- Die abgebildete Variante: Schwarz = +5 V, Grün = GND, Rot = PWM-Eingang,
  Weiß = Datenleitung zum Controller-RX.
- Der Sensor sendet nach stabilem Start selbstständig mit 230400 Baud, 8N1.
- Umbauten nur stromlos. Am Expansion-Board D18/**S**, nicht D18/V oder G.

Rot bleibt offen; Weiß wird mit dem RX des ESP32 verbunden.

Einbaukalibrierung: Rohwinkel 180° zeigt nach vorne
(`SEEKER_LIDAR_FRONT_DEG = 180`, Winkel wachsen im Uhrzeigersinn).
Radar und Hinderniserkennung verwenden dieselbe Ausrichtung.

### Offener Punkt

Die bisherige Annahme Rot = UART war falsch für die Herstellerabbildung.
Der Hersteller warnt vor abweichenden Kabelchargen. Bei einem anderen Kabelsatz
nach Stecker-Pinbelegung statt allein nach Farbe anschließen:
1 = PWM/RX (Eingang), 2 = GND, 3 = TX (Ausgang), 4 = VCC (5 V).
Steckerorientierung anhand der Herstellerzeichnung prüfen.

---

## 5. Gemeinsame Masse

ESP32 GND
↳ L298N GND
↳ LiDAR GND (grün)
↳ Motor-Netzteil MINUS

---

## 6. Komplette GPIO-Belegung

| GPIO | Verwendung |
|---|---|
| GPIO 14 | L298N ENB (Drehmotor PWM) |
| GPIO 18 | LiDAR Weiß (UART2 RX) |
| GPIO 25 | L298N ENA (Fahrmotor PWM) |
| GPIO 26 | L298N IN1 |
| GPIO 27 | L298N IN2 |
| GPIO 32 | L298N IN3 |
| GPIO 33 | L298N IN4 |

Freie Reserve-Pins für Erweiterungen: GPIO 4, 5, 13, 16, 17, 19, 21, 22, 23.
Nicht verwenden: GPIO 6–11 (Flash), GPIO 34–39 (nur Eingang).
