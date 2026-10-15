# Seeker Bot – Verkabelung

Die verbindliche Firmwarebelegung steht in `components/seeker_config/include/seeker_pins.h`. Diese Anleitung beschreibt den Aufbau mit Fahrt- und Drehachse.

## Motorachsen am L298N

| Funktion | ESP32 | L298N |
|---|---:|---|
| Fahrt-PWM | GPIO 25 | ENA |
| Fahrt-Richtung | GPIO 26 / 27 | IN1 / IN2 |
| Dreh-PWM | GPIO 14 | ENB |
| Dreh-Richtung | GPIO 32 / 33 | IN3 / IN4 |
| Bezugspotential | GND | GND |

ENA- und ENB-Jumper für die PWM-Steuerung entfernen. Fahrmotor an OUT1/OUT2, Drehmotor an OUT3/OUT4 anschließen. Dieses Chassis besitzt keine direkt angesteuerten linken und rechten Kettenmotoren.

| Befehl | Richtungssignale |
|---|---|
| Vorwärts | IN1 HIGH, IN2 LOW |
| Rückwärts | IN1 LOW, IN2 HIGH |
| Links | IN3 HIGH, IN4 LOW |
| Rechts | IN3 LOW, IN4 HIGH |

API: positive Fahrt ist vorwärts; negative Drehung ist links. Aktuell sind `SEEKER_DRIVE_INVERTED = 0` und `SEEKER_TURN_INVERTED = 1` gesetzt. Bei einem anderen Aufbau angehoben prüfen.

## LD20 LiDAR

| Anschluss | ESP32 / Versorgung |
|---|---|
| LiDAR TX | GPIO 18, UART2 RX |
| VCC | Geeignete stabile 5-V-Versorgung |
| GND | Gemeinsame Masse |
| PWM/RX-Eingang | Im aktuellen Aufbau offen |

Der vorhandene Kabelsatz verwendet Schwarz für 5 V, Grün für GND, Weiß für TX und Rot für PWM. Chargen können abweichen. Nach Steckerbelegung und [Herstellerzeichnung](https://wiki.youyeetoo.com/en/Lidar/LD20) prüfen: 1 PWM/RX, 2 GND, 3 TX, 4 VCC. Die Steckerorientierung beachten.

Nur stromlos umverdrahten. An einem Expansion-Board gehört die Datenleitung an D18/Signal. UART: 230400 Baud, 8N1, 3,3-V-TTL-Datenpegel. Der ESP32 empfängt; seine UART-TX-Leitung bleibt unbelegt.

Standardausrichtung: Rohwinkel 180° zeigt nach vorne, Winkel wachsen im Uhrzeigersinn. Ein Hindernis vor dem Chassis muss im Radar ebenfalls vorne erscheinen.

## MPU6050 / GY-521

| Modulanschluss | ESP32 |
|---|---|
| VCC | 3V3, Eignung des konkreten Moduls prüfen |
| GND | GND |
| SDA / SCL | GPIO 21 / 22 |
| AD0 | GND/offen für 0x68; 0x69 wird ebenfalls erkannt |
| INT | Nicht verwendet |

I²C arbeitet mit 400 kHz. Die Firmware liest den FIFO; ein Interruptkabel ist nicht erforderlich. Nach dem Einschalten ungefähr zwei Sekunden ruhig halten. Der Sensor muss starr befestigt sein. Die Hochachse wird aus der Schwerkraft bestimmt, sodass eine feste abweichende Einbaulage berücksichtigt werden kann.

## Versorgung und Masse

Motorversorgung PLUS an den Versorgungseingang des L298N, MINUS an GND. Die Motorspannung muss zu Motoren und Modul passen; eine Beschriftung „12V“ allein bestimmt sie nicht. Motoren werden nicht aus einem ESP32-GPIO oder dessen 3V3-Pin versorgt.

Logik und LiDAR benötigen geeignete geregelte Versorgung. Die Nutzung eines L298N-Reglerjumpers hängt vom Modul ab. Gemeinsame Masse verbindet ESP32, L298N, LiDAR, IMU und den Bezug der Motorversorgung. Eine zugängliche physische Abschaltung der Motorversorgung unterstützt die Inbetriebnahme.

## Geometrie und erste Prüfung

Aktueller Körper relativ zum LiDAR: 100 mm nach vorne, 200 mm nach hinten, 70 mm zu jeder Seite. Standard-Fahrkorridor: 200 mm. Diese Werte vor autonomen Fahrten am tatsächlichen Chassis prüfen.

GPIO 21/22 sind durch die IMU belegt. Weitere Anschlüsse unter Berücksichtigung von Flash-Pins, reinen Eingängen, Boot-Strapping und der konkreten ESP32-Platine auswählen.
