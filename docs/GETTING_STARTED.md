# Seeker Bot einrichten

**Autor:** Lennard Schimmer · **Stand:** Firmware 0.4.0

## 1. Voraussetzungen

- ESP32 für die Zielkonfiguration `esp32dev`, 4 MB Flash.
- LD20, L298N und ein Chassis mit der dokumentierten Fahrt-/Drehachsen-Zuordnung.
- MPU6050/GY-521 empfohlen; separate geeignete Motorversorgung und gemeinsame Masse.
- PlatformIO Core oder PlatformIO-Erweiterung in VS Code; USB-Treiber passend zum ESP32-Board.
- Ein lokales 2,4-GHz-WLAN, dessen Zugangsdaten im lokalen Header hinterlegt werden.

Die [Verkabelung](WIRING.md) ist vor dem Einschalten zu prüfen. Der ESP32 versorgt die Motoren nicht.

## 2. WLAN-Konfiguration anlegen

In der Repository-Wurzel:

```powershell
Copy-Item src/credentials.example.h src/credentials.h
```

Linux/macOS:

```sh
cp src/credentials.example.h src/credentials.h
```

In der lokalen Datei SSID und Kennwort eintragen. Die Makronamen der Vorlage beibehalten. Sonderzeichen müssen gültige C-Zeichenketten ergeben. Diese Datei und daraus gebaute Firmware bleiben privat; `.gitignore` nimmt sie aus dem üblichen Git-Ablauf heraus.

## 3. Firmware bauen

```sh
pio run -e seeker_bot
```

Standardziel ist `seeker_bot`. Die Plattform ist `espressif32@7.1.3`; `dependencies.lock` dokumentiert die aktuelle ESP-IDF-Auflösung. `seeker_bot_debug` bietet einen Debugbau.

Der Build erzeugt das komprimierte Dashboard automatisch. Das Post-Build-Werkzeug kopiert das Anwendungsimage zusätzlich nach `ota/seeker_bot_latest.bin` und in einen versionierten Dateinamen. Der Ordner bleibt lokal.

Wenn `pio` nicht verfügbar ist, die PlatformIO-Terminalfunktion öffnen oder den eigenen PlatformIO-Installationspfad konfigurieren. Das Repository enthält keine fest eingetragenen Benutzerpfade.

## 4. Erstinstallation per USB

```sh
pio device list
pio run -e seeker_bot -t upload --upload-port PORT
pio device monitor --port PORT --baud 115200
```

`PORT` durch den tatsächlich erkannten Port ersetzen. Bei nur einem passenden Gerät kann PlatformIO den Uploadport auch automatisch ermitteln. Der erste USB-Upload installiert die zum Projekt gehörende Partitionierung. Ein OTA-Anwendungsimage allein ersetzt diesen Schritt bei einer abweichenden alten Partitionstabelle nicht.

## 5. Im Dashboard in Betrieb nehmen

1. Motoren zunächst ohne Bodenkontakt prüfen. Nach dem Einschalten etwa zwei Sekunden für die IMU-Kalibrierung ruhig halten.
2. Im seriellen Log die zugewiesene IP ablesen und `http://ROBOTER_IP` öffnen.
3. Steuerung übernehmen und im manuellen Modus kurze, einzelne Achsenbefehle prüfen.
4. Vorwärts/rückwärts und links/rechts mit der realen Bewegung vergleichen. Erst danach den Aufbau freigeben.
5. Ein Hindernis vor dem Roboter mit dem Radar abgleichen. Standardausrichtung: LiDAR-Rohwinkel 180° ist vorne.
6. Die erste Karte in einem strukturierten Bereich aufbauen lassen; langsame kurze Fahrten beobachten.
7. Autonome Strategie bewusst aktivieren. Kartenfahrten benötigen eine ausreichend frische und belastbare Positionsschätzung.

Nach einem gemeldeten WLAN-Verbindungsabbruch startet die Fahrt nicht automatisch wieder. Steuerung erneut übernehmen und den gewünschten Modus bewusst starten.

## 6. Patrouillenroute abfahren

Routen beziehen sich immer auf die aktuelle Karte. Zuerst die Umgebung so weit kartieren, dass die gewünschte Strecke als frei bekannt ist, und den Roboter dann im IDLE stehen lassen.

1. Im Tab „Karte“ unter „Routen · geschlossener Rundkurs“ auf „Neue Route zeichnen“ klicken und die Eckpunkte nacheinander in die Karte setzen. Benachbarte Punkte müssen 25 cm bis 2 m auseinanderliegen.
2. „Kreis schließen“ verbindet den letzten mit dem ersten Punkt. „Letzten Punkt entfernen“ und „Verwerfen“ korrigieren den Entwurf.
3. Anzahl vollständiger Runden eintragen oder „∞ Bis zum Stopp wiederholen“ wählen. „Tempo mit Streckenerfahrung steigern“ lässt gerade, mehrfach sauber befahrene Kanten schneller werden; Kurven bleiben langsam.
4. Optional mit Namen über „Route merken“ im Browser speichern. Gemerkte Routen gelten nur für die Karte, auf der sie gezeichnet wurden.
5. „Route starten“. Der Roboter fährt zuerst den ersten Punkt an und folgt dann dem Rundkurs.

Die Firmware lehnt den Start ab, wenn ein Kantenkorridor unbekannte oder belegte Zellen enthält, die Lokalisierung unsicher ist oder die Karte inzwischen zurückgesetzt wurde. Der Grund erscheint im Dashboard; die Bedeutung der Fehlercodes steht in [API.md](API.md#patrouillenrouten). „Route stoppen“ oder jeder andere Stopp beendet die Fahrt; sie wird nicht selbstständig fortgesetzt.

## 7. Parameter ändern

Im Stillstand einstellen. Fahrkorridor, Abstände, Rampen, Mindestleistung und Anfahrimpulse müssen zum Chassis passen. Die Standard-Mindestleistungen wurden am vorhandenen Roboter ermittelt und dürfen nicht als universelle Werte übernommen werden. Validierte Einstellungen liegen im NVS; eine neue Firmware allein setzt sie nicht zwingend zurück.

## 8. OTA-Update

Nur ein passendes Anwendungsimage verwenden und den Roboter im Stillstand lassen. Die neue Firmware startet den ESP32 neu.

```powershell
.\tools\ota_upload.ps1 ROBOTER_IP .\ota\seeker_bot_latest.bin
```

Alternativ die Updatefunktion im Dashboard verwenden. Der Upload erfolgt in 4096-Byte-Blöcken. Der Image-Projektname wird geprüft; eine kryptografische Herkunftsprüfung ist nicht vorhanden. Keine Images aus unbekannter Quelle laden.

**Gemeinsamer Flashbereich:** OTA überschreibt eine gegebenenfalls vorhandene SLAM-Aufzeichnung im inaktiven Slot. Wichtige Aufzeichnungen zuerst herunterladen.

## 9. Eine problematische Fahrt aufzeichnen

Die vorhandene Aufzeichnung zuerst sichern. Ein neuer Recorder-Start löscht den inaktiven OTA-Slot und damit auch eine dort vorhandene vorherige Firmware.

```powershell
.\tools\slam_record.ps1 status ROBOTER_IP
.\tools\slam_record.ps1 download ROBOTER_IP
.\tools\slam_record.ps1 start ROBOTER_IP
# Fahrt im Dashboard durchführen und anschließend stoppen.
.\tools\slam_record.ps1 stop ROBOTER_IP
.\tools\slam_record.ps1 download ROBOTER_IP
```

`download` vor `start` nur ausführen, wenn tatsächlich eine Aufnahme vorhanden ist. Neue Downloads liegen in `recordings/`. Während der Aufnahme kein OTA durchführen. Aufzeichnungen können private Raumgrundrisse und Bewegungsabläufe enthalten.

Die Offline-Auswertung ist in [TESTING.md](TESTING.md) beschrieben. Bei Problemen hilft [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
