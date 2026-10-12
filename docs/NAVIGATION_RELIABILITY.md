# Seeker_Bot: Navigation und sporadische Karten-Drift

Stand: 12.10.2026 · Firmware 0.3.2
Projekt: C:\Users\Lenna\Documents\PlatformIO\Projects\Seeker_Bot

Die vorhandenen, noch nicht committeten SLAM-, OTA-, Dashboard- und Diagnoseänderungen wurden als Ausgangspunkt erhalten. Es wurde keine Firmware auf den Roboter übertragen und keine Fahrt gestartet.

## Aufbau des Projekts

ESP32 mit ESP-IDF/PlatformIO, LD20 an UART und optional MPU6050 über I2C. Der Antrieb besitzt eine Fahrt- und eine Drehachse; er ist kein klassischer Differentialantrieb. Der Robot Manager führt alle 20 ms die Motorsteuerung aus. Der Mapper berechnet aus LIDAR-Umläufen die Position, Karte und Wegpunkte. Die Navigation verbindet Kartenpfad, Exploration, Bug2 und Wandfolge mit einer reaktiven Hindernisprüfung. Es gibt keine Rad-Odometrie.

## Konkrete Lücken und Änderungen

- **Verworfene Messung verschob trotzdem die Position:** Der bisherige Fehlerpfad übernahm die vorhergesagte Position aus der alten Geschwindigkeit. Jetzt bleibt die letzte bestätigte X/Y-Position erhalten; die Geschwindigkeit wird gelöscht. Gyro-Messungen dürfen weiterhin die Orientierung aktualisieren. Verworfene Scans verändern die Karte nicht.
- **Scheinbar guter Match bei unbestimmter Position:** Zwei parallele Wände können eine hohe Trefferquote liefern, ohne die Bewegung längs der Wände zu bestimmen. Eine Prüfung der Kartengradienten erkennt diese fehlende Information. Solche Messungen werden verworfen und als `slamDegenerate` gemeldet.
- **Unplausible Sprünge:** Neue Positionen müssen zur maximalen Geschwindigkeit und Scanzeit passen. Die Position darf weder in eine kartierte Wand springen noch im normalen Tracking durch eine Wand auf deren andere Seite wechseln. Die separate Relokalisierung darf eine größere Korrektur vornehmen.
- **Leere Scans und Zeitlücken:** Zu wenige Punkte zählen nun als fehlgeschlagene Lokalisierung. Längere Unterbrechungen lösen eine Relokalisierung aus.
- **Weiterfahrt nach bloßer Wartezeit:** Exploration, Kartenpfad und Bug2 warten jetzt bis die Lokalisierung wieder gesund ist. Nach einem Fehler werden drei aufeinanderfolgende erfolgreiche Scans verlangt, damit einzelne gute Messungen kein ständiges Stoppen und Wiederanfahren auslösen. Die Prüfung unterbricht auch bereits laufende Ausweichmanöver. Eine junge Karte wird zuerst im Stillstand aufgebaut. Die bestehende Auto-Zeitbegrenzung bleibt aktiv.
- **Veraltete Positionsdaten:** Eine Position ist nach 300 ms ohne Mapper-Aktualisierung nicht mehr zur Navigation freigegeben. Frische Gyro-Messungen aktualisieren die Orientierung zwischen zwei Scans; IMU-Neustarts werden über die Epochennummer erkannt.
- **Zu kräftige kleine Kurskorrekturen:** Ziel- und Wegpunktkorrekturen nutzen höchstens 80 ms lange Drehimpulse, kurze Anlaufimpulse und anschließende Beruhigung. Ohne Gyro wird vor dem nächsten Impuls eine neuere Mapper-Position verlangt. Mit Gyro kann der Impuls bereits vor dem Überschwingen enden. Größere Drehungen und die vorhandenen Befreiungsmanöver bleiben erhalten.
- **LIDAR-Umläufe nach Unterbrechungen:** Die Scan-Zusammenstellung prüft Winkelkontinuität und Umlaufdauer. CRC- und UART-Fehler verwerfen den angefangenen Umlauf. Der Zeitstempel wird auf den Winkelübergang korrigiert, auch wenn dieser mitten im Paket liegt. Nach einem Fehler wird erst wieder ein vollständiger Umlauf veröffentlicht.
- **Alte Hindernisse aus vorherigen Scans:** Ergänzungen aus dem vorherigen Scan sind nur noch im Stillstand, bei einzelnen Lücken und passenden aktuellen Nachbarpunkten erlaubt. Größere fehlende Bereiche bleiben unbekannt. Fahrt und Rückwärtsmanöver benötigen bekannte Sektoren.
- **Karten-Reset:** Veraltete Erkundungsziele und Gyro-Referenzen werden zusammen mit der Karte verworfen.

Zusätzliche Statusfelder: `slamWeak`, `slamDegenerate`, `poseAgeMs`, `lidarDroppedScans`, `lidarUartErrors`. Das Dashboard zeigt bei unsicherer Position einen entsprechenden Zustand.

## Prüfung und Aussagegrenzen

Alle 7 C-Testprogramme und 6 Dashboard-/JavaScript-Tests bestehen. Der Host-Runner kompiliert C mit `-Wall -Wextra -Werror`.

Neue Regressionstests decken Positionsstillstand nach verworfenen Messungen, leere Scans, Zeitlücken, Wanddurchquerungen, parallele Wände, LIDAR-Unterbrechungen, Gyro-Epochen und veraltete Positionsdaten ab. Der Positions-Test scheitert mit der bisherigen SLAM-Implementierung und besteht mit dem Fix.

In der Simulation mit 160-ms-Positionsupdates, 80 % Mindest-PWM und dem bisherigen 300-ms-Anlaufimpuls dauern kleine Lenkkorrekturen vorher bis zu 160 ms, jetzt höchstens 80 ms. Mit dem Fix entstehen in diesem Szenario keine Richtungswechsel und kein Überschwingen; der seitliche Versatz liegt nach 6 s bei 38–40 mm. Diese Werte stammen aus einem vereinfachten Motormodell und sind keine Messung am Roboter.

Die bestehende Simulation der großen Halle besteht weiterhin: 938 Scans, kein Lokalisierungsverlust, maximal rund 34 mm Positionsfehler und 0,2° Orientierungsfehler. Schnelle Drehungen mit Gyro und Zeitversatz bestehen ebenfalls. Ohne Gyro ist die bereits vorhandene extreme Dreh-Simulation weiterhin nicht zuverlässig lokalisierbar.

Die vorhandene Aufzeichnung `slam_20261009_133654.bin` enthält 51 Scans ohne Fahr- oder Drehbefehl. Der Replay zeigt weder vorher noch nachher Positionssprünge. Damit lässt sich der sporadische Ausreißer während der Fahrt nicht nachweisen. Für die reale Ursache ist weiterhin eine Aufzeichnung einer betroffenen Fahrt nötig. Die Änderungen schließen nachgewiesene Software-Lücken, garantieren aber ohne Fahrtest noch keine zuverlässige Navigation auf der Hardware. In geometrisch uneindeutigen Bereichen kann der Roboter jetzt häufiger stehen bleiben; ohne zusätzliche Odometrie ist das die belastbare Reaktion.

## Lokal prüfen

Im Projektordner:

```powershell
.\tools\run_host_tests.ps1
& 'C:\Users\Lenna\.platformio\penv\Scripts\platformio.exe' run -e seeker_bot
```

Die Firmware wird zusätzlich nach `ota\seeker_bot_0.3.2.bin` und `ota\seeker_bot_latest.bin` exportiert. Die hier mitgelieferte BIN-Datei ist ein OTA-Anwendungsimage für die vorhandene Partitionierung, kein vollständiges Flash-Abbild einschließlich Bootloader.

## Nächster Fahrtest

1. Firmware 0.3.2 übertragen; nach dem Einschalten die Gyro-Kalibrierung im Stillstand abwarten. Eine neue Karte im Stillstand aufbauen lassen.
2. Eine kurze gerade Zielanfahrt, eine 90°-Drehung und die Rückfahrt beobachten. Karte, tatsächliche Position und die neuen Diagnosefelder vergleichen.
3. Mit dem vorhandenen Recorder die komplette Fahrt einschließlich des sporadischen Fehlers aufzeichnen. Ein neuer Recorder-Start löscht die vorherige Aufzeichnung im freien OTA-Slot; vorhandene Aufzeichnungen zuerst herunterladen. Danach dürfen während einer Aufnahme keine OTA-Uploads laufen, weil beide denselben Slot verwenden.

```powershell
# Vorhandene Aufnahme sichern, falls vorhanden:
.\tools\slam_record.ps1 download ROBOTER_IP
# Neue Aufnahme starten, wenn der Roboter IDLE ist:
.\tools\slam_record.ps1 start ROBOTER_IP
# Fahrt über das Dashboard durchführen; anschließend:
.\tools\slam_record.ps1 stop ROBOTER_IP
.\tools\slam_record.ps1 download ROBOTER_IP
```

Erwartung: Bei `slamWeak`, `slamDegenerate`, `slamLost` oder veralteter Position hält eine Kartenfahrt an. Die letzte bestätigte Position darf durch verworfene Scans nicht weiter aus dem Raum wandern. Weiterfahrt erfolgt erst mit wieder belastbarer Lokalisierung.

Protokollreferenzen für Scanreihenfolge, Winkeleinheiten und Paketdaten: [LD20 Development Manual](https://wiki.youyeetoo.com/en/Lidar/LD20) und [LDROBOT SDK – Datenverarbeitung](https://github.com/ldrobotSensorTeam/ldlidar_sdk/blob/master/src/ldlidar_dataprocess.cpp). Der angenommene UART-Übertragungsversatz ist softwareseitig berücksichtigt; die genaue Sensorlatenz muss am realen Gerät überprüft werden.

Abschließender Build: ESP32 Release erfolgreich, keine Compiler-Warnungen im Build-Log. RAM 167104 / 327680 Bytes; Flash 987167 / 1966080 Bytes. OTA-Datei: 987600 Bytes. Alle 13 Testprogramme erfolgreich.

SHA-256 des mitgelieferten OTA-Images: `C73F01AFD61E11674B0B2BFE92B910017BAF2A71EBEEF65DE6562ACED32B4DA9`.
