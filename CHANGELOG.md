# Änderungsübersicht

Die Versionsabschnitte beschreiben den Entwicklungsstand des Quelltexts. Sie behaupten keine bereits veröffentlichte oder auf Hardware installierte Version.

## Noch unveröffentlicht – Repository und Dokumentation

- Projektbuch von Lennard Schimmer mit Originalgrafiken, Inhaltsverzeichnis und PDF-Hinweisen.
- README, Einrichtung, Verkabelung, Architektur, API, Tests und Fehlersuche.
- Beitrags- und Sicherheitsinformationen, Drittanbieterhinweise sowie Issue-/PR-Vorlagen.
- Portable Testausführung ohne vorherigen Firmwarebau und vorbereitete GitHub-CI.
- Lokale Pfade und Geräteadresse aus Veröffentlichungsdateien entfernt; Portwahl dem jeweiligen Aufbau überlassen.
- Zugangsdatenvorlage, gezielte Veröffentlichungsprüfung und Quelltextexport ohne lokale Geheimnisse oder Firmware.
- MIT-Lizenz für Quelltext, Dokumentation und Grafiken.
- Dokumentation, API-Beschreibung und Testangaben auf Firmware 0.4.0 abgeglichen.

## 0.4.1 – Routen startbar und robuster, 15.10.2026

- Routenstart wurde abgewiesen, sobald die Auto-Leistung unter der Mindestleistung des Fahrmotors lag. Diese Sperre entfällt; die Ausgangsstufe hebt jeden Fahrbefehl ohnehin auf die Mindestleistung.
- Freiraumprüfung körpergerecht: Die Mittellinie muss durch bekannt freie Fläche führen, im Korridor aus halber Körperbreite plus 5 cm darf kein kartiertes Hindernis liegen. Bisher war entlang jeder Kante ein 60 cm breites, vollständig bekannt freies Band nötig. Auf einer realen Karte von 8,7 m² steigt die nutzbare Fläche damit von 27 % auf 62 %.
- Genauere Startfehler `route_unexplored` und `route_obstacle` statt `route_unknown_or_blocked`.
- Wegpunkte gelten auch beim Durchfahren entlang der Kante als erreicht; Fangradius 20 statt 12 cm, Abweichungsgrenze 60 statt 40 cm.
- Die Anfahrt zum Startpunkt endet nicht mehr als „unerreichbar“, wenn der Kartenpfad sein leicht verschobenes Ziel erreicht.
- Dashboard: rot markierter Problemabschnitt, Hinweis „Zum Start fehlt“, Schrittanleitung und verständliche Abbruchgründe.
- Host-Test einer verrauschten Patrouille mit den realen Einstellungen des Roboters.

## 0.4.0 – Patrouillenrouten, 15.10.2026

- Geschlossene Rundkurse mit 3 bis 64 Punkten im Dashboard auf der Karte zeichnen, pro Karte im Browser merken und 1…n-mal oder bis zum Stopp abfahren.
- Start nur im Stillstand, auf derselben Kartengeneration, mit gesunder Lokalisierung und bekannt freiem Korridor für den Roboterkörper entlang jeder Kante.
- Abbruch bei Kartenreset, zu großer Abweichung von der Kante oder fehlendem Fortschritt; Pause bei unsicherer Lokalisierung; keine selbstständige Wiederaufnahme.
- Optional adaptives Tempo: Gerade, mehrfach sauber befahrene Kanten dürfen schneller werden, Kurven bleiben konservativ.
- `GET`/`POST /api/route` mit kompaktem Binärpaket statt JSON-Baum; Routenfelder im Status.
- Host-Tests für Route Runner, geschlossene Patrouillenfahrt und Routen-Oberfläche.

## 0.3.3 – Verbindung und Speicher, 14.10.2026

- WLAN-/TCP-Puffer reduziert und große HTTP-Antworten bei knapper interner Reserve begrenzt.
- Optionale RLE-Kartenübertragung mit kompatiblem Rohdaten-Rückfall.
- Dashboard-Polling an sichtbare Ansichten, Offline-Zustand und Fehler angepasst.
- Allokationsfehler, Speicherfragmentierung und große Übertragungsfehler besser diagnostizierbar.
- Host-Regressionen und Firmwarebau erfolgreich; Langzeitfahrt gesondert nachzuweisen.

Details: [CONNECTIVITY_RELIABILITY.md](docs/CONNECTIVITY_RELIABILITY.md).

## 0.3.2 – Navigation und Lokalisierung, 12.10.2026

- Vollständige zeitbezogene LiDAR-Umläufe und verbesserte Behandlung von Datenlücken.
- Positionsfortschreibung bei verworfenen Matches begrenzt; Wanddurchquerung und Mehrdeutigkeit geprüft.
- Frische Gyroskop-Rückmeldung, begrenzte Kursimpulse und Hysterese für Kartenfahrten.
- Kartenstrategien warten auf belastbare Lokalisierung.
- Synthetische Regressionen für Drift, Drehung und Rückmeldung ergänzt.

Details: [NAVIGATION_RELIABILITY.md](docs/NAVIGATION_RELIABILITY.md).

## 0.3.1 – Aufzeichnung und Diagnose, 09.10.2026

- SLAM-Eingaben (Scan, Gyro-Yaw, Motorbefehl) im inaktiven OTA-Slot aufzeichnen und offline mit `test/slam_replay.c` wiederholen.
- Log vor einem Absturz im RTC-Speicher sowie Reset-Historie im NVS; Bericht unter `/api/diag`.

## 0.3.0 – OTA-Updates, 09.10.2026

- Zwei OTA-Anwendungsslots mit Rollback; wiederaufnehmbarer Upload in 4-KB-Blöcken über Dashboard oder `tools/ota_upload.ps1`.
- Jeder Build wird zusätzlich nach `ota/` kopiert.
- Stillstehende Scans prägen die Karte stärker als Scans während der Fahrt; Exploration hält an einem Ziel fest und korrigiert den Kurs mit Hysterese.
- Größere Karte (12,8 m) und Qualitätsmaß für große Räume.

## Frühere Entwicklung

Bis Firmware 0.2.0 dokumentiert der Git-Verlauf den Aufbau von Motorsteuerung, LiDAR, Dashboard, MPU6050-Unterstützung, SLAM, Wandfolge und Zielnavigation. Für diese Schritte werden keine nachträglich erfundenen Versionsnummern vergeben.
