# Änderungsübersicht

Die Versionsabschnitte beschreiben den Entwicklungsstand des Quelltexts. Sie behaupten keine bereits veröffentlichte oder auf Hardware installierte Version.

## Noch unveröffentlicht – Repository und Dokumentation

- Projektbuch von Lennard Schimmer mit Originalgrafiken, Inhaltsverzeichnis und PDF-Hinweisen.
- README, Einrichtung, Verkabelung, Architektur, API, Tests und Fehlersuche.
- Beitrags- und Sicherheitsinformationen, Drittanbieterhinweise sowie Issue-/PR-Vorlagen.
- Portable Testausführung ohne vorherigen Firmwarebau und vorbereitete GitHub-CI.
- Lokale Pfade und Geräteadresse aus Veröffentlichungsdateien entfernt; Portwahl dem jeweiligen Aufbau überlassen.
- Zugangsdatenvorlage, gezielte Veröffentlichungsprüfung und Quelltextexport ohne lokale Geheimnisse oder Firmware.
- Projektlizenz noch zur Entscheidung durch den Autor offen.

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

## Frühere Entwicklung

Der Git-Verlauf dokumentiert den Aufbau von Motorsteuerung, LiDAR, Dashboard, MPU6050-Unterstützung, SLAM, Wandfolge und Zielnavigation. Für ältere Änderungen werden hier keine nachträglich erfundenen Versionsnummern vergeben.
