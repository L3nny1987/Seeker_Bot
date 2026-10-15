# Veröffentlichung vorbereiten

Stand: 15.10.2026 · Autor: Lennard Schimmer

## Vorbereitet

- Öffentliche Projektbeschreibung und ausführliches Projektbuch mit Originalgrafiken.
- Einrichtung ohne feste COM-Ports und ohne persönliche absolute Rechnerpfade.
- Lokale Zugangsdatenvorlage; Ausschluss von Zugangsdaten, Builds und Aufzeichnungen.
- Cross-Platform-Testwerkzeug, CI-Konfiguration und Vorlagen für Beiträge.
- Technische Grenzen, Sicherheitsmodell und Stand der Hardwareprüfung dokumentiert.
- Gezielte Prüfung und ZIP-Export einer ausdrücklich definierten Quelltextmenge.

Es ist noch kein Remote-Repository eingerichtet und mit dieser Vorbereitung wurde nichts hochgeladen.

## Noch zu entscheiden oder nachzuweisen

1. **Projektlizenz:** Der Autor legt fest, unter welcher Lizenz Quelltext, Dokumentation und Grafiken weitergegeben werden dürfen. Bis dahin enthält das Projekt keine ausdrückliche Open-Source-Lizenz.
2. **Veröffentlichungsziel:** GitHub-/Git-URL und ein privater Kontaktweg für Sicherheitsfunde ergänzen.
3. **Hardwarestatus:** Den aktuellen Stand über wiederholte Fahrten prüfen, bevor Zuverlässigkeits- oder Genauigkeitsangaben als reale Produkteigenschaft veröffentlicht werden.
4. **CI auf dem Zielhost:** Der Workflow ist vorbereitet; ein tatsächlicher GitHub-Lauf steht erst nach einem Upload fest.

## Geprüften Quelltext exportieren

```sh
python tools/check_publication.py
python tools/export_source.py exports/Seeker_Bot_Public_Source.zip
```

Der Export enthält eine SHA-256-Dateiliste. Er umfasst Quelltext, Konfiguration, Dokumentation und Vorlagen. Ausgeschlossen sind insbesondere `.git`, `.pio`, `managed_components`, `src/credentials.h`, Firmware, ELF-Dateien, lokale OTA-Images und Aufzeichnungen. Die Lizenzentscheidung bleibt auch im Export ausdrücklich offen, solange sie nicht getroffen wurde.

Ein ZIP-Export kann als sauberer Ausgangspunkt für ein neues Repository verwendet werden. Wer stattdessen den vorhandenen Git-Verlauf veröffentlichen möchte, muss diesen zusätzlich überprüfen: Das Entfernen einer Datei aus dem aktuellen Stand entfernt frühere Git-Objekte nicht. Bei der aktuellen Untersuchung wurden die beiden lokalen WLAN-Zeichenketten in den erreichbaren Git-Blobs nicht gefunden. Das ist eine gezielte Prüfung dieser Zugangsdaten, keine vollständige Aussage über alle denkbaren früheren Geheimnisse.

## Beschreibung für die spätere Repository-Seite

> Autonomer ESP32-Kettenroboter mit LD20-LiDAR, MPU6050-Sensorfusion, Scan-Matching-SLAM, lokaler Wegplanung und integriertem Browser-Dashboard. Entwickelt von Lennard Schimmer.

Passende Themen: `esp32`, `esp-idf`, `platformio`, `robotics`, `lidar`, `slam`, `navigation`, `mpu6050`, `embedded`, `c`.

Ein reales Foto und ein kurzer dokumentierter Fahrversuch wären eine gute spätere Ergänzung. Die beigefügten Illustrationen sind ausdrücklich keine realen Aufnahmen oder Messkurven.
