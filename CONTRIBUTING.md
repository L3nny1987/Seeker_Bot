# Beiträge zu Seeker Bot

Autor und Maintainer: **Lennard Schimmer**. Das Projekt steht unter der [MIT-Lizenz](LICENSE); eingereichte Beiträge werden unter derselben Lizenz übernommen.

## Einrichten und ändern

Die [Einrichtung](docs/GETTING_STARTED.md) beschreibt den Build. Für Änderungen eine eigene Branch verwenden, den konkreten Fehler beziehungsweise das gewünschte Verhalten beschreiben und die Änderung darauf begrenzen.

GPIOs gehören in den zentralen Konfigurationsheader. Neue Motorbefehle laufen über den Robot Manager. Scan- und Pose-Zeitbezüge, Koordinaten, Vorzeichen und Einheiten an den Schnittstellen dokumentieren. Große neue Puffer und Allokationen müssen zum ESP32-Speicherbudget passen.

## Prüfen

```sh
python tools/run_host_tests.py
python tools/check_publication.py
pio run -e seeker_bot
```

Den Firmwarebau bei Firmware- oder Buildänderungen ausführen. Für geänderte Algorithmen einen aussagekräftigen reproduzierbaren Testfall ergänzen; reine Textkorrekturen brauchen keine künstlichen Verhaltenstests. Simulation, Codeprüfung und tatsächliche Fahrversuche im Pull Request getrennt benennen.

## Fehlerberichte

Version, Zustand, Hardware, Reproduktionsschritte und relevante Diagnose angeben. Bei Neustarts Resetgrund, Heap-Minimum und gegebenenfalls UART-Backtrace sichern. Für Drift einen Recorder-Datensatz erwägen. Vor dem Teilen private Daten entfernen.

WLAN-Zugangsdaten, lokal gebaute Firmware, ELF-Dateien und private Aufzeichnungen gehören nicht in Issues oder Pull Requests. Sicherheitsrelevante Funde gemäß [SECURITY.md](SECURITY.md) behandeln.
