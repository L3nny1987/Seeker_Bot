# Seeker_Bot 0.3.3 – Verbindungsabbrüche und ESP32-Neustarts

14.10.2026 · Projekt: C:\Users\Lenna\Documents\PlatformIO\Projects\Seeker_Bot

## Direkt am Roboter festgestellte Fakten

Die Endpunkte `/api/status` und `/api/diag` waren zunächst unter http://192.168.178.91 erreichbar. Auf dem Gerät läuft Firmware 0.3.2. Es wurde keine Fahrt gestartet oder Steuerung übernommen.

Die Reset-Historie meldet drei `interrupt_watchdog`-Neustarts im autonomen Betrieb:

| Vorherige Laufzeit | Minimaler freier Heap | Zustand laut Diagnose |
|---|---:|---|
| 784 s | 316 Byte | AUTO, Fahrt 100 %, WLAN verbunden |
| 124 s | 752 Byte | AUTO, Antrieb 0/0, WLAN verbunden |
| 368 s | 3.148 Byte | AUTO, Antrieb 0/0, WLAN verbunden |

Beim Auslesen des aktuellen Starts: Laufzeit 247 s, freier Heap 43.796 Byte, historisches Minimum 596 Byte, RSSI −82 dBm, vier HTTP-Verbindungen. Seit diesem Start waren keine WLAN-Trennereignisse gezählt. Gleichzeitig enthält das Log zahlreiche HTTP-Sendefehler 11 und 104. Eine spätere Statusabfrage lief in einen Timeout.

Der Nutzer bestätigt eine separate Versorgung bzw. USB. In der gespeicherten Reset-Historie stehen keine Brownouts.

**Bewiesen sind echte Watchdog-Neustarts und extrem knapper Speicher.** Die konkrete blockierende Interrupt-/Critical-Section-Stelle ist ohne Panic-Backtrace noch nicht bewiesen. Netzwerk-Pufferspitzen und unnötiger Datenverkehr sind eine klar identifizierte Belastung und die Arbeitshypothese für den Speicherdruck. Das schwache Funksignal verschärft ausstehende Übertragungen. Ein höher gesetzter oder deaktivierter Watchdog wäre kein belastbarer Fix.

## Implementierte Entlastung

1. **WLAN-Puffer begrenzt:** statische RX-Puffer 10 → 6, dynamische RX/TX-Puffer jeweils 32 → 8. Die Grenzen werden direkt beim Initialisieren angewendet. Die SDK-Defaults wurden entsprechend gesetzt.
2. **TCP-Fenster verkleinert:** Sende- und Empfangsfenster jeweils 5.760 → 2.880 Byte. Auch die vorhandenen, generierten `sdkconfig.seeker_bot` und `sdkconfig.seeker_bot_debug` wurden angepasst, weil neue Defaults bestehende Einstellungen nicht überschreiben. Ein Compile-Check verhindert versehentliche Builds mit den alten großen Fenstern.
3. **Weniger HTTP-Sitzungen:** maximal vier statt sechs Clients; TCP-Keepalive prüft tote Verbindungen. Das Sendetimeout sinkt von zwei auf eine Sekunde.
4. **Keine unsichtbaren Karten-Downloads:** Karte wird nur in der sichtbaren Kartenansicht bei vorhandener Verbindung nachgeladen, regulär alle drei Sekunden. Fehler lösen eine Pause von bis zu 15 Sekunden aus.
5. **Radar nur bei Bedarf:** Die sichtbare Fahransicht erhält weiterhin fünf Scans pro Sekunde. Andere Ansichten erhalten kleine Statusmeldungen mit zwei Aktualisierungen pro Sekunde. Heartbeat und manuelle Befehle behalten ihre unabhängige Taktung und Sicherheitsfristen. Langsame Telemetrie-Anfragen können sich nicht überlappen.
6. **Kartenkompression ohne zusätzliche Heap-Reservierung:** Ein einfaches RLE-Verfahren nutzt den bereits vorhandenen HTTP-Puffer. Neue Clients fragen `/api/map?rle=1` an und verstehen Version 2; ältere Clients erhalten weiterhin Version 1. Wird eine Karte durch Kompression größer oder passt sie nicht in den Puffer, erfolgt der sichere Rückfall auf Version 1. Auch Pfaddaten bleiben exakt erhalten.
7. **Speicherreserve für Steuerung:** Große Karten-/Dashboard-Transfers pausieren unter 16 KiB freiem internem Heap. Kartenübertragungen haben zusätzlich ein Zeitbudget; der bestehende Socket-Sendetimeout begrenzt einen einzelnen blockierten Sendeschritt. Die Deadline ist deshalb keine harte Echtzeitgarantie von 750 ms, verhindert aber unbegrenzt aneinandergereihte Sendewartezeiten.
8. **Nachweis für künftige Fehler:** Ein minimaler, allokationsfreier Hook zählt fehlgeschlagene Speicherreservierungen und speichert Größe, Memory-Caps und Zeitpunkt im RTC-Speicher. Der Bericht `/api/diag` zeigt diese Informationen auch nach einem Reset. Status meldet internen freien Heap, größten freien Block, Allokationsfehler sowie pausierte/fehlgeschlagene große Übertragungen.

Die Navigation und die Schutzmaßnahmen aus 0.3.2 bleiben erhalten. Kein höheres Watchdog-Limit und kein automatischer Fahrtstart wurden ergänzt.

## Tests

Alle acht C-Testprogramme und sechs JavaScript-Tests bestehen, einschließlich der bisherigen SLAM-/Navigationstests.

Neue Prüfungen decken RLE-Roundtrip, 255-Byte-Läufe, kleine Puffer, Rauschdaten mit Rückfall, fehlerhafte komprimierte Daten, Pfaddaten hinter nicht durch vier teilbaren Datenlängen sowie Decoder-Kompatibilität mit tatsächlichen Ausgaben des C-Encoders ab. Browser-Tests prüfen ausbleibende Downloads in verborgenen/offline Ansichten, die getrennte Steuerungstaktung, Rücknahme der Last nach Fehlern und fehlende parallele Telemetrie-Aufrufe.

Die synthetische Raumkarte benötigt **698 statt 16.384 Byte Kartenzellendaten**, also 95,7 % weniger. Kopf- und Pfaddaten kommen hinzu. Dies ist ein Testwert; die Kompressionsrate der echten Raumkarte hängt vom Karteninhalt ab.

## Anwendung und Hardwareprüfung

Das OTA-Image ist eine Anwendungsfirmware für die vorhandene Partitionierung. Ein Update startet den ESP32 neu und ist nur im Stillstand möglich. Nach dem Update das Dashboard einmal vollständig neu laden, damit auch der neue Browsercode aktiv ist.

```powershell
# Im Projektordner, Roboter im Stillstand:
.\tools\ota_upload.ps1 192.168.178.91 .\ota\seeker_bot_0.3.3.bin
```

Vor einem Update vorhandene SLAM-Aufzeichnungen herunterladen, falls sie benötigt werden: Der Recorder verwendet den freien OTA-Slot, den das Update überschreibt.

Für die Prüfung zunächst im Stillstand Status, Karte und Diagnose mehrfach öffnen; anschließend mehrere überwachte Fahrten durchführen. Erwartet wird ein deutlich höheres Heap-Minimum, keine neuen Allokationsfehler und keine weiteren Watchdog-Neustarts. Zeitweise zurückgestellte Kartenübertragungen bei knappem Speicher sind beabsichtigt. Die praktische Stabilität ist erst durch diese Hardwareprüfung bestätigt.

Bleiben Interrupt-Watchdog-Neustarts bei ausreichendem Speicher bestehen, ist als nächster Schritt ein serieller Panic-Backtrace erforderlich. Beim Auslesen war kein USB-Port des ESP32 angeschlossen; verfügbar war nur COM1.

## Quellen

[Espressif: Wi-Fi-Speicher und Puffergrenzen](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi-driver/wifi-performance-and-power-save.html) erklärt die dynamische Speicherbelegung von WLAN/lwIP. [Espressif: Interrupt-Watchdog](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/wdts.html) beschreibt die Bedeutung blockierter Interrupts. [Espressif: Heap-Diagnose](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/heap_debug.html) dokumentiert die Speicherdiagnose. Die konkrete Implementierung wurde zusätzlich gegen die lokal installierten ESP-IDF-6.1-Header und Quellen geprüft.

Build-Verifikation: ESP32 Release 0.3.3 erfolgreich. Acht C-Testprogramme und sechs JavaScript-Tests erfolgreich. OTA-Image: 990032 Byte. SHA-256: A225FA92CD2AEAD619CA07F4DAEF46944E7ED985423A981CAD6D0F636E2A8451.

Während der Arbeiten wurde der ESP32 ohne Firmwareänderung unerreichbar: HTTP lief wiederholt in 4-5-s-Timeouts, der lokale Rechner meldete anschließend bei Ping: Zielhost nicht erreichbar. Ein Update oder Hardwaretest wurde bis zu diesem Stand nicht durchgeführt.
