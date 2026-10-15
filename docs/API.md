# HTTP-API und Datenformate

Stand: Firmware 0.3.3. Basisadresse: `http://ROBOTER_IP`. Die Implementierung in `src/web_server.c` ist bei Abweichungen maßgeblich.

## Endpunkte

| Methode | Pfad | Inhalt |
|---|---|---|
| GET | `/` | Eingebettetes gzip-Dashboard |
| GET | `/api/status` | Status ohne vollständigen Scan |
| GET | `/api/telemetry` | Status mit eingebettetem Scan |
| GET | `/api/scan` | Aktueller Scan als JSON |
| GET | `/api/map` | Binäre Karte, Version 1 |
| GET | `/api/map?rle=1` | Version 2 mit RLE, falls kleiner; sonst Version 1 |
| POST | `/api/control` | Steuerung, Zustände und Parameter |
| GET | `/api/diag` | Diagnose, Neustarthistorie und Log |
| POST | `/api/diag?cmd=clear` | Diagnosezähler und Historie löschen |
| GET | `/api/record` | Recorder-Status |
| GET | `/api/record?offset=N&len=N` | Aufzeichnungsbytes |
| POST | `/api/record?cmd=start` | Inaktiven OTA-Slot löschen und Aufnahme beginnen |
| POST | `/api/record?cmd=stop` | Aufnahme beenden |
| POST | `/api/ota?offset=N&total=N` | Wiederholbarer Firmwareblock |

Große Antworten können bei knapper interner Speicherreserve mit HTTP 503 zurückgewiesen werden. Clients müssen Fehler behandeln und ihre Abfragerate reduzieren. Das Dashboard verwendet bereits ein entsprechendes Backoff.

## Schreibzugriffe

Schreibzugriffe benötigen einen zur Anfrage passenden Origin und den Marker:

```http
Origin: http://ROBOTER_IP
X-Seeker-Control: 1
```

Für `/api/control` zusätzlich exakt `Content-Type: application/json`. Der Body ist auf 768 Byte begrenzt. Dieser Origin-Schutz und der Controller-Lease sind keine Authentifizierung gegenüber anderen LAN-Clients. Siehe [Sicherheitsmodell](../SECURITY.md).

## Controller-Lease und Befehle

`{"command":"claim"}` fordert die zeitlich begrenzte Steuerzuständigkeit an. Bei Erfolg liefert der Server `{"ok":true,"token":...}`. Folgende normale Befehle benötigen dieses Token und eine gegenüber dem zuletzt angenommenen Befehl höhere ganzzahlige `sequence`. Der Header-Schutz bleibt unabhängig davon erforderlich.

| `command` | Zusätzliche Felder / Verhalten |
|---|---|
| `claim` | Token anfordern, sofern möglich |
| `heartbeat` | Lease erneuern; verlängert keinen manuellen Bewegungsbefehl |
| `release` | Steuerung aufgeben |
| `stop` | Normale Fahrt beenden und IDLE herstellen |
| `reset` | Stoppverriegelung zurücksetzen und IDLE herstellen |
| `commission` | Inbetriebnahmefreigabe im IDLE-Zustand setzen |
| `mode` | `mode`: `idle`, `manual` oder `auto` |
| `move` | Im manuellen Modus: `drive` und `turn`, jeweils ganze Werte −100 bis +100 |
| `map_reset` | Kartierung zurücksetzen; im AUTO-Zustand abgewiesen |
| `settings` | Vollständigen gültigen Parametersatz im IDLE-Zustand speichern |
| `estop` | Verriegelter Stopp; benötigt kein Controller-Token |

Beispiel für einen Lease-Heartbeat, mit einem bereits erhaltenen Beispieltoken:

```json
{"command":"heartbeat","token":123456,"sequence":1}
```

Für `mode: auto` sind `strategy` und gegebenenfalls `goalX`/`goalY` relevant. Strategien: `explore`, `wall_left`, `wall_right`, `goal`, `path`, `home`, `path_home`. Zielkoordinaten liegen in Millimetern innerhalb der lokalen Weltkarte. Home bedeutet den Ursprung, keinen Dockvorgang. Zielkoordinaten werden auf endliche Werte von −10000 bis +10000 geprüft; die tatsächliche feste Karte ist kleiner als dieser Eingabebereich.

Vor AUTO müssen Inbetriebnahmefreigabe, LiDAR und Betriebszustand passen; Zielstrategien verlangen eine gültige Kartenposition. Weitere Qualitätsbedingungen können die Fahrt anschließend pausieren. Ein HTTP-Erfolg bedeutet die Annahme des Befehls, nicht das Erreichen des Ziels.

Ungültige Eingaben liefern typischerweise HTTP 400, unpassende Betriebszustände oder Lease-Konflikte HTTP 409 und falsche Origin-Header HTTP 403. Während eines OTA-Vorgangs sind normale Steuerbefehle bis auf `estop` gesperrt.

## Einstellungen

`settings` überträgt alle folgenden Felder; partielle Aktualisierung ist nicht vorgesehen:

```text
manualStopMm, manualRotateMm, autoStopMm, autoRotateMm, autoSlowMm,
autoDrivePercent, autoTurnPercent, autoSlowPercent, rampMs, reverseMs,
turnMs, resumeMarginMm, robotWidthMm, autoLimitS, turnRampMs,
steerMinPercent, wallDistanceMm, driveMinPercent, turnMinPercent,
driveKickMs, turnKickMs
```

Die gültigen Bereiche und Beziehungen stehen in `src/motion_settings.h`. Beispielsweise muss die Langsamzone über Stoppabstand plus Wiederanfahrreserve liegen. Die NVS-Strukturversion ist aktuell 4. Körperabstände beziehen sich auf den Roboterkörper; der Wandfolgeabstand wird seitlich ab LiDAR-Mitte bestimmt.

## Status und Scan

Der Status enthält unter anderem `version`, `mode`, `reason`, `controller`, `linkOk`, `lidarHealthy`, `coverage`, `scanAgeMs`, `scanSequence`, Motorwerte und Navigationsdetails. Für die Lokalisierung sind `poseValid`, `poseX`, `poseY`, `poseDeg`, `poseAgeMs`, `slamQuality`, `slamLost`, `slamWeak` und `slamDegenerate` relevant. Speicherdiagnose umfasst `heap`, `minHeap`, `heapInternal`, `heapLargest` und `allocFailures`.

`GET /api/scan` liefert `fresh`, `rpm`, `sequence`, `ageMs` und `distance` mit 360 Werten. Entfernungen sind Millimeter; 0 bedeutet kein gültiger Messwert. Index 0 zeigt nach vorne, 90 nach rechts, 180 nach hinten, 270 nach links. `frontMm` und ähnliche Rohabstände im Status sind ab LiDAR gemessen; `navCenterMm` und `clearRearMm` sind freie Fahrstrecken der Körperkante.

## Binärkarte SMAP

Alle Mehrbytewerte sind Little Endian. Floats sind IEEE-754 binary32. Jede Antwort beginnt mit einem 40-Byte-Header:

| Offset | Typ | Bedeutung |
|---:|---|---|
| 0 | 4 Bytes | ASCII `SMAP` |
| 4 | `uint8` | Version 1 oder 2 |
| 5 | `uint8` | Flags: Bit 0 Pose gültig, Bit 1 verloren, Bit 2 Ziel gesetzt |
| 6 | `uint16` | Rasterbreite und -höhe, aktuell 256 |
| 8 | `uint16` | Zellgröße in mm, aktuell 50 |
| 10 | `uint16` | Anzahl Fahrspurpunkte |
| 12 / 16 | `float32` | Pose x / y in mm |
| 20 | `float32` | Pose theta in Radiant |
| 24 / 28 | `float32` | Ziel x / y in mm |
| 32 | `uint8` | Qualität 0–100 |
| 33 | `uint8` | Reserviert |
| 34 | `uint16` | RLE-Byteanzahl bei v2; bei v1 0 |
| 36 | `uint32` | Anzahl integrierter Scans |

Danach folgt das Raster und unmittelbar danach die Fahrspur. Rasterindex ist `gy * size + gx`. Der Ursprung ist bei 128/128; positive Welt-y läuft in Richtung größerer gy. Für ein Bild mit positiver y nach oben ist die Zeilenfolge entsprechend zu spiegeln.

**Version 1:** Vier Zellklassen pro Byte, niederwertigste zwei Bits zuerst. Gesamtlänge des Rasters `size * size / 4`. Klassen: 0 unbekannt, 1 frei, 2 wahrscheinlich belegt, 3 belegt.

**Version 2:** RLE über denselben gepackten Bytes. Jedes Paar ist `(Anzahl, Wert)`, mit Anzahl 1–255. Die Dekodierung muss genau die erwartete v1-Rasterlänge ergeben. Null-Läufe, unvollständige Paare, Überläufe oder widersprüchliche Längen sind ungültig. Der Server liefert v2 nur nach Opt-in und wenn die Kodierung kleiner ist und in den vorhandenen Puffer passt.

**Fahrspur:** Je Punkt zwei `int16`, x und y in Zentimetern. Ihr Startoffset ist `40 + Rasterdatenlänge`; bei v2 ist keine zusätzliche Ausrichtung auf vier Byte vorgesehen. Decoder sollten deshalb unaligned-sichere Zugriffe verwenden. Die Karte kann sich während der Übertragung ändern und ist kein atomarer Snapshot.

## OTA und Recorder

OTA verwendet `application/octet-stream` mit maximal 4096 Byte pro Block. Die Antwort meldet unter anderem `ok`, `written` und `done`. Für Wiederholungen und Fehlerbehandlung die Implementierung in `tools/ota_upload.ps1` verwenden. Eine Uploadsitzung läuft nach 30 Sekunden ohne Block aus. Nach vollständiger Annahme startet das Gerät neu.

Recorderdaten bestehen aus 768-Byte-Datensätzen. Das Format wird in `src/mapper.c` geschrieben und in `test/slam_replay.c` gelesen. Bei Formatänderungen beide Seiten gemeinsam aktualisieren.

Recorder und OTA teilen sich den inaktiven Anwendungsslot. Recorderstart löscht dessen vorherigen Inhalt, OTA überschreibt die Aufnahme. Keines der beiden Verfahren ist eine zusätzliche unabhängige Datensicherung.
