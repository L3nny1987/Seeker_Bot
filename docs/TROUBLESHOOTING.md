# Fehlersuche

## Nach einigen Fahrten verschwindet die Verbindung

Zuerst nach der Rückkehr `uptime`, `resetReasonText`, `minHeap`, `heapInternal`, `heapLargest`, `allocFailures` und `/api/diag` sichern. Eine kleine neue Laufzeit spricht für einen Neustart. Eine weiterlaufende Uhr spricht eher für eine unterbrochene Kommunikation ohne Reset.

Für die tatsächlich untersuchte Firmware 0.3.2 sind Watchdog-Neustarts mit extrem knappen Heap-Minima dokumentiert. Das beweist Speicherknappheit und Neustarts, aber noch nicht die genaue blockierende Stelle. Firmware 0.3.3 reduziert Übertragungslast und erweitert die Diagnose; die reale Langzeitwirkung bleibt gesondert zu prüfen.

RSSI, WLAN-Trennzähler und HTTP-Fehler getrennt betrachten. Wenige parallele Dashboard-Clients und ein besserer Funkstandort erleichtern die Eingrenzung. Bei erneutem Watchdog den seriellen Panic-Backtrace mit der zugehörigen ELF-Datei lokal sichern. Siehe [Entwicklungsnotiz](CONNECTIVITY_RELIABILITY.md).

## Der Roboter driftet aus der sichtbaren Raumkarte

1. Tatsächliche Position mit der Kartendarstellung vergleichen. Ein verschobenes Symbol allein ist noch kein physischer Ortswechsel.
2. `slamQuality`, `slamWeak`, `slamLost`, `slamDegenerate`, `poseAgeMs`, `scanAgeMs` und IMU-Diagnose beobachten.
3. Prüfen, ob lange parallele Wände die Bewegung entlang des Gangs kaum bestimmen.
4. Scanlücken und Gyro-Ausfälle vor dem Ereignis suchen. Sensorbefestigung und Bewegung während der Kalibrierung prüfen.
5. Die komplette Fehlerszene aufzeichnen und offline wiederholen.

Eine lokale Karte besitzt keine globale Schleifenschließung. Kleine, langsam aufgebaute Fehler können trotz Sprung- und Wandprüfungen verbleiben. Nach einem Kartenreset müssen die neue Karte und ihr Ursprung neu bewertet werden. Siehe [Navigation 0.3.2](NAVIGATION_RELIABILITY.md).

## Wellenlinien zum Ziel

Kursabweichung und neue Scan-Sequenzen vergleichen. Dieselbe veraltete Blickrichtung darf nicht mehrere Korrekturen auslösen. Kleine Kursimpulse dürfen nicht durch einen langen Anfahrkick verlängert werden. Mindestleistung, Spiel im Antrieb und Gyro-Frische prüfen. Werte im Stillstand einzeln ändern und Auswirkungen dokumentieren.

## LiDAR liefert nichts oder wird ungesund

| `lidarDiagnostic` | Nächste Prüfung |
|---|---|
| `no_uart_bytes` | Versorgung, gemeinsame Masse, TX-Leitung an GPIO 18 |
| `no_ld_header` | Richtige Datenleitung, Baudrate und Signalpegel |
| `no_valid_packets` | CRC-Fehler, Störung, Kabel und Protokoll |
| `waiting_for_revolution` | Winkelübergang und vollständiger Datenstrom |
| `stale_or_incomplete_scan` | Alter, Abdeckung, Paketlücken, Oberflächen |
| `healthy` | Datenerfassung gesund; räumliche Eignung separat prüfen |

Kabelfarben allein genügen nicht für andere Chargen. Die [Verkabelung](WIRING.md) erläutert die aktuelle Variante. Vorne im Radar muss dem tatsächlichen Vorn entsprechen.

## AUTO startet nicht oder bleibt stehen

Controller-Lease, Inbetriebnahmefreigabe, Betriebszustand und LiDAR-Gesundheit prüfen. Zielnavigation braucht eine gültige Kartenposition. Schwache oder verlorene Lokalisierung kann die kartengestützte Fahrt bewusst pausieren. `reason`, `autoPhase` und `navDetail` erklären den aktuellen Zustand.

Ein gemeldeter WLAN-Abbruch löscht den Lease und setzt IDLE. Nach der Rückkehr Steuerung übernehmen und den Modus erneut bewusst starten. Im ESTOP-/FAULT-Zustand zunächst die Ursache beheben und die Verriegelung bewusst zurücksetzen.

## OTA oder Aufnahme funktioniert nicht

Beim ersten Aufbau die korrekte Partitionstabelle per USB installieren. OTA nur mit passendem Anwendungsimage und ruhendem Roboter durchführen. Recorder und OTA benötigen denselben inaktiven Slot; eine Funktion überschreibt die Daten der anderen. Ein leerer Recorder kann nicht heruntergeladen werden. Schreibzugriffe benötigen passende Origin-Header.

## Sinnvolle Angaben für einen Fehlerbericht

Firmwareversion, Hardware, Strategie, reproduzierbare Schritte und Häufigkeit angeben. Relevante Diagnose vor dem Anhängen auf Zugangsdaten, WLAN-Namen, Adressen und private Raumgrundrisse prüfen. Eine externe Positionsreferenz hilft, tatsächlichen Drift von einer geänderten Kartenansicht zu unterscheiden.
