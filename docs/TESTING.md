# Prüfen und Fehler reproduzieren

## Host-Regressionen

Voraussetzungen: Python 3, Node.js und GCC auf dem Suchpfad. Unter Windows kann beispielsweise ein passend eingerichteter MSYS2-UCRT64-GCC verwendet werden. Alle Befehle gelten ab Repository-Wurzel.

```sh
python tools/run_host_tests.py
python tools/check_publication.py
```

Ein anderer Compilerpfad kann mit `--compiler` angegeben werden. Auch die JavaScript-Motorprüfung verwendet dann denselben Compiler. Das Werkzeug erzeugt sein Dashboard-Testasset selbst und benötigt weder einen vorherigen Firmwarebau noch Zugangsdaten. Testdateien liegen anschließend unter `.pio/host-tests/`.

PowerShell-Einstieg mit identischer Suite:

```powershell
.\tools\run_host_tests.ps1
```

| C-Testprogramm | Schwerpunkt |
|---|---|
| `navigation_test` | Scanlücken, Korridore, Körperbreite, Orientierung und Fusion |
| `navigation_escape_test` | Strategien, Ausweichen, Blockierung, Wandfolge, Bug2, Pfade und SLAM-Pausen |
| `navigation_feedback_test` | Geschlossene Kurskorrektur mit Mindest-PWM und Impulszeiten |
| `motion_settings_test` | Parameter, Rampen, unmittelbarer Stopp und Richtungswechsel |
| `pose_feedback_test` | Gyro-Frische, Bias und Referenzepochen |
| `map_transport_test` | RLE, Rundlauf, Grenzen, Rauschkarten und Decoder-Fixtures |
| `ld20_scan_test` | Vollständige Umläufe, Zeitüberlauf, Paketlücken und Erholung |
| `slam_test` | Synthetische Szenen, Gyro, Verlust, Wiederfinden und Mehrdeutigkeit |

Die sechs JavaScript-Programme prüfen Dashboard-Asset, Kartenformat, Standardwerte, Motorrichtungen mit gemockter Hardware, Polling und JavaScript-Syntax.

## Firmwarebau

```sh
pio run -e seeker_bot
```

Benötigt lokale `src/credentials.h`; Einrichtung siehe [GETTING_STARTED.md](GETTING_STARTED.md). Ein erfolgreicher Build prüft die Übersetzbarkeit, keine reale Sensorfunktion.

## Offline-Replay

Eine Aufnahme wie in der Einrichtungsanleitung herunterladen. Dann:

```sh
gcc -std=c11 -O2 -I src test/slam_replay.c src/slam.c -lm -o .pio/host-tests/slam_replay
.pio/host-tests/slam_replay recordings/AUFNAHME.bin .pio/host-tests/replay.csv .pio/host-tests/map.pgm
```

Unter Windows beim Ausgabepfad und Aufruf die Endung `.exe` ergänzen. Das Programm vergleicht neu berechnete Positionen mit den im Datensatz gespeicherten Schätzungen und dem integrierten Gyroskopwinkel. Die Aufnahme liefert keine unabhängige Bodenwahrheit. Absolute Positionsgenauigkeit verlangt eine externe Referenz.

Vor Veröffentlichung von Aufnahmen private Raumstrukturen und Betriebsinformationen prüfen. Aufnahmen und Replay-Ergebnisse bleiben standardmäßig außerhalb des Quelltextexports.

## Fahrabnahme am tatsächlichen Roboter

Firmwareversion, Parameter, Versorgung, Boden und Sensorbefestigung festhalten. Für eine neue Firmware mindestens diese Szenen einzeln untersuchen:

| Szene | Beobachten |
|---|---|
| Kurze Geradeausfahrt | Richtung, Anfahrverhalten, Wandstabilität |
| Wiederholte Zielanfahrt | Zielabweichung, Korrekturen, Messfrische |
| Langsame und schnelle Drehung | Gyro, Deskew und Kartenverschmierung |
| Engstelle | Körperkorridor, Drehraum, sinnvoller Stillstand |
| Paralleler Gang | `slamDegenerate`, Drift und Wiederaufnahme |
| Fehlende LiDAR-Daten | Stoppverhalten und Sensorerholung |
| Bedienverbindung unterbrechen | Deadman, Lease, IDLE nach WLAN-Abbruch |
| Mehrere Fahrten hintereinander | Laufzeit, Heap-Minimum, Allokationsfehler und Resetgrund |

Bei einem Fehler Diagnose und möglichst UART-Ausgabe sichern, bevor die Historie gelöscht wird. Reset, Stromausfall, Funkunterbrechung und abgebrochene HTTP-Antwort sind unterschiedliche Ereignisse.

## Veröffentlichungscheck und CI

`tools/check_publication.py` prüft die definierte Exportmenge. Es ersetzt weder einen vollständigen Geheimnisscanner noch die Prüfung alter Git-Objekte, Bildmetadaten oder hochgeladener Artefakte.

Die mitgelieferte CI führt unter Ubuntu Host-Tests, Veröffentlichungscheck und Firmwarebau mit Platzhalter-Zugangsdaten aus. Der lokale Testlauf am 15.10.2026 erfolgte unter Windows. Ein Lauf auf GitHub ist vor dem Hochladen noch nicht nachgewiesen.

Der [Verifikationsbericht](VALIDATION.md) beschreibt den erfolgreichen vollständigen Neubau und die Tests aus einem frischen Quelltextexport.
