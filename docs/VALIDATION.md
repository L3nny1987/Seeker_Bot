# Verifikationsstand vom 15.10.2026

Diese Prüfung gehört zur Veröffentlichungsvorbereitung für Seeker Bot 0.4.1. Sie wurde lokal unter Windows durchgeführt; sie ist kein GitHub-CI-Lauf und keine Fahrabnahme.

## Frischer Quelltextexport

Der geprüfte Quelltext wurde mit dem Exportwerkzeug in ein neues Verzeichnis entpackt. Es enthielt keine vorhandenen `.pio`-Buildreste, keine verwalteten Komponenten und keine echten WLAN-Zugangsdaten. Für den Firmwarebau wurde ausschließlich die Zugangsdatenvorlage kopiert; cJSON wurde dabei neu aus der Komponentenregistry bezogen.

| Prüfung | Ergebnis |
|---|---|
| 10 C-Testprogramme | Erfolgreich |
| 7 JavaScript-Testprogramme | Erfolgreich |
| Dashboard-Asset ohne vorherigen Firmwarebau | Selbstständig erzeugt und geprüft |
| Firmware `seeker_bot` mit PlatformIO Core 6.2.0 | Vollständiger Neubau erfolgreich |
| Statische RAM-Belegung laut Build | 167584 / 327680 Byte |
| Flash-Anwendung laut Build | 1003899 / 1966080 Byte |
| Anwendungsimage | 1004320 Byte; nicht im öffentlichen Export |
| Compiler-Warnungen und -Fehler im frischen Build-Log | Keine Treffer |
| Veröffentlichungscheck | Erfolgreich, 116 Dateien |

## Jeder Commit des Git-Verlaufs

Jeder Commit ab Firmware 0.2.0 wurde einzeln ausgecheckt, mit Platzhalter-Zugangsdaten gebaut und mit den zu diesem Zeitpunkt vorhandenen Host-Tests geprüft. Änderte ein Commit `sdkconfig.defaults`, `platformio.ini` oder die Partitionstabelle, wurde die generierte Konfiguration verworfen und neu erzeugt.

Alle 14 bis Firmware 0.4.0 geprüften Stände bauen ohne Compiler-Warnung. Die Tests bestehen in 13 Ständen. Im Commit „Keep reset diagnostics and the log before a crash“ scheitert `telemetry_test.js`, weil der DOM-Stub des Tests `Element.after()` nicht kennt; der direkt folgende Commit behebt das. Ab dem Commit mit dem Veröffentlichungswerkzeug besteht in jedem Stand auch der Veröffentlichungscheck. Die beiden anschließenden Commits ändern nur Lizenz und Dokumentation. Die Korrekturen zu 0.4.1 wurden ebenfalls einzeln mit Host-Tests und Firmwarebau geprüft.

## Dokumentation und Konfiguration

Das Inhaltsverzeichnis des Projektbuchs verweist auf alle 18 Kapitel; die neuen Verweise auf `API.md#patrouillenrouten` sind gültig. Tabellenbreiten und Codeblock-Begrenzungen wurden für alle Markdown-Dateien geprüft. Lokale Markdown-Dateiverweise validiert der Veröffentlichungscheck.

Alle fünf Abbildungen lassen sich mit dem ursprünglichen Generator byte-identisch reproduzieren. Das Titelbild wurde damit auf Firmware 0.4.1 aktualisiert und, wie die Architekturgrafik, visuell kontrolliert.

Die VS-Code-Konfiguration wurde als JSON beziehungsweise JSON mit Kommentaren geprüft, der CI-Workflow als YAML. Die PowerShell-Werkzeuge wurden syntaktisch geparst, ohne einen Upload oder eine Fahrt auszulösen. `git diff --check` meldet nur den beabsichtigten Markdown-Zeilenumbruch im Projektbuch und die unveränderte Leerzeile am Ende des cJSON-Lizenztexts.

## Geheimnisse und Grenzen dieser Prüfung

Die beiden lokal hinterlegten WLAN-Zeichenketten wurden weder in der vorgesehenen Veröffentlichungsmenge noch in einem Commit des Git-Verlaufs gefunden. Werte wurden dabei nicht ausgegeben. Ältere Commits enthalten lokale Werkzeugpfade, einen festen COM-Port und eine private LAN-Adresse als Skript-Standardwert; der aktuelle Stand enthält sie nicht.

Die Prüfung ist auf die genannten Inhalte begrenzt. Andere denkbare Geheimnisse, der externe Veröffentlichungszustand, sämtliche Drittanbieter-Lizenzpflichten eines später verteilten Binaries und tatsächliches Fahrverhalten sind damit nicht vollständig geprüft. Der Quelltextexport enthält keine Git-Historie, lokale Zugangsdaten, Firmware oder Aufzeichnungen.

**Noch offen:** private Schwachstellenmeldung auf GitHub und dokumentierte Fahrten mit Firmware 0.4.1 einschließlich Patrouillenrouten.
