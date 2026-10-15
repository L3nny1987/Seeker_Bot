# Verifikationsstand vom 15.10.2026

Diese Prüfung gehört zur Dokumentations- und Veröffentlichungsvorbereitung für Seeker Bot 0.3.3. Sie wurde lokal unter Windows durchgeführt; sie ist kein GitHub-CI-Lauf und keine Fahrabnahme.

## Frischer Quelltextexport

Der geprüfte Quelltext wurde mit dem Exportwerkzeug in ein neues Verzeichnis entpackt. Es enthielt keine vorhandenen `.pio`-Buildreste, keine verwalteten Komponenten und keine echten WLAN-Zugangsdaten. Für den Firmwarebau wurde ausschließlich die Zugangsdatenvorlage kopiert.

| Prüfung | Ergebnis |
|---|---|
| 8 C-Testprogramme | Erfolgreich |
| 6 JavaScript-Testprogramme | Erfolgreich |
| Dashboard-Asset ohne vorherigen Firmwarebau | Selbstständig erzeugt und geprüft |
| Firmware `seeker_bot` mit PlatformIO Core 6.2.0 | Vollständiger Neubau erfolgreich |
| Statische RAM-Belegung laut Build | 167128 / 327680 Byte |
| Flash-Anwendung laut Build | 989603 / 1966080 Byte |
| Anwendungsimage | 990032 Byte; nicht im öffentlichen Export |
| Compiler-Warnungen und -Fehler im frischen Build-Log | Keine Treffer |
| Veröffentlichungscheck | Erfolgreich |

## Dokumentation und Konfiguration

Das Projektbuch wurde mit einem Markdown-Parser eingelesen. Alle 18 Kapitelziele des Inhaltsverzeichnisses, die Tabellenstruktur und die Codeblock-Begrenzungen wurden geprüft. Die fünf Abbildungen wurden visuell kontrolliert. Lokale Markdown-Dateiverweise wurden über den Veröffentlichungscheck validiert.

VS-Code-Konfiguration wurde als JSON beziehungsweise JSON mit Kommentaren geprüft, der CI-Workflow als YAML. PowerShell-Werkzeuge wurden syntaktisch geparst, ohne einen Upload oder eine Fahrt auszulösen. Der Git-Diff enthielt bei der abschließenden Whitespace-Prüfung keine Fehler.

## Geheimnisse und Grenzen dieser Prüfung

Die beiden aktuell lokal hinterlegten WLAN-Zeichenketten wurden weder in der vorgesehenen Veröffentlichungsmenge noch in den untersuchten erreichbaren Git-Blobs gefunden. Werte wurden dabei nicht ausgegeben. Persönliche absolute Rechnerpfade und die lokale Geräteadresse wurden aus den Veröffentlichungsdateien entfernt.

Die Prüfung ist auf die genannten Inhalte begrenzt. Alte andere Geheimnisse, externer Veröffentlichungszustand, sämtliche Drittanbieter-Lizenzpflichten eines später verteilten Binaries und tatsächliches Fahrverhalten sind damit nicht vollständig geprüft. Der Quelltextexport enthält keine Git-Historie, lokale Zugangsdaten, Firmware oder Aufzeichnungen.

**Noch offen:** Projektlizenz, Veröffentlichungsziel, privater Meldekanal und dokumentierte Langzeitfahrt mit Firmware 0.3.3.
