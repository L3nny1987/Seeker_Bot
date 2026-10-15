# Sicherheitsmodell

Seeker Bot ist ein lokaler Entwicklungsroboter. Die Firmware stellt unverschlüsseltes HTTP bereit und besitzt derzeit keine Benutzeranmeldung und keine kryptografisch signierten OTA-Images. Der Betrieb ist für ein vertrauenswürdiges lokales Netzwerk vorgesehen. Keine Portweiterleitung aus dem Internet einrichten.

## Vorhandene Mechanismen

Controller-Lease und Sequenznummern ordnen konkurrierende Clients und verwerfen verspätete normale Steuerbefehle. Schreibzugriffe benötigen einen zum Host passenden Origin und `X-Seeker-Control: 1`. Diese Browserprüfung verhindert bestimmte Cross-Site-Zugriffe; ein selbst geschriebener LAN-Client kann diese Header ebenfalls setzen.

Manuelle Bewegungsbefehle laufen nach 350 ms aus. Autonomes Fahren pausiert ohne frischen Heartbeat; ein gemeldeter WLAN-Abbruch führt zu IDLE. Die Firmware startet nach dem Boot nicht selbstständig eine Fahrt.

Der Software-Notstopp ist verriegelt, setzt aber eine funktionierende Ausführung und Befehlskette voraus. Er ersetzt keine physische Abschaltung der Motorversorgung. Absturzerkennung und zertifizierte Personenschutzfunktionen sind nicht vorhanden.

## Zugangsdaten und Artefakte

`src/credentials.h` bleibt lokal. Zugangsdaten werden in das Anwendungsimage einkompiliert. Firmware, ELF-Dateien und vollständige Flash-Abbilder deshalb nicht öffentlich hochladen. `.gitignore` und der Quelltextexport nehmen diese Dateien aus; bereits früher veröffentlichte Inhalte werden dadurch nicht rückwirkend entfernt.

Logs, WLAN-Daten und Kartenaufzeichnungen können private Informationen enthalten. Den vorgesehenen Anhang vor einer Veröffentlichung prüfen. Der gezielte Veröffentlichungscheck ersetzt keinen vollständigen Security-Audit.

## Einen Fund melden

Autor: Lennard Schimmer. Sobald ein GitHub-Repository mit privater Schwachstellenmeldung eingerichtet ist, dessen Funktion „Report a vulnerability“ verwenden. Bis ein privater Kontaktkanal veröffentlicht ist, keine Zugangsdaten, privaten Logs oder ausnutzbaren Angriffsschritte in öffentliche Issues schreiben. Ein separater privater Meldekanal ist noch einzurichten.
