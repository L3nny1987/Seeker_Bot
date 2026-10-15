# Drittanbieter und Grundlagen

Die Projektlizenz von Seeker Bot ist noch offen. Diese Entscheidung ändert nicht die Lizenzen der verwendeten Drittanbieter.

## cJSON

Direkte verwaltete Komponente: `espressif/cjson`, aktuell `1.7.19~2` gemäß `dependencies.lock`. cJSON ist unter der MIT-Lizenz veröffentlicht. Der originale Copyright- und Lizenztext aus der aufgelösten Komponente liegt in [docs/licenses/cJSON.txt](docs/licenses/cJSON.txt).

Die Komponente wird beim Build bezogen und ist im öffentlichen Quelltextexport nicht als vollständiger Fremdquelltext enthalten. Wird sie oder ein sie enthaltendes Artefakt weitergegeben, sind die zugehörigen Lizenzhinweise zu erhalten.

## ESP-IDF und Toolchain

Das Buildsystem bezieht ESP-IDF und die Toolchain über PlatformIO. Die aktuelle Auflösung ist in `dependencies.lock` beschrieben. Maßgeblich sind die jeweiligen Lizenzdateien der bezogenen Framework-, Komponenten- und Toolchainpakete. ESP-IDF umfasst Komponenten mit unterschiedlichen Lizenzhinweisen; dieser Überblick ersetzt keine vollständige Liste eines später verteilten Binärartefakts.

Primärquellen: [ESP-IDF-Repository](https://github.com/espressif/esp-idf), [PlatformIO Espressif32](https://github.com/platformio/platform-espressif32), [cJSON](https://github.com/DaveGamble/cJSON).

## Sensorunterlagen und Grafiken

Die [LD20-Unterlagen](https://wiki.youyeetoo.com/en/Lidar/LD20) und das [LDROBOT SDK](https://github.com/ldrobotSensorTeam/ldlidar_sdk) dienen als Protokoll- und Anschlussreferenzen. Fremde Handbuchabbildungen oder SDK-Quelltexte werden durch diese Verweise nicht in das Projekt kopiert.

Die Grafiken in `docs/assets/` wurden für die Projektdokumentation neu erstellt. Sie sind schematische Illustrationen und keine Herstellerzeichnungen, Fotografien oder Messnachweise. Ihre Weitergabe richtet sich nach der noch festzulegenden Projektlizenz.
