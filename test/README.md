# Hardwarefreie Tests

Ab Repository-Wurzel `python tools/run_host_tests.py` ausführen. Erforderlich sind Python, Node.js und GCC. Eine Verbindung zum Roboter und WLAN-Zugangsdaten werden nicht benötigt.

Die Suite enthält 10 C- und 7 JavaScript-Testprogramme. Ausführliche Testzuordnung, Replay und Fahrabnahme: [TESTING.md](../docs/TESTING.md).

`slam_replay.c` ist ein separates Werkzeug für aufgezeichnete LiDAR-/Gyrodaten. `dashboard_mock.js` stellt eine synthetische Dashboard-Demo bereit und steuert keine Hardware.
