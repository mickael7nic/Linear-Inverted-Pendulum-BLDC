# Inverses Pendel auf Schiene – Dokumentation

Stand: 04.11.2026

---

## 1. Überblick

Ein Wagen fährt auf einer 900-mm-Schiene, angetrieben über einen GT2-Riemen von einem BLDC-Motor mit FOC (SimpleFOC). Am Wagen hängt ein 27 cm langes Pendel. An der Spitze des Pendels sitzt ein eigener Sender (ESP32-C6 + BMI270). Er schickt die Rohdaten der IMU per ESP-NOW mit 800 Hz an den Wagen. Die gesamte Sensorfusion und Regelung läuft auf dem Wagen.

Bedient wird das System über eine Web-App (`index.html`, Web Bluetooth) oder über den seriellen Monitor.

| Komponente | Aufgabe |
|---|---|
| Wagen (`Pendel_bldc/`) | FOC-Motorregelung, Homing, Winkelschätzung (EKF), LQR-Balancieren, Swing-up, Beruhigen, BLE, OTA |
| Pendel-Sender (`PendelSender/`) | BMI270 mit 800 Hz auslesen, per ESP-NOW senden, Akku überwachen, Tiefschlaf |
| Web-App (`WebApp/index.html`) | Bedienung, Plot, Log, CSV-Aufzeichnung, Rekorder-Download, OTA-Upload |

---

## 2. Hardware

### 2.1 Wagen

| Teil | Daten |
|---|---|
| Treiberboard | SimpleFOC High-Power ESP32 20A (ESP32-WROOM-32E, 3 × IR2104, 6 × NMOS 30 V/30 A, 2 × INA240A2) |
| Versorgung | 12 V / 10 A (Board erlaubt 12–28 V) |
| Motor | BE5208-300KV, 22 Magnete = 11 Polpaare, Phase–Phase ca. 0,11 Ω |
| Encoder | MT6835 (21 bit, SPI) |
| Riemenscheibe | GT2, 20 Zähne, Radius 6,37 mm |
| Schiene | 900 mm, Wagen 45 mm, Verfahrweg ca. 855 mm |
| Schutz | Elko 470–1000 µF / 25 V an der Brücke, TVS-Diode 1.5KE15A (oder 13A/14A), Kathode an +12 V |
| EN-Pin ESP32 | 1 µF nach GND (gegen Störungen auf der Reset-Leitung) |

**Pins Wagen**

| Funktion | GPIO |
|---|---|
| IR2104 IN Phase A / B / C | 16 / 17 / 5 |
| IR2104 SD# (gemeinsam, aktiv HIGH) | 4 |
| MT6835 CS / SCK / MOSI / MISO | 13 / 18 / 23 / 19 |
| MT6835 CAL_EN | 22 (SCL-Anschluss, `PIN_ENC_CAL_EN`) |
| I2C SDA / SCL (Platine) | 21 / 22 (SDA frei, SCL für CAL_EN) |
| INA240 Phase A / B | 39 / 36 |

**Strommessung:** Seit 04.11.2026 aktiv (`USE_CURRENT_SENSE 1`) mit echter dq-Stromregelung. Zu den bestückten 20-mΩ-Shunts sind 2-mΩ-Shunts parallel gelötet, zusammen 1,82 mΩ (`SHUNT_OHM`). Der Messbereich liegt damit bei etwa ±18 A. Mit dem Strom-Test `I` lässt sich der Wert prüfen: R_eff sollte wie früher bei etwa 0,119 Ω liegen. Das Netzteil liefert 12 V / 10 A.

### 2.2 Pendel

| Teil | Daten |
|---|---|
| Länge | 27 cm, Sensor bei 22,5 cm vom Drehpunkt |
| Ersatzlänge ℓ | **0,21 m** (aus freiem Schwingen bestimmt) |
| Sender | ESP32-C6 Super Mini |
| IMU | BMI270 über SPI, 800 Hz, Filter OSR4 (ca. 75–80 Hz Bandbreite) |
| Akku | LiPo 150 mAh |

**Pins Sender (ESP32-C6 Super Mini)**

| Funktion | GPIO |
|---|---|
| BMI270 INT1 (Data-Ready, Weckpin aus dem Tiefschlaf) | 7 |
| BMI270 CS / SCK / MOSI / MISO | 6 / 5 / 4 / 3 |
| Akkuspannung (Teiler 2:1) | 0 |
| einfache LED | 15 |
| WS2812 RGB-LED | 8 |

---

## 3. Software und Bibliotheken

| | Wagen | Sender |
|---|---|---|
| Board (Arduino IDE) | ESP32 Dev Module | ESP32C6 Dev Module, USB CDC On Boot: Enabled |
| Partition | Minimal SPIFFS (1.9MB APP with OTA) | Standard |
| Bibliotheken | Simple FOC ≥ 2.4, SimpleFOCDrivers ≥ 1.0.9, NimBLE-Arduino ≥ 2.1 | SparkFun BMI270 Arduino Library |
| Flashen | USB oder OTA | nur USB |

**Dateien**

| Datei | Inhalt |
|---|---|
| `Pendel_bldc/Pendel_bldc.ino` | Hauptprogramm Wagen: Includes in fester Reihenfolge, `setup()`, `loop()` |
| `Pendel_bldc/config.h` | alle Pins und Parameter |
| `Pendel_bldc/secrets.h` | `WIFI_SSID`, `WIFI_PASS`, `AP_PASS` |
| `Pendel_bldc/protocol.h` | ESP-NOW-Pakete und Telemetrie (**identisch** in `PendelSender/`) |
| `Pendel_bldc/pendel_core.h` | Regelungskern (auch in der Simulation): `core_math.h`, `lqr.h`, `ekf.h`, `period_meter.h`, `controller.h` |
| `Pendel_bldc/homing.h` | sensorloses Homing (Zustandsautomat) |
| `Pendel_bldc/globals.h` | Hardware-Objekte und gemeinsamer Zustand |
| `Pendel_bldc/log.h` | Log (Serial + BLE), Log-Verlauf über Neustarts |
| `Pendel_bldc/ble.h` | Bluetooth (Befehle, Log, Telemetrie) |
| `Pendel_bldc/espnow.h` | Funk zum Pendel |
| `Pendel_bldc/sensors.h` | Wagenposition, IMU-Umrechnung, Pendel-Kalibrierung |
| `Pendel_bldc/recorder.h` | 800-Hz-Rekorder |
| `Pendel_bldc/control_task.h` | Regel-Task im 800-Hz-Takt |
| `Pendel_bldc/ota_web.h`, `ota.h` | Firmware-Update (Web-Upload, WLAN, ArduinoOTA) |
| `Pendel_bldc/status.h` | Statusausgabe (`?`) |
| `Pendel_bldc/belt_sensor.h` | Riemenmarke (TCS34725) für die Mittensuche |
| `Pendel_bldc/cogging.h` | Rastmoment-Kompensation (`M`) |
| `Pendel_bldc/led_strip.h` | WS2812-Leiste: Live-Anzeige, LED-Test |
| `Pendel_bldc/ui_commands.h` | Befehle aus App/Serial |
| `Pendel_bldc/wake.h` | schlafendes Pendel wecken |
| `Pendel_bldc/trim.h` | Balancierpunkt-Abgleich speichern |
| `Pendel_bldc/telemetry.h` | Telemetrie an die App |
| `Pendel_bldc/ui_task.h` | UI-Task (Ereignisse, Meldungen) |
| `Pendel_bldc/foc_setup.h` | Motor, Encoder, Strommessung einrichten |
| `Pendel_bldc/homing_step.h` | Homing-Schritt in `loop()` |
| `Pendel_bldc/current_test.h` | Strom-Test (`I`) |
| `Pendel_bldc/encoder_tools.h` | Encoder-Test (`S`), MT6835-Selbstkalibrierung (`K`) |
| `PendelSender/PendelSender.ino` | Sender |
| `PendelSender/status_led.h` | WS2812-Statusfarben |
| `WebApp/index.html` | Web-App |

### 3.1 Architektur Wagen

| Ort | Aufgabe |
|---|---|
| Core 1, `loop()` | `loopFOC()` + `move()` so schnell wie möglich (gemessen ca. 18–23 kHz), Homing |
| Core 1, Task „ctrl“ | **fester 800-Hz-Takt per Timer**: neuestes IMU-Paket → EKF → Regler → Sollbeschleunigung → Wagen-Servo → Iq-Sollwert |
| Core 0, Task „ui“ | Befehle (Serial/BLE), Log, Telemetrie, OTA |

Der feste Takt ist wichtig. Früher lief pro Paket ein Schritt, und nach Funklücken wurden die verpassten Schritte auf einmal nachgeholt. Das gab Rucke und scheinbare Winkelsprünge.

### 3.2 Regelung

- **Winkelschätzung (EKF):** Zustände Winkel, Winkelgeschwindigkeit und Gyro-Bias. Das Messmodell nimmt **nicht** an, dass das Pendel frei schwingt, es gilt also auch bei Hand oder Stoß. Die Latenz wird durch Vorausrechnen ausgeglichen. Eine einzelne Beschleunigungsmessung darf den Winkel höchstens um 0,2° verschieben, damit nach Funklücken keine Sprünge entstehen.
- **Balancieren:** LQR auf [x, v, θ, ω] mit der Wagenbeschleunigung als Stellgröße. ω läuft über einen 25-Hz-Tiefpass (`OM_LP_HZ`). Darunter liegt ein Wagen-Servo (Vorsteuerung + PD, ωₙ = 40 rad/s).
- **Sanfter Start aus WARTET:** Die Verstärkung läuft in 1 s von 50 % auf 100 % hoch.
- **Balancierpunkt-Abgleich (Trim):** gleicht einen konstanten Winkelfehler mit τ = 10 s aus und wird im NVS gespeichert.
- **Swing-up:** Energieregelung mit `SU_AMAX` = 5 m/s² und `SU_VMAX` = 1,0 m/s. Ist das Pendel oben, übernimmt der LQR.
- **Beruhigen:** Der Wagen entzieht dem hängenden Pendel aktiv Energie (wirkt wie Reibung im Drehpunkt) und zentriert sich langsam.

### 3.3 Modi

| Nr. | Modus | Bedeutung |
|---|---|---|
| 0 | IDLE | Motor aus |
| 1 | HOLD | Wagen hält die Soll-Mitte |
| 2 | SWINGUP | Pendel aufschwingen |
| 3 | BALANCE | Balancieren |
| 4 | STOPPING | kontrolliert bremsen (Grund steht im Log) |
| 5 | FAULT | Fehler, Motor aus (Grund steht im Log) |
| 6 | PERIOD | Pendelperiode messen |
| 7 | WARTET | wartet, bis das Pendel von Hand senkrecht steht |
| 8 | BERUHIGEN | hängendes Pendel aktiv dämpfen |

---

## 4. Bedienung

### 4.1 Web-App

`index.html` in Chrome bzw. Edge öffnen (Web Bluetooth) und **Verbinden** drücken. Danach verbindet sich die App beim Laden automatisch mit dem zuletzt genutzten Gerät. Mit `?demo` an der URL läuft sie mit simulierten Daten.

| Bereich | Inhalt |
|---|---|
| Kopfzeile | Status-Chips: Verbindung, Modus, Homing, Kalibrierung, Schätzer, Pendel-Funk, Akku, Telemetrie-Rate |
| Pendel-Grafik | Wagen und Pendel, Werte x, θ, a, Iq |
| Verlauf | Plot von x, θ, a, Iq; Zeitfenster; Pause; **CSV**-Export |
| Soll-Mitte | Schieberegler ±300 mm, Button „Mitte“ |
| Betrieb (immer offen) | STOPP, Homing, Halten, Swing-up, Balancieren, Beruhigen, Nullpunkt |
| Einrichten & Diagnose (eingeklappt) | Kalibrieren, Pendel vermessen, Zeitlupe, Hand-Modus, Rekorder, Status, Funk-Test, Encoder-Test, Endstufen-Test, Encoder kalibrieren, Abgleich zurücksetzen, Strom-Test, FOC neu, Neustart |
| Firmware-Update (eingeklappt) | OTA-Modus, .bin auswählen, Hochladen mit Fortschritt |
| Log | Meldungen des Wagens, Eingabefeld für Befehle, Speichern, Leeren |

**Leertaste = STOPP.** Bei laufendem Motor sendet der Wagen die Telemetrie mit 10 Hz, im Ruhezustand mit 50 Hz. Das schont den Funk.

### 4.2 Befehle (ein Zeichen)

| Befehl | Funktion |
|---|---|
| `H` | Homing (sensorlos: beide Anschläge über den Strom suchen, Mitte setzen) |
| `h` | Halten: zur Soll-Mitte fahren und dort halten |
| `s` | Swing-up, danach Balancieren (fällt das Pendel, schwingt er neu auf) |
| `b` | Balancieren: startet, wenn das Pendel 0,3 s lang senkrecht gehalten wird |
| `G` | Beruhigen: hängendes Pendel dämpfen, danach HALTEN |
| `x` | STOPP (bricht auch Homing, Wecken und Encoder-Kalibrierung ab) |
| `0` | Nullpunkt: aktuelle Position = Mitte (ohne Homing) |
| `c` | Pendel kalibrieren (5 s ruhig hängen lassen, Ergebnis ins NVS) |
| `p` | Pendel vermessen (10–20° auslenken, loslassen, dann `p`) → ℓ und Dämpfung |
| `Z` | Zeitlupe: Richtungstest, Wagen fährt langsam zur geneigten Spitze |
| `N` | Hand-Modus an/aus (nur Winkelregelung, keine Positionsregelung) |
| `T` | Balancierpunkt-Abgleich zurücksetzen |
| `D` | Rekorder ausgeben (letzte 2 s mit 800 Hz als CSV) |
| `L` | Funk-Test: keine Telemetrie bei laufendem Motor |
| `S` | Encoder-Test (MT6835 roh, CRC, Register; dabei Motor von Hand drehen) |
| `E` | Endstufen-Test (alle Phasen 8 s auf 6 V, dann 8 s auf 0 V; ohne Motor messen) |
| `K` | MT6835-Selbstkalibrierung (**Riemen ab!**) |
| `I` | Strom-Test (nur mit `USE_CURRENT_SENSE 1`) |
| `f` | FOC-Ausrichtung und R/L-Messung beim nächsten Start neu |
| `R` | Neustart |
| `O` | OTA-Modus |
| `?` | Status ins Log |
| `m` | Monitor an/aus (nur seriell) |
| `F` | Homing über beide Anschläge (lernt die Riemenmarke neu) |
| `M` | Rastmoment lernen (Wagen fährt ~80 mm langsam vor und zurück) |
| `U` | LED-Test |

### 4.3 Befehle mit Wert (ins Befehlsfeld tippen)

| Befehl | Funktion | Beispiel |
|---|---|---|
| `X<mm>` | Soll-Mitte (wie Schieberegler) | `X-50` |
| `Q<qx>,<qv>,<qθ>,<qω>,<R>` | LQR-Gewichte setzen und speichern | `Q2000,50,400,10,2` |
| `Q0` | LQR-Gewichte auf Standard aus `config.h` | `Q0` |
| `Y<mm>` | Ersatzlänge ℓ von Hand setzen und speichern | `Y210` |
| `W<0,01°>` | Winkel-Offset zur Kalibrierung, ±3° (wird gespeichert) | `W0` |

### 4.4 Typischer Ablauf

1. **Einschalten.** Das Pendel verbindet sich, im Log steht „Pendel verbunden“.
2. **Homing (`H`).** Der Wagen sucht beide Anschläge und fährt zur Mitte.
3. **Einmalig bzw. nach Umbau:** kalibrieren mit `c` (Pendel hängt ruhig) und vermessen mit `p`. Plausibel ist ℓ ≈ 0,21 m.
4. **Starten:**

    - **Swing-up (`s`):** Das Pendel schwingt in 2–4 s auf, dann balanciert der Wagen.
    - **Balancieren (`b`):** Pendel locker unterhalb der Elektronik senkrecht halten und loslassen, sobald BALANCE angezeigt wird.

5. **Beenden:** STOPP, oder **Beruhigen (`G`)**, damit das hängende Pendel schnell zur Ruhe kommt.

Schläft der Sender, wenn du `s`, `b`, `G`, `Z` oder `p` drückst, **weckt ihn der Wagen mit einem kurzen Ruck** (ca. 6 mm, 0,16 s). Er wartet dann auf die Daten und führt den Befehl danach aus.

---

## 5. Sicherheit und Grenzen

| Grenze | Wert | Reaktion |
|---|---|---|
| Notstopp | Verfahrweg/2 − 6 cm (ca. ±37 cm) | FAULT „Endlage überschritten“ |
| Arbeitsbereich | Notstopp − 8 cm (ca. ±29 cm) | Swing-up und Soll-Mitte bleiben innerhalb |
| Bremsweg-Überwachung | alle Fahrmodi | bremst, sobald der Bremsweg mit 70 % von `A_MAX` nicht mehr bis zum Notstopp reicht |
| Pendel verloren | 20° beim Balancieren | STOPPING (bei `s`: neu aufschwingen) |
| Funkverbindung | 150 ms ohne Daten | FAULT „Funkverbindung verloren“ |
| Beschleunigung | 15 m/s² (`A_MAX`) | unter der Stromgrenze von 12 A |
| Geschwindigkeit | 1,5 m/s (`V_MAX`), Swing-up 1,0 m/s | – |
| Rückfahrt | HALTEN/WARTET dürfen bis 5 cm hinter dem Notstopp zur Mitte zurückfahren | – |

Nach jedem Moduswechsel aus BALANCE steht die Funkstatistik im Log: längste Lücke und Anzahl der Lücken ≥ 50 ms.

---

## 6. Pendel-Sender

### 6.1 Statusanzeige WS2812

| Farbe | Bedeutung |
|---|---|
| weiß | Start / Initialisierung |
| grün, kurzer Blitz alle 2 s | alles ok |
| blau, langsam blinkend | kein Heartbeat vom Wagen |
| cyan, dauerhaft | pausiert (Wagen im OTA-Modus) |
| orange, langsam blinkend | Akku schwach (< 3,45 V) |
| rot, langsam blinkend | BMI270 liefert keine Daten |
| rot, schnell blinkend | BMI270-Init fehlgeschlagen |
| magenta, schnell blinkend | ESP-NOW-Init fehlgeschlagen |
| 3 × blau | geht in den Ruhe-Tiefschlaf |
| 3 × rot | Akku kritisch → Tiefschlaf |

Die einfache LED an GPIO15 blitzt einmal pro Sekunde, solange gesendet wird.

### 6.2 Tiefschlaf

| | Ruhe-Tiefschlaf | Akku-Tiefschlaf |
|---|---|---|
| Auslöser | 60 s keine Bewegung (Lage < 2°, Drehrate < 8°/s) | Akku < 3,30 V für 5 s |
| Nicht, wenn | das Pendel aufrecht steht, oder der Wagen in den letzten 30 s „aktiv“ gemeldet hat | – |
| Im Schlaf | Funk aus, nur Beschleunigungssensor im Low-Power-Modus mit 50 Hz | BMI270 aus |
| Aufwachen | Bewegung (Any-Motion ca. 2,8° bzw. Anstoßen) oder Ruck durch den Wagen | Timer alle 10 min, startet erst ab 3,70 V |

Geschätzter Ruhestrom: 0,1 mA ohne, bis ca. 1 mA mit WS2812-Ruhestrom. Messen in der Plusleitung des Akkus.

---

## 7. Funk

| | Wert |
|---|---|
| Verfahren | ESP-NOW Broadcast ohne ACK, 12 Mbit/s, 800 Pakete/s |
| Kanal | **13** (`ESPNOW_CHANNEL` in beiden `protocol.h`, Länderkennung „DE“) |
| Sendeleistung Pendel | 15 dBm |
| Heartbeat Wagen → Pendel | alle 0,5 s, mit „aktiv“-Flag |

Auf Kanal 1 gab es Lücken von über 200 ms, weil die FRITZ!Box (1–7) im selben Band funkt. Seit Kanal 13: längste Lücke 8 ms. Die FRITZ!Box am besten fest auf Kanal 1 stellen.

---

## 8. OTA (nur Wagen)

1. Den Block **Firmware-Update** öffnen und **OTA-Modus aktivieren**. Der Motor geht aus, das Pendel pausiert.
2. Der Wagen startet das WLAN neu und verbindet sich per DHCP mit dem Router aus `secrets.h`. Die Adresse übernimmt die App automatisch.
3. Ist der Router nicht erreichbar, startet der Notfall-AP `pendel-wagen` / `pendel123` auf Kanal 1 (Adresse 192.168.4.1).
4. Die `.bin` auswählen (Arduino: *Sketch → Kompilierte Binärdatei exportieren*) und hochladen. Danach startet der Wagen neu.

Ohne Upload startet der Wagen nach 5 min von selbst neu.

---

## 9. Diagnose

| Werkzeug | Zweck |
|---|---|
| **Rekorder** (`D`) | letzte 2 s mit 800 Hz: x, v, θ, ω, Gyro, fr, ft, a, Iq, Modus, Paketverlust, Winkelkorrektur, FOC-Durchläufe. Friert 0,3 s nach dem Ende von BALANCE, bei FAULT oder bei Winkelsprüngen ein und bleibt bis zum nächsten echten Balancieren bzw. Swing-up erhalten. Übersteht Absturz, Watchdog und EN-Reset. |
| Log nach Neustart | „Neustart-Grund“, außerdem das Ende des Logs **vor** dem Neustart („=== Log VOR dem Neustart ===“) |
| Neustart-Grund „Einschalten“ **ohne** altes Log | Versorgung war wirklich weg (Netzteil, Stecker, Lötstellen) |
| Neustart-Grund „Einschalten“ **mit** altem Log | Reset über den EN-Pin (Störung) |
| Neustart-Grund „ABSTURZ“ / „WATCHDOG“ | Firmwarefehler → Rekorder und Log schicken |
| Zeitlupe (`Z`) | prüft die Vorzeichen: Spitze nach +x geneigt → Wagen fährt nach +x |
| Endstufen-Test (`E`) | **ohne Motor** messen: jede Phase 6 V, dann 0 V |
| Encoder-Test (`S`) | Rohwerte, CRC, Register des MT6835 |
| Funk-Test (`L`) | Telemetrie bei laufendem Motor abschalten, Funklücken vergleichen |
| `?` | Status: Pakete, Verluste, Kalibrierung, ℓ, K, LQR-Gewichte, Offset, Abgleich, FOC-Rate |

### 9.1 Behobene Fehler (Erfahrungen)

| Symptom | Ursache | Lösung |
|---|---|---|
| Wagen fährt in die falsche Richtung | Vorzeichen der IMU-Achsen | Zeitlupe-Test, `AX_T_SIGN = GYR_SIGN = −1` |
| Schwingung 10–20 Hz beim Balancieren | Vibrationen der Spitze (bis 8 g) im Sensor | BMI270 auf OSR4, 25-Hz-Tiefpass auf ω, `LQR_R` = 2 |
| Winkelsprünge, Rucke | verpasste Regelschritte wurden nach Funklücken nachgeholt | fester 800-Hz-Takt |
| Abbrüche durch Funklücken | Kollision mit dem WLAN auf Kanal 1 | ESP-NOW auf Kanal 13 |
| Neustarts mitten im Betrieb | defektes Netzteil, kalte Lötstellen | Netzteil getauscht, Lötstellen nachgelötet |
| Motor ohne Kraft | kalte Lötstellen an der Endstufe | nachgelötet, Endstufen-Test ohne Motor |
| Swing-up schafft es nicht nach oben | gespeichertes ℓ = 0,339 m (falsch) | ℓ = 0,21 m, Plausibilitätsprüfung |
| Absturz beim Swing-up | 4-A-Netzteil überlastet | Swing-up sanfter, Netzteil 10 A |
| Wagen fährt in den Anschlag | Stopp zu spät bei hoher Geschwindigkeit | größere Abstände, Bremsweg-Überwachung |
| Wagen pendelt ±3–6 cm | ungenaue Kraft bei kleinen Strömen (geschätzter Strom) | 2-mΩ-Shunts, echte Stromregelung → ca. 1 mm |

---

## 10. Feineinstellung

### 10.1 Ruhe beim Balancieren

**Stand 04.11.2026 mit Strommessung:** Nach dem Einschwingen (15–20 s) bewegt sich der Wagen nur noch etwa **1 mm** (Spitze–Spitze). Der Winkel streut um 0,3–0,6°, der Motorstrom liegt im Mittel bei etwa 0,4 A.

| Entwicklung | Wagenbewegung | Ursache / Maßnahme |
|---|---|---|
| geschätzter Strom, Funk auf Kanal 1 | ±4,5 cm, 0,4 Hz | Funklücken und Störungen regen die langsame Eigenschwingung an |
| geschätzter Strom, Kanal 13 | ±3–6 cm, Periode ca. 13 s | bei kleinen Strömen kommt wegen der Endstufen-Totzeit ungenau Kraft an, dazu Haftreibung ca. 2 N |
| **echte Stromregelung (INA240, 1,82-mΩ-Shunts)** | **ca. 1 mm** | der Motor liefert auch kleine Kräfte genau |

Weitere Stellschrauben, falls nötig:

| Maßnahme | Wert / Befehl | Wirkung |
|---|---|---|
| Einschwingen verkürzen | `TRIM_TAU_S` z. B. 5 s | der Balancierpunkt-Abgleich findet schneller seinen Wert |
| Positionsgewicht erhöhen | `Q2000,50,400,10,2` | weniger Versatz pro Grad Winkelfehler (3,7 statt 6,1 cm) |
| Standard | `Q0` | – |
| Haftreibung ausgleichen | `F_COULOMB_N` in `config.h` | mit Strommessung meist nicht mehr nötig |

### 10.2 Wichtige Parameter in `config.h`

| Parameter | Wert | Bedeutung |
|---|---|---|
| `LQR_QX, QV, QTH, QOM, R` | 400, 10, 400, 10, 2 | LQR-Gewichte (zur Laufzeit mit `Q`) |
| `A_MAX`, `V_MAX` | 15 m/s², 1,5 m/s | Grenzen beim Balancieren |
| `SU_AMAX`, `SU_VMAX` | 5 m/s², 1,0 m/s | Swing-up (mit 10-A-Netzteil ggf. höher) |
| `OM_LP_HZ` | 25 Hz | Tiefpass auf ω |
| `TRIM_TAU_S` | 10 s | Balancierpunkt-Abgleich, 0 = aus |
| `MAX_LOST` | 120 (150 ms) | Funk-Abbruchgrenze |
| `BAL_LOST_TH` | 0,35 rad (20°) | Pendel verloren |
| `HOME_SPEED`, `HOME_I_MAX`, `HOME_I_STALL` | 0,20 m/s, 1,0 A, 0,2 A | Homing |
| `HOME_MARGIN`, `HOME_SOFT_GAP` | 6 cm, 8 cm | Sicherheitsabstände |
| `PENDULUM_ELL_DEFAULT_M` | 0,21 m | Ersatzlänge |
| `F_COULOMB_N` | 0 | Haftreibungs-Vorsteuerung |

---

## 11. LED-Leiste und Riemenmarke (optional)

### LED-Leiste (Anzeige)

Eine WS2812-2020-Leiste (144 LEDs/m, 1 m = ganze Schiene) liegt parallel zur Schiene und zeigt Wagen und Modus live an (`#define LED_STRIP`).

| | |
|---|---|
| Anschluss | Daten GPIO0 (330 Ω in Reihe oder Pegelwandler; Strapping-Pin: Leiste beim Start mit Strom versorgen), 5 V aus eigenem Wandler mit 1000 µF, gemeinsame Masse |
| Länge / Anzahl | `LED_PER_M`, `LED_STRIP_M` (Anzahl = Länge × LED/m). Mitte der Leiste = Mitte des Verfahrwegs, Richtung `LED_DIR`, Feinjustage `LED_OFS_M` |
| Anzeige | Wagen als 3 LEDs, Farbe nach Modus: weiß IDLE, cyan HALTEN, gelb blinkend WARTET, orange Swing-up, **grün → rot mit dem Pendelwinkel (0–10°) beim Balancieren, blau im Totpunkt (Ruhezone)**, violett Beruhigen, rot STOPP/FAULT. Arbeitsbereich-Enden rot. Helligkeit `LED_SHOW_LEVEL` |
| LED-Test (`U`) | bunter Lauflicht-Test hin und zurück (`LED_RUN_S` je Richtung) |
| Ruhe-Show | Wagen steht (IDLE/HALTEN, gehomt), Pendel hängt ruhig (< 5°, < 0,15 rad/s): alle `LED_IDLE_SHOW_S` s ein Lauflicht vom Wagen zu beiden Enden und zurück, je Durchlauf zufällige harmonische Palette (analog, komplementär, triadisch). `LED_IDLE_SPEED` LEDs/s, Helligkeit `LED_IDLE_LEVEL`, 0 s = aus |

### Riemenmarke für die Mittensuche

Ein TCS34725 sitzt nahe der Motorwelle und sieht die glatte Rückseite des Riemens (`#define BELT_SENSOR`). Der Riemen ist fest mit dem Wagen verbunden, deshalb entspricht die Riemenstelle am Sensor eindeutig der Wagenposition. Ein Stück des Riemens (gut ein halber Verfahrweg, ca. 50 cm) ist hell markiert, der Rest bleibt dunkel. Die Hell-Dunkel-Grenze ist die Referenzkante.

| | |
|---|---|
| Anschluss | SDA 21, SCL 22, 3,3 V. Die Modul-LED bleibt immer an (LED-Pin offen bzw. an 3,3 V, ~10–20 mA, aus dem 3,3-V-Regler) |
| Markierung | matt hell (Gewebeband, matter Lack), scharfe Kante; glänzende Farbe vermeiden |
| Fremdlicht | schwarzes Gehäuse, Riemen läuft durch einen Schlitz, Sensor 3–5 mm vom Riemen; Raumlicht flackert mit 100 Hz und verschiebt Hell- und Dunkelwert. Bei geändertem Licht/Gehäuse einmal `F` |
| Einlernen (`F`) | Homing über beide Anschläge: Hell-/Dunkelwert an den Anschlägen → Schwelle (Mitte, ±8 % Hysterese), helle Seite, danach wird die Kante vermessen. Mindestkontrast `BELT_MIN_CONTRAST` |
| Mittensuche (`H`, `HOME_ON_START`) | Marke lesen → mit `CSW_FAST` zur Kante → `CSW_BACK` zurück ins Helle → mit `CSW_SLOW` von hell nach dunkel an die Kante (immer gleich) → Mitte und Grenzen setzen. Keine Anschläge |
| Plausibilität | im Betrieb passt die Marke nicht zur Encoder-Position (5 mm Toleranz) → Warnung „Riemen übersprungen?“ |
| Sensor-Abtastung | eigener Task, alle ~2–3 ms, Integrationszeit 2,4 ms (`BELT_ATIME`, `BELT_GAIN`) |
| Status | `?` zeigt Rohwert, Schwelle, hell/dunkel und ob eingelernt |

Zum Kalibrieren des MT6835 (`K`) den Sensor abziehen und CAL_EN an GPIO22 stecken; die Firmware beendet dafür den I2C-Bus.

Verworfene Varianten: ToF-Sensor (Messkegel erfasst Schiene und Umgebung), LED-Leiste + Spiegel + Lichtsensor und Schleifkontakt auf Kupferband (mechanisch aufwendig bzw. hemmend).

## 12. MT6835-Selbstkalibrierung (optional)

Voraussetzungen laut Datenblatt:

- Der Chip wird ausschließlich über **CAL_EN (Pin 4)** gestartet. Einen SPI-Befehl dafür gibt es nicht.
- Der Encoder muss sich **mehr als 64 Umdrehungen gleichmäßig** drehen. Dafür muss der Riemen ab.

Ablauf mit `K`:

1. Der Motor dreht gesteuert mit 150 U/min.
2. CAL_EN (GPIO22) geht automatisch auf HIGH.
3. Der Status wird mitgeloggt.
4. Bei „ERFOLGREICH“ **mehr als 6 s warten, dann die Versorgung aus- und wieder einschalten.** Ein Neustart mit `R` reicht nicht.
5. Beim nächsten Start richtet sich der Motor neu aus.

CAL_EN nicht dauerhaft auf 3,3 V lassen.

---

## 13. Ausblick

- **Doppel-Pendel:** grundsätzlich machbar. Dafür nötig: zweiter identischer Sender, LQR mit 6 Zuständen, erweitertes Messmodell für das obere Glied, stärkeres Netzteil. Begrenzend sind eher Strom und Schienenlänge als die Versorgungsspannung.
- **Versorgungsspannung:** 12 V beibehalten (MOSFETs 30 V). Bei 20 V gibt es keinen TVS-Schutz mehr, der sicher unter 30 V begrenzt.
