// belt_sensor.h – Riemenmarke für die Mittensuche: TCS34725 sieht die glatte Riemenrückseite (#define BELT_SENSOR)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Der Riemen ist fest mit dem Wagen verbunden: welche Riemenstelle am Sensor vorbeiläuft, hängt nur von der
// Wagenposition ab. Ein Riemenstück ist hell markiert -> "hell" = Wagen auf der einen Seite der Kante.
// Ein eigener Task (Kern 0) liest den Sensor alle ~3 ms; centerSw() liefert den entprellten Zustand (Hysterese).
// Die Schwelle wird beim Homing über die Anschläge gelernt (Hell-/Dunkelwert an den beiden Anschlägen).
#pragma once

#ifdef BELT_SENSOR
static constexpr uint8_t TCS_ADDR = 0x29;
static volatile uint16_t beltClear = 0;         // letzter Rohwert (Summe aller Farben)
static volatile bool beltBright = false;        // Zustand mit Hysterese
static volatile bool beltOk = false;
static volatile uint16_t beltThr = 0;           // gelernte Schwelle (0 = nicht gelernt)
static volatile bool beltRun = false;
static TaskHandle_t beltTaskH = nullptr;

// TCS34725-Register: Befehlsbit 0x80 + Registeradresse; 0x20 = Auto-Inkrement (mehrere Bytes am Stück)
static void tcsWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | reg); Wire.write(v); Wire.endTransmission();
}
static uint8_t tcsRead8(uint8_t reg) {
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | reg); Wire.endTransmission();
  Wire.requestFrom(TCS_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static bool tcsClear(uint16_t& c) {             // CDATA, nur wenn ein neuer Wert vorliegt (AVALID)
  if (!(tcsRead8(0x13) & 0x01)) return false;
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | 0x20 | 0x14); Wire.endTransmission();
  Wire.requestFrom(TCS_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return false;
  const uint8_t lo = Wire.read(), hi = Wire.read();
  c = (uint16_t)(lo | (hi << 8));
  return true;
}

static void beltTask(void*) {
  for (;;) {
    if (beltRun) {
      uint16_t c;
      if (tcsClear(c)) {
        beltClear = c;
        const uint16_t t = beltThr;
        if (t) {                                  // ±8 % Hysterese um die Schwelle
          if (!beltBright && c > t + t / 12) beltBright = true;
          else if (beltBright && c < t - t / 12) beltBright = false;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void beltSetup() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  const uint8_t id = tcsRead8(0x12);
  if (id != 0x44 && id != 0x4D && id != 0x10) {
    logf("Riemenmarke: TCS34725 nicht gefunden (ID 0x%02X, SDA %d / SCL %d) - Homing ueber die Anschlaege", id, PIN_SDA, PIN_SCL);
    return;
  }
  // ATIME 0x01: Integrationszeit (0xFF = 2,4 ms, kurz für schnelles Abtasten); 0x0F: Verstärkung 1/4/16/60×;
  // 0x00 ENABLE: erst PON (Oszillator an, 3 ms warten), dann zusätzlich AEN (Messung starten)
  tcsWrite(0x01, BELT_ATIME);
  tcsWrite(0x0F, BELT_GAIN & 3);
  tcsWrite(0x00, 0x01); delay(3);                // PON
  tcsWrite(0x00, 0x03);                          // PON + AEN
  beltThr = prefs.getUShort("bs_thr", 0);
  beltOk = true; beltRun = true;
  xTaskCreatePinnedToCore(beltTask, "belt", 3072, nullptr, 2, &beltTaskH, 0);
  delay(20);
  if (beltThr) beltBright = beltClear > beltThr;
  logf("Riemenmarke: TCS34725 bereit, Wert %u, Schwelle %u -> %s, %s", beltClear, beltThr,
       beltThr ? (beltBright ? "hell" : "dunkel") : "-",
       prefs.getChar("cs_dir", 0) && beltThr ? "eingelernt" : "noch nicht eingelernt (einmal F)");
}
// vor der Encoder-Kalibrierung: I2C freigeben (CAL_EN liegt dann an GPIO22)
static void beltStop() { beltRun = false; delay(10); beltOk = false; Wire.end(); }
#endif

// true = Sensor sieht die helle Riemenmarke (= "sw" im Homing-Zustandsautomaten)
static inline bool centerSw() {
#ifdef BELT_SENSOR
  return beltBright;
#else
  return false;
#endif
}
