// led_pos.h – LED-Leiste (WS2812 an PIN_LED_STRIP) + TCS34725: absolute Wagenposition und Live-Anzeige
// Wird in CartController.ino eingebunden (nach logf/ctrl/prefs). Nur mit #define LED_POS.
#pragma once

// ---------------- WS2812 über RMT (10 MHz, ohne Zusatzbibliothek) ----------------
static rmt_data_t ledRmt[LED_COUNT * 24];
static uint8_t ledBuf[LED_COUNT][3];            // R, G, B
static bool ledInitOk = false;

static void ledClear() { memset(ledBuf, 0, sizeof(ledBuf)); }
static void ledSet(int i, uint8_t r, uint8_t g, uint8_t b) {
  if (i < 0 || i >= LED_COUNT) return;
  ledBuf[i][0] = r; ledBuf[i][1] = g; ledBuf[i][2] = b;
}
static void ledShow() {
  if (!ledInitOk) return;
  int k = 0;
  for (int i = 0; i < LED_COUNT; i++) {
    const uint8_t grb[3] = {ledBuf[i][1], ledBuf[i][0], ledBuf[i][2]};
    for (int c = 0; c < 3; c++)
      for (int bit = 7; bit >= 0; bit--) {
        const bool one = (grb[c] >> bit) & 1;
        ledRmt[k].level0 = 1; ledRmt[k].duration0 = one ? 8 : 4;
        ledRmt[k].level1 = 0; ledRmt[k].duration1 = one ? 4 : 8;
        k++;
      }
  }
  rmtWrite(PIN_LED_STRIP, ledRmt, k, 50);
  delayMicroseconds(100);                       // Reset > 80 us
}

// ---------------- TCS34725 (Register direkt über Wire) ----------------
static constexpr uint8_t TCS_ADDR = 0x29;
static void tcsWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | reg); Wire.write(v); Wire.endTransmission();
}
static uint8_t tcsRead8(uint8_t reg) {
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | reg); Wire.endTransmission();
  Wire.requestFrom(TCS_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static uint16_t tcsClear() {                    // CDATA (Summe aller Farben)
  Wire.beginTransmission(TCS_ADDR); Wire.write(0x80 | 0x20 | 0x14); Wire.endTransmission();
  Wire.requestFrom(TCS_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  const uint8_t lo = Wire.read(), hi = Wire.read();
  return (uint16_t)(lo | (hi << 8));
}
static const uint32_t tcsIntegMs = (uint32_t)((256 - TCS_ATIME) * 2.4f) + 1;
static const uint16_t tcsMax = (uint16_t)((256 - TCS_ATIME) * 1024 > 65535 ? 65535 : (256 - TCS_ATIME) * 1024);

// LEDs setzen und nach zwei vollen Integrationszyklen messen (erster Zyklus kann das alte Muster enthalten)
static int32_t tcsFrame() {
  ledShow();
  delay(2 * tcsIntegMs + 3);
  return tcsClear();
}

static void posSetup() {
  ledInitOk = rmtInit(PIN_LED_STRIP, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  if (!ledInitOk) logf("LED-Leiste: RMT-Init an GPIO%d fehlgeschlagen", PIN_LED_STRIP);
  ledClear(); ledShow();
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  const uint8_t id = tcsRead8(0x12);
  if (id != 0x44 && id != 0x4D && id != 0x10) { logf("TCS34725: nicht gefunden (ID 0x%02X, SDA %d / SCL %d)", id, PIN_SDA, PIN_SCL); return; }
  tcsWrite(0x01, TCS_ATIME);
  tcsWrite(0x0F, TCS_GAIN & 3);
  tcsWrite(0x00, 0x01); delay(3);               // PON
  tcsWrite(0x00, 0x03);                         // PON + AEN
  posOk = ledInitOk;
  logf("LED-Position: TCS34725 bereit (ID 0x%02X), %d LEDs an GPIO%d, %s", id, LED_COUNT, PIN_LED_STRIP,
#ifdef LED_POS_GRAY
       "Gray-Code"
#else
       "Lauflicht"
#endif
  );
}

static void ledAll(uint8_t v) { for (int i = 0; i < LED_COUNT; i++) ledSet(i, v, v, v); }

// Position der LED vor dem Spiegel als "Abstand" d = Index * LED_PITCH_M [m]
static bool posMeasure(float& d, float& spread) {
  if (!posOk) return false;
  const uint8_t L = LED_MEAS_LEVEL;
  ledClear();               const int32_t c0 = tcsFrame();   // Fremdlicht
  ledAll(L);                const int32_t c1 = tcsFrame();   // alle an
  const int32_t full = c1 - c0;
  if (c1 >= (int32_t)(0.9f * tcsMax)) { logf(">> LED-Position: Sensor uebersteuert (%ld) - LED_MEAS_LEVEL/TCS_GAIN senken", (long)c1); ledClear(); ledShow(); return false; }
  if (full < (int32_t)LED_MIN_SIGNAL) { logf(">> LED-Position: kein Spiegel-Signal (%ld Counts) - Ausrichtung pruefen", (long)full); ledClear(); ledShow(); return false; }
  int32_t sig[LED_COUNT];
  for (int i = 0; i < LED_COUNT; i++) sig[i] = INT32_MIN;
  int best = 0;
#ifdef LED_POS_GRAY
  int bits = 0;
  while ((1 << bits) < LED_COUNT) bits++;
  uint32_t g = 0;
  for (int b = 0; b < bits; b++) {
    ledClear();
    for (int i = 0; i < LED_COUNT; i++) if (((i ^ (i >> 1)) >> b) & 1) ledSet(i, L, L, L);
    const int32_t cb = tcsFrame() - c0;
    if (cb > full / 2) g |= (1u << b);
  }
  uint32_t idx = g;
  for (uint32_t sh = 1; sh < 32; sh <<= 1) idx ^= idx >> sh;   // Gray -> binär
  best = idx < (uint32_t)LED_COUNT ? (int)idx : LED_COUNT - 1;
  for (int i = best - 2; i <= best + 2; i++) {                  // Nachbarn einzeln messen
    if (i < 0 || i >= LED_COUNT) continue;
    ledClear(); ledSet(i, L, L, L);
    sig[i] = tcsFrame() - c0;
  }
#else
  for (int i = 0; i < LED_COUNT; i++) {                          // Lauflicht
    ledClear(); ledSet(i, L, L, L);
    sig[i] = tcsFrame() - c0;
  }
#endif
  ledClear(); ledShow();
  for (int i = 0; i < LED_COUNT; i++) if (sig[i] != INT32_MIN && (sig[best] == INT32_MIN || sig[i] > sig[best])) best = i;
  if (sig[best] < full / 8) { logf(">> LED-Position: Einzel-LED zu schwach (%ld von %ld)", (long)sig[best], (long)full); return false; }
  // Schwerpunkt der Nachbarn (Spiegel deckt 2-3 LEDs ab) -> Bruchteile des LED-Abstands
  double sw = 0, swi = 0;
  for (int i = best - 2; i <= best + 2; i++) {
    if (i < 0 || i >= LED_COUNT || sig[i] == INT32_MIN) continue;
    const double w = sig[i] > sig[best] / 4 ? (double)sig[i] : 0.0;
    sw += w; swi += w * i;
  }
  const float idxF = sw > 0 ? (float)(swi / sw) : (float)best;
  d = idxF * LED_PITCH_M;
  spread = 0.5f * LED_PITCH_M;
  logf(">> LED-Position: LED %.2f (Signal %ld von %ld)", idxF, (long)sig[best], (long)full);
  return true;
}

// ---------------- Live-Anzeige ----------------
// Wagen als 3 LEDs, Farbe nach Modus; beim Balancieren grün -> rot mit dem Pendelwinkel; Arbeitsbereich-Enden rot
static void ledShowService() {
  static uint32_t last = 0;
  if (!ledInitOk || LED_SHOW_LEVEL == 0 || tofTeachState) return;
  const uint32_t now = millis();
  if (now - last < 40) return;                                    // 25 Hz
  last = now;
  const float S = LED_SHOW_LEVEL / 255.0f;
  auto col = [&](float r, float g, float b, int i) { ledSet(i, (uint8_t)(r * S * 255), (uint8_t)(g * S * 255), (uint8_t)(b * S * 255)); };
  ledClear();
  const int8_t sg = prefs.getChar("tof_sg", 0);
  const float d0 = prefs.getFloat("tof_d0", 0);
  const bool mapped = sg != 0 && homed;
  auto idxOf = [&](float x) { return (int)lroundf((d0 + sg * x) / LED_PITCH_M); };
  const bool blink = (now / 300) & 1;
  float r = 0.3f, g = 0.3f, b = 0.3f;                             // IDLE: weiß gedimmt
  switch (ctrl.mode) {
    case pc::Mode::HOLD: r = 0; g = 0.6f; b = 0.6f; break;
    case pc::Mode::ARMED: r = blink ? 1 : 0; g = blink ? 0.8f : 0; b = 0; break;
    case pc::Mode::SWINGUP: r = 1; g = 0.4f; b = 0; break;
    case pc::Mode::BALANCE: {
      const float e = fminf(1.0f, fabsf(pc::wrapPi(ctrl.th_c)) / (10.0f * DEG_TO_RAD));   // 0..10°
      r = e; g = 1.0f - e; b = 0; break;
    }
    case pc::Mode::DAMP: r = 0.6f; g = 0; b = 1; break;
    case pc::Mode::STOPPING: r = 1; g = 0; b = 0; break;
    case pc::Mode::FAULT: r = blink ? 1 : 0; g = 0; b = 0; break;
    case pc::Mode::PERIOD: r = 0; g = 0; b = 1; break;
    default: break;
  }
  if (mapped) {
    const int c = idxOf(tx_x);
    for (int i = c - 1; i <= c + 1; i++) col(r, g, b, i);
    col(0.4f, 0, 0, idxOf(-ctrl.cfg.x_soft));
    col(0.4f, 0, 0, idxOf(ctrl.cfg.x_soft));
  } else {
    col(r, g, b, 0);                                             // ohne Zuordnung: nur Modusfarbe an LED 0
  }
  ledShow();
}
