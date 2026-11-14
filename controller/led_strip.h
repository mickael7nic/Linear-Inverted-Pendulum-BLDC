// led_strip.h – WS2812-Leiste an PIN_LED_STRIP: Live-Anzeige des Wagens, LED-Test (#define LED_STRIP)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge). Läuft auch ohne Lichtsensor,
#pragma once
#ifdef LED_STRIP

// ---------------- WS2812 über RMT (10 MHz, ohne Zusatzbibliothek) ----------------
static rmt_data_t* ledRmt = nullptr;            // ledCount*24 Einträge, vom Heap (statisch passt bei vielen LEDs nicht ins DRAM-Segment)
static constexpr int ledCount = LED_COUNT;     // = LED_STRIP_M x LED_PER_M
static uint8_t ledBuf[LED_COUNT][3];            // R, G, B
static bool ledInitOk = false;
#define LED_RMT_BLOCKS RMT_MEM_NUM_BLOCKS_4     // 256 Symbole Puffer -> seltener nachladen (robuster bei Flash-Zugriffen)

static void ledClear() { memset(ledBuf, 0, sizeof(ledBuf)); }
static void ledSet(int i, uint8_t r, uint8_t g, uint8_t b) {
  if (i < 0 || i >= ledCount) return;
  ledBuf[i][0] = r; ledBuf[i][1] = g; ledBuf[i][2] = b;
}
static void ledShow() {
  if (!ledInitOk) return;
  int k = 0;
  for (int i = 0; i < ledCount; i++) {
    const uint8_t grb[3] = {ledBuf[i][1], ledBuf[i][0], ledBuf[i][2]};
    for (int c = 0; c < 3; c++)
      for (int bit = 7; bit >= 0; bit--) {
        const bool one = (grb[c] >> bit) & 1;
        ledRmt[k].level0 = 1; ledRmt[k].duration0 = one ? 8 : 4;
        ledRmt[k].level1 = 0; ledRmt[k].duration1 = one ? 4 : 8;
        k++;
      }
  }
  // Bleibt eine Übertragung hängen (TX-Done-Meldung verloren, z. B. Nachladen des RMT-Puffers während eines
  // Flash-Schreibvorgangs verpasst), lehnt rmtWrite() danach jede weitere ab -> Leiste "friert ein".
  // Abhilfe: nach 3 Fehlschlägen in Folge den RMT-Kanal neu anlegen.
  static uint8_t fails = 0;
  static uint16_t resets = 0;
  if (rmtWrite(PIN_LED_STRIP, ledRmt, k, 50)) fails = 0;
  else if (++fails >= 3) {
    fails = 0;
    rmtDeinit(PIN_LED_STRIP);
    ledInitOk = rmtInit(PIN_LED_STRIP, RMT_TX_MODE, LED_RMT_BLOCKS, 10000000);
    if (++resets <= 5 || resets % 100 == 0) logf("   LED-Leiste: Uebertragung haengt - RMT neu gestartet (%u x)", resets);
  }
  delayMicroseconds(100);                       // Reset > 80 us
}

static void ledStripSetup() {
  ledRmt = (rmt_data_t*)malloc(sizeof(rmt_data_t) * ledCount * 24);
  ledInitOk = ledRmt && rmtInit(PIN_LED_STRIP, RMT_TX_MODE, LED_RMT_BLOCKS, 10000000);
  if (!ledInitOk) { logf("LED-Leiste: RMT-Init an GPIO%d fehlgeschlagen", PIN_LED_STRIP); return; }
  ledClear(); ledShow();
  logf("LED-Leiste: %d LEDs (%.0f/m, %.2f m) an GPIO%d", ledCount, LED_PER_M, LED_STRIP_M, PIN_LED_STRIP);
}
static void ledAll(uint8_t v) { for (int i = 0; i < ledCount; i++) ledSet(i, v, v, v); }

// ---------------- LED-Test (ohne Sensor): Lauflicht einmal hin und zurück, Regenbogenfarben ----------------
static bool ledTestOn = false;
static uint32_t ledTestT0 = 0;
static void hsv(float h, float v, uint8_t& r, uint8_t& g, uint8_t& b) {   // h 0..1, v 0..1
  const float H = fmodf(h, 1.0f) * 6.0f, f = H - floorf(H);
  const float q = v * (1 - f), t = v * f;
  float R, G, B;
  switch ((int)H % 6) { case 0: R = v; G = t; B = 0; break; case 1: R = q; G = v; B = 0; break;
                        case 2: R = 0; G = v; B = t; break; case 3: R = 0; G = q; B = v; break;
                        case 4: R = t; G = 0; B = v; break; default: R = v; G = 0; B = q; }
  r = (uint8_t)(R * 255); g = (uint8_t)(G * 255); b = (uint8_t)(B * 255);
}
static void ledTestStart() {
  if (!ledInitOk) { logf(">> LED-Test: LED-Leiste nicht initialisiert (GPIO%d)", PIN_LED_STRIP); return; }
  ledTestOn = true; ledTestT0 = millis();
  logf(">> LED-Test: Lauflicht hin und zurueck, %d LEDs, %.0f s", ledCount, 2 * LED_RUN_S);
}
static void ledTestService() {
  const float T = LED_RUN_S, t = (millis() - ledTestT0) * 1e-3f;
  if (t >= 2 * T) { ledTestOn = false; ledClear(); ledShow(); logf(">> LED-Test: fertig"); return; }
  const bool back = t >= T;
  const float pos = (back ? (2 * T - t) : t) / T * (ledCount - 1);
  const int head = (int)lroundf(pos), dir = back ? -1 : 1;
  ledClear();
  for (int k = 0; k < 8; k++) {                                   // Kopf + ausklingender Schweif
    const int i = head - dir * k;
    if (i < 0 || i >= ledCount) continue;
    uint8_t r, g, b;
    hsv((float)i / ledCount + (back ? 0.5f : 0.0f), 0.5f * powf(0.6f, k), r, g, b);   // Farbe nach Ort, Rückweg versetzt
    ledSet(i, r, g, b);
  }
  ledShow();
}

// ---------------- Zuordnung Wagenposition -> LED-Index ----------------
// Mitte der Leiste = Mitte des Verfahrwegs, Richtung LED_DIR, Feinjustage LED_OFS_M
static int ledIdxOfX(float x) {
  return (int)lroundf((0.5f * LED_STRIP_M + LED_DIR * x + LED_OFS_M) / LED_PITCH_M);
}

// ---------------- Ruhe-Show: Lauflicht vom Wagen zu beiden Enden und zurück ----------------
// Nur wenn der Wagen steht (IDLE/HOLD, gehomt) und das Pendel ruhig hängt; alle LED_IDLE_SHOW_S Sekunden.
// Je Durchlauf eine zufällige, harmonische Farbpalette (analog, komplementär oder triadisch).
static bool idleShowOn = false;
static uint32_t idleShowT0 = 0;
static int idleShowC = 0;                                          // Start-LED (Wagen)
static float idlePal[3];                                           // Farbtöne 0..1: hin links, hin rechts, zurück
static void idleShowStart(int c) {
  const float h = (esp_random() % 1000) * 1e-3f;
  switch (esp_random() % 3) {
    case 0:  idlePal[0] = h; idlePal[1] = h + 0.08f; idlePal[2] = h + 0.16f; break;     // analog
    case 1:  idlePal[0] = h; idlePal[1] = h + 0.06f; idlePal[2] = h + 0.5f;  break;     // komplementär
    default: idlePal[0] = h; idlePal[1] = h + 1.0f / 3; idlePal[2] = h + 2.0f / 3; break; // triadisch
  }
  idleShowOn = true; idleShowT0 = millis(); idleShowC = c;
}
// zeichnet einen Durchlauf über die Live-Anzeige; false = fertig
static bool idleShowDraw() {
  const float t = (millis() - idleShowT0) * 1e-3f, sp = LED_IDLE_SPEED;
  const int c = idleShowC;
  bool busy = false;
  for (int side = -1; side <= 1; side += 2) {
    const int len = side < 0 ? c : ledCount - 1 - c;                // Weg bis zum Ende
    if (len <= 0) continue;
    const float T = len / sp;
    if (t >= 2 * T) continue;
    busy = true;
    const bool back = t >= T;
    const float d = back ? (2 * T - t) * sp : t * sp;              // Abstand vom Wagen
    const int head = c + side * (int)lroundf(d), dir = back ? -side : side;
    for (int k = 0; k < 10; k++) {                                  // Kopf + ausklingender Schweif
      const int i = head - dir * k;
      if (i < 0 || i >= ledCount || (side < 0 ? i > c : i < c)) continue;
      const float u = (float)abs(i - c) / len;                     // 0 am Wagen, 1 am Ende
      const float hue = back ? idlePal[2] + 0.05f * u : idlePal[side < 0 ? 0 : 1] + 0.06f * u;
      uint8_t r, g, b;
      hsv(hue, LED_IDLE_LEVEL * powf(0.65f, k), r, g, b);
      ledSet(i, r, g, b);
    }
  }
  return busy;
}

// ---------------- Live-Anzeige ----------------
// Wagen als 3 LEDs, Farbe nach Modus; beim Balancieren grün -> rot mit dem Pendelwinkel, im Totpunkt blau; Arbeitsbereich-Enden rot
static void ledShowService() {
  static uint32_t last = 0;
  static bool otaOff = false;
  if (otaActive) {                                              // OTA: LEDs aus, Sendepuffer (14 kB) freigeben
    if (!otaOff && ledInitOk) { otaOff = true; ledClear(); ledShow(); ledInitOk = false; free(ledRmt); ledRmt = nullptr; }
    return;
  }
  if (!ledInitOk) return;
  const uint32_t now = millis();
  if (ledTestOn) { if (now - last >= 15) { last = now; ledTestService(); } return; }
  if (LED_SHOW_LEVEL == 0) return;
  if (now - last < (idleShowOn ? 15u : 40u)) return;              // 25 Hz, während der Ruhe-Show ~67 Hz
  last = now;
  // Ruhe-Show: Wagen steht, Pendel hängt ruhig, Funk da
  static uint32_t calmSince = 0;
  const bool calm = LED_IDLE_SHOW_S > 0 && homed && !ledTestOn &&
                    (ctrl.mode == pc::Mode::IDLE || ctrl.mode == pc::Mode::HOLD) &&
                    fabsf(tx_v) < 0.005f && msSinceRx() < 500 &&
                    fabsf(pc::wrapPi(ctrl.th_c - PI)) < 5.0f * DEG_TO_RAD && fabsf(ctrl.om_c) < 0.15f;
  if (!calm) { calmSince = 0; idleShowOn = false; }
  else if (!calmSince) calmSince = now;
  else if (!idleShowOn && now - calmSince >= (uint32_t)(LED_IDLE_SHOW_S * 1000)) idleShowStart(ledIdxOfX(tx_x));
  const float S = LED_SHOW_LEVEL / 255.0f;
  auto col = [&](float r, float g, float b, int i) { ledSet(i, (uint8_t)(r * S * 255), (uint8_t)(g * S * 255), (uint8_t)(b * S * 255)); };
  ledClear();
  const bool mapped = homed;
  auto idxOf = [](float x) { return ledIdxOfX(x); };
  const bool blink = (now / 300) & 1;
  float r = 0.3f, g = 0.3f, b = 0.3f;                             // IDLE: weiß gedimmt
  switch (ctrl.mode) {
    case pc::Mode::HOLD: r = 0; g = 0.6f; b = 0.6f; break;
    case pc::Mode::ARMED: r = blink ? 1 : 0; g = blink ? 0.8f : 0; b = 0; break;
    case pc::Mode::SWINGUP: r = 1; g = 0.4f; b = 0; break;
    case pc::Mode::BALANCE: {
      if (ctrl.resting) { r = 0; g = 0.25f; b = 1; break; }      // Totpunkt (Ruhezone): blau
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
    if (idleShowOn && !idleShowDraw()) { idleShowOn = false; calmSince = now; }   // nächste Show in LED_IDLE_SHOW_S
  } else {
    col(r, g, b, 0);                                             // ohne Zuordnung: nur Modusfarbe an LED 0
  }
  ledShow();
}
#endif
