// cogging.h – Rastmoment-Kompensation (Anti-Cogging) für Motor + Riemen ('M' lernen)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Der Motor rastet (Nuten/Magnete, dazu der Zahneingriff des Riemens). Ohne Lagerreibung bleibt der Wagen in
// diesen Rasten hängen und springt dann weiter (gemessen: ~3 Hz, ~1 mm, bis 0,8 A Haltestrom).
// Lernen: Der Motor fährt mit einem eigenen, steifen Lageregler sehr langsam 2 Umdrehungen vor und zurück
// (Wagen ~80 mm) und merkt sich je Winkelstück den nötigen Strom. Vor/zurück gemittelt -> Reibung fällt heraus,
// übrig bleibt das Rastmoment. Im Betrieb wird es als Strom-Vorsteuerung abgezogen (loop(), FOC-Takt).
#pragma once

constexpr int   COG_BINS   = 1024;                  // je Motorumdrehung (Rastperiode 1/84 U -> ~12 Stützstellen)
constexpr float COG_VEL    = 0.6f;                  // [rad/s] Lerngeschwindigkeit (Wagen ~4 mm/s)
constexpr float COG_REVS   = 2.0f;                  // Umdrehungen je Richtung
constexpr float COG_KP     = 80.0f;                 // [A/rad] Lageregler beim Lernen (~ωn 150 rad/s)
constexpr float COG_KD     = 0.5f;                  // [A/(rad/s)] (ζ ~0,5)
constexpr float COG_FADE_W = 40.0f;                 // [rad/s] darüber Vorsteuerung ausblenden (~0,25 m/s)

static int16_t cogTab[COG_BINS];                    // [mA], Motorstrom (Motor-Koordinaten, ohne X_SIGN)
static bool cogOk = false;

static inline int cogBin(float ang) {
  float a = fmodf(ang, _2PI);
  if (a < 0) a += _2PI;
  int b = (int)(a * (COG_BINS / _2PI));
  return b >= COG_BINS ? COG_BINS - 1 : b;
}

// Vorsteuerstrom [A] für den aktuellen Winkel (0 ohne Tabelle oder bei schneller Fahrt)
static inline float cogFF() {
  if (!cogOk || !COG_COMP) return 0;
  const float w = fabsf(motor.shaft_velocity);
  if (w > 2 * COG_FADE_W) return 0;
  const float k = w < COG_FADE_W ? 1.0f : 2.0f - w / COG_FADE_W;
  return k * cogTab[cogBin(motor.shaft_angle)] * 1e-3f;
}

static void cogLoad() {
  cogOk = prefs.getBytesLength("cog") == sizeof(cogTab) && prefs.getBytes("cog", cogTab, sizeof(cogTab)) == sizeof(cogTab);
  if (cogOk) {
    int lo = 0, hi = 0;
    for (int i = 0; i < COG_BINS; i++) { lo = min(lo, (int)cogTab[i]); hi = max(hi, (int)cogTab[i]); }
    logf("Rastmoment-Kompensation: %s, Tabelle %d..%d mA", COG_COMP ? "an" : "AUS (COG_COMP)", lo, hi);
  } else logf("Rastmoment-Kompensation: noch nicht gelernt ('M')");
}

// blockierend, aus loop() (Kern 1). Wagen muss gehomt sein und ~45 mm Platz in beide Richtungen haben.
static void cogCalibrate() {
  cogCalReq = false; cogAbort = false;
  const float r = PULLEY_RADIUS_M;
  const float span = COG_REVS * _2PI * r;           // Wagenweg je Richtung [m]
  logf(">> Rastmoment lernen: Wagen faehrt langsam %.0f mm vor und zurueck (~%.0f s), Pendel haengen lassen",
       span * 1e3f, 2 * COG_REVS * _2PI / COG_VEL + 2);
  // Arbeitsspeicher nur während des Lernens (statisch wäre kein Platz: ~16 kB)
  float* sumF = (float*)calloc(COG_BINS, sizeof(float));   // Summe Strom je Winkelstück, vorwärts
  float* sumB = (float*)calloc(COG_BINS, sizeof(float));   //   rückwärts
  uint16_t (*cnt)[COG_BINS] = (uint16_t(*)[COG_BINS])calloc(2 * COG_BINS, sizeof(uint16_t));
  if (!sumF || !sumB || !cnt) { free(sumF); free(sumB); free(cnt); logf(">> Rastmoment lernen: kein Speicher"); return; }
  const bool wasOk = cogOk; cogOk = false;          // während des Lernens keine Vorsteuerung
  motor.enable();
  for (int i = 0; i < 200; i++) { motor.loopFOC(); motor.move(); }
  const float a0 = motor.shaft_angle, aEnd = a0 + COG_REVS * _2PI;
  float at = a0;                                    // Sollwinkel
  int phase = 0;                                    // 0 Anlauf vor, 1 vor (messen), 2 zurück (messen)
  uint32_t tLast = micros(), tStart = millis();
  float iqMax = 0;
  bool abortRun = false;
  while (phase < 3) {
    const uint32_t now = micros();
    const float dt = (now - tLast) * 1e-6f; tLast = now;
    const float dir = phase == 2 ? -1.0f : 1.0f;
    at += dir * COG_VEL * dt;
    if (phase == 0 && at >= a0 + 0.3f) { phase = 1; }
    if (phase == 1 && at >= aEnd) { phase = 2; }
    if (phase == 2 && at <= a0 + 0.3f) { phase = 3; }
    motor.loopFOC(); motor.move();
    const float e = at - motor.shaft_angle;
    float iq = COG_KP * e + COG_KD * (dir * COG_VEL - motor.shaft_velocity);
    iq = pc::clampf(iq, -3.0f, 3.0f);
    motor.target = iq;
    if (phase == 1 || phase == 2) {
      const int b = cogBin(motor.shaft_angle);
      const float meas = motor.current.q;          // gemessener Strom = tatsächlich nötiger Strom
      if (phase == 1) { sumF[b] += meas; if (cnt[0][b] < 65000) cnt[0][b]++; }
      else            { sumB[b] += meas; if (cnt[1][b] < 65000) cnt[1][b]++; }
      iqMax = fmaxf(iqMax, fabsf(meas));
    }
    if (fabsf(e) > 0.5f) { logf(">> Rastmoment lernen: Wagen folgt nicht (Fehler %.2f rad) - abgebrochen", e); abortRun = true; break; }
    if (millis() - tStart > 60000) { logf(">> Rastmoment lernen: Zeitlimit - abgebrochen"); abortRun = true; break; }
    if (cogAbort) { logf(">> Rastmoment lernen: durch STOPP abgebrochen"); abortRun = true; break; }
  }
  motor.target = 0;
  motor.disable();
  cogAbort = false;
  if (abortRun) { cogOk = wasOk; free(sumF); free(sumB); free(cnt); return; }
  // Mittel aus vor und zurück = Rastmoment (Reibung/Dämpfung haben entgegengesetztes Vorzeichen)
  int missing = 0;
  double mean = 0;
  float* tab = sumF;                                // Ergebnis in den Vorwärts-Puffer (wird nicht mehr gebraucht)
  for (int i = 0; i < COG_BINS; i++) {
    if (cnt[0][i] && cnt[1][i]) tab[i] = 0.5f * (sumF[i] / cnt[0][i] + sumB[i] / cnt[1][i]);
    else { tab[i] = NAN; missing++; }
  }
  for (int i = 0; i < COG_BINS; i++) if (tab[i] != tab[i]) {       // Lücken linear füllen
    int l = i, h = i;
    while (tab[(l + COG_BINS) % COG_BINS] != tab[(l + COG_BINS) % COG_BINS] && l > i - COG_BINS) l--;
    while (tab[h % COG_BINS] != tab[h % COG_BINS] && h < i + COG_BINS) h++;
    const float vl = tab[(l + COG_BINS) % COG_BINS], vh = tab[h % COG_BINS];
    tab[i] = (vl == vl && vh == vh) ? vl + (vh - vl) * (float)(i - l) / (float)(h - l) : 0;
  }
  // leicht glätten (3er-Mittel, zyklisch) und Mittelwert abziehen (Gleichanteil = keine Rastung)
  float* sm = sumB;
  for (int i = 0; i < COG_BINS; i++)
    sm[i] = (tab[(i + COG_BINS - 1) % COG_BINS] + tab[i] + tab[(i + 1) % COG_BINS]) / 3.0f;
  for (int i = 0; i < COG_BINS; i++) mean += sm[i];
  mean /= COG_BINS;
  float lo = 1e9f, hi = -1e9f, rms = 0;
  for (int i = 0; i < COG_BINS; i++) {
    const float v = sm[i] - (float)mean;
    cogTab[i] = (int16_t)pc::clampf(v * 1000.0f, -3000, 3000);
    lo = fminf(lo, v); hi = fmaxf(hi, v); rms += v * v;
  }
  rms = sqrtf(rms / COG_BINS);
  free(sumF); free(sumB); free(cnt);
  prefs.putBytes("cog", cogTab, sizeof(cogTab));
  cogOk = true;
  const float npa = KT_NM_PER_A / r;               // N je A am Wagen
  logf(">> Rastmoment gelernt: %+.2f..%+.2f A (rms %.2f A = %.1f N am Wagen), %d Luecken gefuellt, gespeichert",
       lo, hi, rms, rms * npa, missing);
  logf("   Kompensation %s. Erneut 'M' zum Vergleich: rms sollte deutlich kleiner sein", COG_COMP ? "aktiv" : "AUS (COG_COMP)");
}
