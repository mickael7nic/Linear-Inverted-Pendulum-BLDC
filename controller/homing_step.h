// homing_step.h – Homing-Schritt (läuft in loop() im FOC-Takt)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Verbindet den Homing-Zustandsautomaten (homing.h) mit der Hardware: Start/Abbruch, Messwerte hinein,
// Ergebnis (Nullpunkt xZeroAngle, Fahrgrenzen x_hard/x_soft) heraus. Mit BELT_SENSOR außerdem Lernen und
// Speichern der Riemenmarke (NVS: cs_dir = helle Seite, cs_e0 = Kante - Mitte, cs_half = halber Weg,
// bs_thr = Hell/Dunkel-Schwelle).
#pragma once

// Homing läuft hier in loop(), ausgedünnt auf 1 kHz; der ctrl-Task ist währenddessen IDLE
static void homingStep() {
  static uint32_t lastUs = 0;
  if (homeAbort) {
    homeAbort = false;
    if (homing.active()) { homing.abort("durch STOPP abgebrochen"); motor.target = 0; motor.disable(); }
  }
  if (homeReq && !homing.active()) {
    homeReq = false;
    if (ctrl.motorEnabled() || otaActive) return;
    homed = false;
#ifdef BELT_SENSOR
    // Riemenmarke gelernt und Sensor da -> nur Mittensuche; sonst (oder mit 'F') über die Anschläge
    const float half = prefs.getFloat("cs_half", 0);
    homing.bandDir = prefs.getChar("cs_dir", 0);
    homing.edgeOfs = prefs.getFloat("cs_e0", 0);
    if (!homeFullReq && beltOk && beltThr && homing.bandDir != 0 && half > 0.2f) {
      logf(">> Mittensuche: Riemenmarke %s (%u), fahre zur Kante", centerSw() ? "hell" : "dunkel", beltClear);
      homing.startCenter(rawX(), half);
    } else
#endif
      homing.start(rawX());
    homeFullReq = false;
    lastUs = micros();
    return;
  }
  if (!homing.active()) return;
  const uint32_t now = micros();
  const float dt = (now - lastUs) * 1e-6f;
  if (dt < 0.001f) return;   // höchstens 1 kHz (loop() läuft viel schneller)
  lastUs = now;
  const uint8_t stPrev = homing.st;
  // Schritt des Zustandsautomaten: Roh-Position, Geschwindigkeit, gemessener Strom (in x-Richtung), Sensor hell?
  homing.update(fminf(dt, 0.01f), rawX(), cartV(), X_SIGN * motor.current.q, centerSw());
#ifdef BELT_SENSOR
  // Homing über die Anschläge: Hell-/Dunkelwert an beiden Anschlägen -> Schwelle lernen
  static uint16_t cNeg = 0;
  if (beltOk && !homing.centerRun) {
    if (stPrev == pc::Homing::SEEK_NEG && homing.st == pc::Homing::SEEK_POS) cNeg = beltClear;
    if (stPrev == pc::Homing::SEEK_POS && homing.st == pc::Homing::CENTER) {
      const uint16_t cPos = beltClear, hi = max(cNeg, cPos), lo = min(cNeg, cPos);
      // genug Kontrast: Schwelle in die Mitte; homing.swNeg/swPos verschieden -> Automat vermisst danach die Kante
      if (hi > 0 && (float)(hi - lo) / hi >= BELT_MIN_CONTRAST) {
        beltThr = (uint16_t)((hi + lo) / 2);
        beltBright = beltClear > beltThr;
        homing.swNeg = cNeg > beltThr; homing.swPos = cPos > beltThr;   // -> Kante vermessen
      } else {
        homing.swNeg = homing.swPos = false;                            // kein Kontrast -> nichts lernen
        beltNoContrastEvt = true; beltNegVal = cNeg; beltPosVal = cPos;
      }
    }
  }
#endif
  // Weg bekannt -> Nullpunkt + Grenzen setzen. x_hard: Notstopp-Grenze (HOME_MARGIN vor dem Anschlag),
  // x_soft: Arbeitsbereich für Swing-up/Balancieren (noch HOME_SOFT_GAP weiter innen)
  if (homing.st == pc::Homing::CENTER && !homed) {
    xZeroAngle = X_SIGN * homing.xC / PULLEY_RADIUS_M;
    const float half = 0.5f * homing.travel();
    ctrl.cfg.x_hard = half - HOME_MARGIN;
    ctrl.cfg.x_soft = fminf(X_SOFT_M, ctrl.cfg.x_hard - HOME_SOFT_GAP);
    homed = true;
  }
  if (homing.st == pc::Homing::FAILED) homed = false;
#ifdef BELT_SENSOR
  if (homing.st == pc::Homing::DONE && homing.edgeLearned) {          // nach normalem Homing gelernt -> speichern
    homing.edgeLearned = false;
    prefs.putChar("cs_dir", homing.bandDir);
    prefs.putFloat("cs_e0", homing.edgeOfs);
    prefs.putFloat("cs_half", 0.5f * homing.travel());
    prefs.putUShort("bs_thr", beltThr);
    csLearnedEvt = true;
  }
#endif
}
