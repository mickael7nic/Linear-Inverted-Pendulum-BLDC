// current_test.h – Strom-Test (Befehl I, nur mit Strommessung)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

#if USE_CURRENT_SENSE
// Prägt Gleichstrom-Stufen in d-Richtung ein (kein Moment) und vergleicht Mess- mit Erwartungswert U/R.
// Übersteuert die Messung, bleibt der Messwert bei steigender Spannung hängen.
// Abgleich der beiden Strommess-Kanäle A/B. Ungleiche Verstärkung (z. B. Lötstellen an den parallelen Shunts)
// erzeugt eine Drehmoment-Welligkeit mit 2x elektrischer Frequenz -> Wagen "rastet" bei langsamer Fahrt.
// Spannungsvektor bei -30°: Strom fließt A -> B in Reihe (C ~ 0), also |Ia| = |Ib| unabhängig von R.
// Beide Richtungen (+/-) -> Offsets fallen heraus. Rotor dreht sich dabei um eine halbe Polteilung (~1 mm Wagen).
static void currentGainBalance(float R) {
  const float ud = fminf(0.6f, fmaxf(0.05f, 2.5f * R));   // ~2,5 A
  float a[2], b[2], c[2];
  for (int k = 0; k < 2; k++) {
    const float ang = -_PI / 6.0f + k * _PI;
    motor.setPhaseVoltage(0, ud, _normalizeAngle(ang));
    delay(400);
    double sa = 0, sb = 0;
    for (int i = 0; i < 200; i++) {
      const PhaseCurrent_s cur = currentSense.getPhaseCurrents();
      sa += cur.a; sb += cur.b; delayMicroseconds(500);
    }
    a[k] = sa / 200; b[k] = sb / 200; c[k] = -(a[k] + b[k]);
  }
  motor.setPhaseVoltage(0, 0, 0);
  const float IA = 0.5f * (a[0] - a[1]), IB = -0.5f * (b[0] - b[1]);
  logf(">> Kanal-Abgleich bei Ud=%.2f V: |Ia|=%.3f A  |Ib|=%.3f A  (Ic=%.3f/%.3f A, sollte ~0)  Offset a=%.3f b=%.3f A",
       ud, IA, IB, c[0], c[1], 0.5f * (a[0] + a[1]), 0.5f * (b[0] + b[1]));
  if (IA < 0.5f || IB < 0.5f) { logf(">> Kanal-Abgleich: Strom zu klein, nicht übernommen"); return; }
  const float k = IA / IB;                                 // Korrektur für Kanal B
  if (k < 0.6f || k > 1.6f) { logf(">> Kanal-Abgleich: Verhaeltnis %.3f unplausibel, nicht übernommen", k); return; }
  if (!CS_GAIN_CORR) {
    logf(">> Kanal B / A = %.3f (nur gemessen; Korrektur aus, CS_GAIN_CORR = false)", 1.0f / k);
    return;
  }
  const float tot = prefs.getFloat("gB", 1.0f) * k;
  currentSense.gain_b *= k;
  prefs.putFloat("gB", tot);
  logf(">> Kanal B / A = %.3f -> Kanal B um %+.1f %% korrigiert (gesamt %.3f, gespeichert). Erneut 'I' -> sollte ~1,000 zeigen",
       1.0f / k, (k - 1.0f) * 100.0f, tot);
}

static void currentSenseTest() {
  csTestReq = false;
  const float range = 1.6f / (SHUNT_OHM * INA240_GAIN);
  logf(">> Strom-Test: R gespeichert %.4f Ohm, Messbereich rechnerisch +-%.2f A", motor.phase_resistance, range);
  motor.enable();
  const float el = motor.electricalAngle();
  const float steps[] = {0.02f, 0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.4f, 0.5f, 0.6f};
  float U[10], I[10];
  int n = 0, over = 0;
  for (float ud : steps) {
    if (ud / 0.05f > CURRENT_LIMIT_A) break;          // konservativ: R >= 0,05 Ohm angenommen
    motor.setPhaseVoltage(0, ud, el);
    delay(150);
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      const PhaseCurrent_s c = currentSense.getPhaseCurrents();
      const float cc = (c.c == 0.0f) ? -(c.a + c.b) : c.c;
      sum += sqrtf((2.0f / 3.0f) * (c.a * c.a + c.b * c.b + cc * cc));
      delay(2);
    }
    U[n] = ud; I[n] = sum / 50;
    logf("   Ud=%.2f V  gemessen %.2f A", U[n], I[n]);
    n++;
    if (I[n - 1] > 0.9f * range && ++over >= 2) break;  // zwei Stufen über dem Bereich reichen
  }
  motor.setPhaseVoltage(0, 0, el);
  currentGainBalance(motor.phase_resistance > 0.02f && motor.phase_resistance < 0.5f ? motor.phase_resistance : 0.15f);
  motor.disable();
  // Gerade I = (U - U0) / R durch die Punkte im sicheren Bereich (0,2 A .. 80 % Messbereich)
  double su = 0, si = 0, suu = 0, sui = 0; int m = 0;
  for (int k = 0; k < n; k++)
    if (I[k] > 0.2f && I[k] < 0.8f * range) { su += U[k]; si += I[k]; suu += U[k] * U[k]; sui += U[k] * I[k]; m++; }
  if (m < 2 || (m * suu - su * su) <= 0) { logf(">> Strom-Test: zu wenige Punkte im Messbereich"); return; }
  const double slope = (m * sui - su * si) / (m * suu - su * su), icpt = (si - slope * su) / m;
  if (slope <= 0) { logf(">> Strom-Test: keine sinnvolle Gerade"); return; }
  const float Rfit = 1.0f / slope, U0 = -icpt / slope;
  logf(">> Ergebnis: R_eff = %.4f Ohm (Wicklung+Shunt+FETs+Kabel), Totzeit-Verlust ~%.3f V", Rfit, U0);
  // Übersteuerung: Messwert bleibt hinter der Geraden zurück
  for (int k = 0; k < n; k++) {
    const float pred = (U[k] - U0) / Rfit;
    if (pred > range && I[k] < 0.85f * pred) {
      logf(">> UEBERSTEUERT ab ~%.1f A (erwartet %.2f A, gemessen %.2f A) -> kleinerer Shunt noetig", range, pred, I[k]);
      break;
    }
    if (k == n - 1) logf(">> keine Uebersteuerung bis %.2f A erkennbar", I[k]);
  }
  if (Rfit > 0.02f && Rfit < 0.5f) {
    motor.phase_resistance = Rfit;
    prefs.putFloat("R", Rfit);
    if (U0 > 0.0f && U0 < 0.5f) prefs.putFloat("dtU", U0);
    float bw = CURRENT_BW_HZ;
    while (motor.tuneCurrentController(bw) != 0 && bw > 50.0f) bw *= 0.8f;
    logf(">> R gespeichert, Stromregler neu: BW=%.0f Hz  P=%.3f  I=%.1f", bw, motor.PID_current_q.P, motor.PID_current_q.I);
  }
}
#endif
