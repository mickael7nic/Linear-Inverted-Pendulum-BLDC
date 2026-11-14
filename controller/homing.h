// homing.h – Homing-Zustandsautomat: Anschläge über den Phasenstrom (INA240) und Riemenmarke
// Plattformunabhängig (kein Arduino-Code, läuft auch in der Host-Simulation). Wird in loop() mit update()
// aufgerufen (homing_step.h) und liefert den Stromsollwert iq. Der Encoder kennt nach dem Einschalten nur
// relative Winkel; das Homing legt fest, welcher Winkel die Schienenmitte (x = 0) ist.
//
// A) Homing über die Anschläge (start, Befehl 'F' oder ohne gelernte Riemenmarke):
//   SEEK_NEG: mit Rampe Richtung -x fahren, bis |Iq| > i_stall und |v| < v_stall für t_stall (= Anschlag)
//   SEEK_POS: dasselbe Richtung +x
//   CENTER:   Weg prüfen, Mitte = (xNeg + xPos)/2 anfahren, dort ruhig stehen -> DONE
//   Mit Riemensensor: sieht er an den beiden Anschlägen verschiedene Werte (hell/dunkel), wird danach die
//   Hell-Dunkel-Kante einmal vermessen (EDGE_*) und ihr Abstand zur Mitte gelernt (edgeOfs).
// B) Mittensuche (startCenter, Befehl 'H' mit gelernter Marke): nur die Kante suchen, Mitte = Kante - edgeOfs.
//   Kein Anfahren der Anschläge.
//   EDGE_FAST: schnell in Richtung Kante (Richtung aus dem aktuellen Hell/Dunkel-Zustand)
//   EDGE_BACK: nach dem Überfahren e_back weit ins Helle zurück
//   EDGE_SLOW: langsam von hell nach dunkel (Richtung -bandDir) -> Kante immer gleich angefahren = wiederholgenau
// "sw" = Sensor sieht hell; "Band" bzw. bandDir = Seite, auf der der Sensor hell sieht (Begriff stammt vom
// früheren Kupferband-Kontakt, die Logik ist dieselbe).
// Positionen in "Roh-x" [m] = X_SIGN * Wellenwinkel * Radius (ohne Nullpunkt).
#pragma once
#include <math.h>
#include <stdint.h>

namespace pc {

struct HomingConfig {
  float speed, accel;          // Suchfahrt [m/s], [m/s^2]
  float i_max, i_stall;        // Strombegrenzung / Anschlagschwelle [A]
  float v_stall, t_stall;      // [m/s], [s]
  float min_travel, max_travel;// Plausibilität Verfahrweg [m]
  float timeout;               // je Suchfahrt [s]
  float m_eff, wn, zeta;       // Positionsregler
  float n_per_a;               // Kraft pro Iq [N/A] = Kt / r
  float e_fast = 0.10f, e_slow = 0.01f, e_back = 0.01f, t_db = 0.003f;   // Riemenmarke: Fahrten [m/s], Rückweg [m], Entprellung [s]
};

class Homing {
 public:
  enum State : uint8_t { OFF, SEEK_NEG, SEEK_POS, CENTER, DONE, FAILED, EDGE_FAST, EDGE_BACK, EDGE_SLOW };
  HomingConfig cfg{};
  volatile State st = OFF;   // aktueller Zustand (wird vom UI-Task fürs Log gelesen)
  float xNeg = 0, xPos = 0, xC = 0;   // Anschlag -x, Anschlag +x, Mitte (Roh-x) [m]
  float iqFreeMax = 0;         // größter Strom bei freier Fahrt (zum Einstellen von i_stall)
  float iq = 0;                // Stromsollwert [A] in x-Richtung
  const char* msg = "";   // Fehlertext bei FAILED

  // Riemenmarke (dauerhaft gespeichert in homing_step.h)
  int8_t bandDir = 0;          // hell auf Seite +1 (x > Kante) / -1 (x < Kante) / 0 unbekannt
  float edgeOfs = 0;           // Kante - Mitte [m] (gelernt)
  bool edgeLearned = false;    // nach diesem Homing neu gelernt -> speichern
  bool centerRun = false;      // laufende Fahrt ist eine Mittensuche
  float xEdge = 0;   // gemessene Kantenposition (Roh-x)

  bool active() const { return st == SEEK_NEG || st == SEEK_POS || st == CENTER || st == EDGE_FAST || st == EDGE_BACK || st == EDGE_SLOW; }
  float travel() const { return xPos - xNeg; }

  // Homing über die Anschläge starten (x = aktuelle Roh-Position)
  void start(float x) { xr = x; vr = 0; iqFreeMax = 0; msg = ""; edgeLearned = false; centerRun = false; swNeg = swPos = false; enter(SEEK_NEG); }
  // Mittensuche: Kante finden, Mitte = Kante - edgeOfs, Weg = 2*half
  void startCenter(float x, float half) {
    xr = x; vr = 0; msg = ""; edgeLearned = false; centerRun = true; halfRun = half; swInit = true; enter(EDGE_FAST);
  }
  // STOPP: laufendes Homing beenden
  void abort(const char* why) { if (active()) fail(why); }

  // dt [s], x/v Rohposition/-geschwindigkeit, iqMeas gemessener Iq in x-Richtung
  void update(float dt, float x, float v, float iqMeas, bool swRaw = false) {
    if (!active()) { iq = 0; return; }
    tState += dt;
    // Sensor entprellen (t_db): Zustand muss so lange stabil anders sein
    if (swRaw != sw) { tSw += dt; if (tSw >= cfg.t_db) { sw = swRaw; tSw = 0; swChanged = true; } } else tSw = 0;
    // erster Aufruf einer Kantensuche: Zustand übernehmen und Suchrichtung festlegen
    // (steht der Wagen im Hellen, liegt die Kante in Richtung -bandDir, sonst in Richtung bandDir)
    if (swInit) { sw = swRaw; swInit = false; swChanged = false; edgeDir = sw ? -bandDir : bandDir; }
    // Sollgeschwindigkeit je Zustand; daraus eine Rampe (vr, xr) als Sollbahn
    float vt = 0;
    if (st == SEEK_NEG) vt = -cfg.speed;
    else if (st == SEEK_POS) vt = cfg.speed;
    else if (st == EDGE_FAST) vt = edgeDir * cfg.e_fast;
    else if (st == EDGE_SLOW) vt = -bandDir * cfg.e_slow;
    else {  // CENTER bzw. EDGE_BACK: Bremsprofil auf das Ziel
      const float tgt = st == EDGE_BACK ? xBack : xC;
      const float e = tgt - xr;
      vt = copysignf(fminf(st == EDGE_BACK ? cfg.e_fast : 2.0f * cfg.speed, sqrtf(2.0f * cfg.accel * fabsf(e))), e);
    }
    const float dv = cfg.accel * dt;
    vr += fmaxf(-dv, fminf(dv, vt - vr));
    xr += vr * dt;
    if (st == CENTER && fabsf(xC - xr) < 0.0005f && fabsf(vr) < 0.03f) { xr = xC; vr = 0; }
    if (st == EDGE_BACK && fabsf(xBack - xr) < 0.0005f && fabsf(vr) < 0.02f) { xr = xBack; vr = 0; }
    // Folgefehler begrenzen -> am Anschlag läuft der Strom sicher bis i_max
    const float ex = fmaxf(-0.03f, fminf(0.03f, xr - x));
    xr = x + ex;
    // PD-Lageregler auf die Sollbahn -> Kraft -> Strom (begrenzt auf i_max: am Anschlag drückt er nur sanft)
    const float F = cfg.m_eff * (cfg.wn * cfg.wn * ex + 2.0f * cfg.zeta * cfg.wn * (vr - v));
    iq = fmaxf(-cfg.i_max, fminf(cfg.i_max, F / cfg.n_per_a));

    // --- Auswertung je Zustand ---
    if (st == SEEK_NEG || st == SEEK_POS) {
      const bool stall = fabsf(iqMeas) > cfg.i_stall && fabsf(v) < cfg.v_stall;
      tStall = stall ? tStall + dt : 0.0f;
      if (!stall && fabsf(v) > 0.8f * cfg.speed && fabsf(iqMeas) > iqFreeMax) iqFreeMax = fabsf(iqMeas);   // Diagnose
      if (tStall >= cfg.t_stall) {
        xr = x; vr = 0;
        if (st == SEEK_NEG) { xNeg = x; swNeg = swRaw; enter(SEEK_POS); }
        else {
          xPos = x; swPos = swRaw;
          const float L = travel();
          if (L < cfg.min_travel || L > cfg.max_travel) return fail("Verfahrweg unplausibel (i_stall pruefen)");
          xC = 0.5f * (xNeg + xPos);
          enter(CENTER);
        }
        return;
      }
      if (tState > cfg.timeout) return fail("Timeout - kein Anschlag erkannt");
    } else if (st == CENTER) {
      // Restfehler durch Reibung ist egal: der Nullpunkt wird aus xC gesetzt, nicht aus der Endposition
      if (xr == xC && fabsf(x - xC) < 0.005f && fabsf(v) < 0.005f) {
        // nach Homing über die Anschläge: Riemensensor an beiden Anschlägen verschieden -> Kante einmal vermessen
        if (!centerRun && !edgeLearned && swNeg != swPos) {
          bandDir = swPos ? 1 : -1; centerRun = false; swInit = true; enter(EDGE_FAST); return;
        }
        st = DONE; iq = 0;
      }
      else if (tState > 10.0f) fail("Mitte nicht erreicht");
    } else {  // Kantensuche (EDGE_*)
      const bool stall = fabsf(iqMeas) > cfg.i_stall && fabsf(v) < cfg.v_stall;
      tStall = stall ? tStall + dt : 0.0f;
      if (tStall >= cfg.t_stall) return fail("Anschlag statt Kante - Riemenmarke/Sensor pruefen");
      if (st == EDGE_FAST) {
        if (swChanged) {                       // Kante überfahren -> ins Helle zurück bzw. weiter ins Helle
          swChanged = false;
          xBack = x + bandDir * cfg.e_back;     // in beiden Fällen: e_back weit im Hellen stehen
          xr = x; enter(EDGE_BACK);
        } else if (tState > (2.2f * halfRunOrTravel()) / cfg.e_fast + 2.0f) return fail("Riemenmarke: Kante nicht gefunden");
      } else if (st == EDGE_BACK) {
        if (xr == xBack && fabsf(v) < 0.01f) {
          if (!sw) return fail("Riemenmarke: hell nicht erkannt (Sensor?)");
          swChanged = false; enter(EDGE_SLOW);
        } else if (tState > 3.0f) return fail("Riemenmarke: Rueckweg nicht erreicht");
      } else {  // EDGE_SLOW: langsam von hell nach dunkel
        if (swChanged && !sw) {
          xEdge = x - (-bandDir) * cfg.e_slow * cfg.t_db;      // Entprellzeit herausrechnen
          xr = x; vr = 0;
          if (centerRun) {                      // Mittensuche: Mitte und Grenzen aus der Kante
            xC = xEdge - edgeOfs; xNeg = xC - halfRun; xPos = xC + halfRun;
          } else { edgeOfs = xEdge - xC; edgeLearned = true; }   // Lernen nach normalem Homing
          enter(CENTER);
        } else if (tState > 3.0f * cfg.e_back / cfg.e_slow + 2.0f) return fail("Riemenmarke: Kante bei langsamer Fahrt nicht gefunden");
      }
    }
  }

 private:
  float xr = 0, vr = 0, tState = 0, tStall = 0;   // Sollposition/-geschwindigkeit, Zeit im Zustand, Zeit am Anschlag
  bool sw = false, swInit = false, swChanged = false;   // entprellter Sensorzustand, Init-Flag, Flanke erkannt
 public:
  bool swNeg = false, swPos = false;           // Marke an den Anschlägen (von außen überschreibbar: gelernte Schwelle)
 private:
  float tSw = 0, xBack = 0, halfRun = 0.45f;   // Entprell-Zeit, Ziel EDGE_BACK, halber Verfahrweg (Mittensuche)
  int8_t edgeDir = 1;   // Richtung von EDGE_FAST
  float halfRunOrTravel() const { return centerRun ? halfRun : 0.5f * travel(); }
  void enter(State s) { tState = 0; tStall = 0; st = s; }
  void fail(const char* why) { msg = why; iq = 0; st = FAILED; }
};

}  // namespace pc
