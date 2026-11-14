// period_meter.h – Pendelperiode messen (Ersatzlänge und Dämpfung)
// Teil des Regelungskerns (pendel_core.h), plattformunabhängig, auch in der Host-Simulation genutzt.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "core_math.h"

namespace pc {

// ============================================================================
// Pendelperiode messen (Pendel hängt, Wagen steht, kleine Auslenkung)
// ============================================================================
// Befehl 'p': Pendel von Hand 10–20° auslenken und loslassen. Gemessen werden 12 halbe Schwingungen
// (Umkehrpunkte = Vorzeichenwechsel von om). Aus der Periode T folgt die Ersatzlänge ell = g·(T/2π)²,
// aus dem Abklingen der Spitzen die Dämpfung c. Beides geht in EKF und LQR ein.
class PeriodMeter {
 public:
  static constexpr int N_HALF = 12;
  bool done = false, valid = false;
  float ell = 0, c = 0, T = 0, amp = 0;

  void start() { n = 0; done = valid = false; prevSign = 0; peak = 0; t = 0; ampMax = 0; }

  // om: Winkelgeschwindigkeit, dth: Auslenkung aus der Ruhelage (wrapPi(th - pi))
  void feed(float dt, float om, float dth) {
    if (done) return;
    t += dt;
    if (fabsf(dth) > ampMax) ampMax = fabsf(dth);
    if (fabsf(om) > peak) peak = fabsf(om);
    const float hyst = 0.02f;
    int sgn = om > hyst ? 1 : (om < -hyst ? -1 : 0);
    if (sgn == 0) return;
    if (prevSign != 0 && sgn != prevSign) {   // Nulldurchgang von om = Umkehrpunkt
      if (n < N_HALF + 1) { tz[n] = t; pk[n] = peak; n++; }
      peak = 0;
      if (n == 1) ampMax = 0;
      if (n == N_HALF + 1) finish();
    }
    prevSign = sgn;
  }
  float timeout() const { return t; }

 private:
  int n = 0, prevSign = 0;
  float t = 0, peak = 0, ampMax = 0;
  float tz[N_HALF + 1], pk[N_HALF + 1];

  void finish() {
    done = true;
    T = 2.0f * (tz[N_HALF] - tz[0]) / N_HALF;
    amp = ampMax;
    // Großwinkel-Korrektur: T = T0 (1 + amp^2/16)
    const float T0 = T / (1.0f + amp * amp / 16.0f);
    ell = G * (T0 / (2.0f * PI_F)) * (T0 / (2.0f * PI_F));
    // Dämpfung aus Abklingen der om-Spitzen (pk[1] .. pk[N_HALF])
    const float a0 = pk[1], a1 = pk[N_HALF];
    c = (a0 > 0 && a1 > 0 && a1 < a0) ? 2.0f * logf(a0 / a1) / (tz[N_HALF] - tz[1]) : 0.0f;
    valid = (amp > 0.03f && amp < 0.6f && ell > 0.03f && ell < 2.0f);
  }
};

}  // namespace pc
