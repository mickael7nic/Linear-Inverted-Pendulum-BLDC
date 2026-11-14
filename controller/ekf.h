// ekf.h – EKF für Pendelwinkel, Winkelgeschwindigkeit und Gyro-Bias
// Teil des Regelungskerns (pendel_core.h), plattformunabhängig, auch in der Host-Simulation genutzt.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "core_math.h"

namespace pc {

// ============================================================================
// EKF: Zustand [th, om, b_gyro], Gyro als Messung -> Bias über Pendeldynamik beobachtbar
// ============================================================================
// Warum ein EKF: Der Gyro allein driftet (Bias), die Beschleunigung allein ist durch die Wagenbewegung und
// die Zentripetalbeschleunigung gestört. Der EKF kennt das Pendelmodell und die kommandierte Wagenbeschleunigung
// a und kann deshalb beides richtig verrechnen.
// Je Takt: predict() rechnet das Modell einen Schritt weiter, correct() vergleicht mit den Messungen
// (Gyro, Radial-, Tangentialbeschleunigung) und korrigiert. Ausreißer (> gate Sigma) werden verworfen.
// P = Kovarianz (Unsicherheit) der Schätzung.
struct EKFParams {
  float ell, rs, c;       // Pendelmodell
  float sig_gyro;         // Gyro-Rauschen [rad/s]
  float sig_acc;          // Accel-Rauschen inkl. Modellfehler [m/s^2]
  float q_om;             // Prozessrauschen auf om_dot [(rad/s^2)^2 * s]
  float q_b;              // Bias-Random-Walk [(rad/s)^2 / s]
  float gate;             // Innovations-Gate in Sigma
};

class EKF {
 public:
  float th = PI_F, om = 0, b = 0;   // Winkel [rad], Winkelrate [rad/s], Gyro-Bias [rad/s]
  float P[3][3] = {{0.1f, 0, 0}, {0, 0.1f, 0}, {0, 0, 1e-3f}};
  EKFParams p{};
  uint32_t rejected = 0;   // verworfene Messungen (Status)
  float prevGyro = 0, lastDt = 1e-3f;   // für th_dd aus der Gyro-Differenz
  int nSince = 0;   // Takte seit dem letzten Gyro-Wert
  bool havePrev = false;
  // Max. Winkelkorrektur je Beschleunigungs-Update: nach Funklücken ist P groß, dann würde ein einzelnes
  // verrauschtes Accel-Sample den Winkel um Grad springen lassen -> stattdessen sanft nachführen.
  float dth_max = 0.0035f;       // [rad] ~0,2° je Sample (800 Hz -> bis 160°/s Nachführung)
  int clampCnt = 0;              // aufeinanderfolgende begrenzte Updates (Diagnose)

  void init(const EKFParams& prm) { p = prm; }

  // Neustart mit bekanntem Winkel (Kalibrierung, hängendes Pendel)
  void reset(float th0, float b0) {
    th = th0; om = 0; b = b0;
    memset(P, 0, sizeof(P));
    P[0][0] = 1e-3f; P[1][1] = 1e-3f; P[2][2] = 1e-6f;
    havePrev = false;
  }

  // Pendelmodell: Winkelbeschleunigung aus Winkel, Winkelrate und Wagenbeschleunigung a
  float omDot(float thv, float omv, float a) const {
    return (G * sinf(thv) - a * cosf(thv)) / p.ell - p.c * omv;
  }

  // Zeitschritt: Zustand mit dem Modell weiterrechnen, Unsicherheit P wachsen lassen
  void predict(float dt, float a) {
    lastDt = dt;
    if (nSince < 100) nSince++;
    const float s = sinf(th), c = cosf(th);
    const float od = (G * s - a * c) / p.ell - p.c * om;
    th += om * dt + 0.5f * od * dt * dt;
    om += od * dt;
    const float f10 = (G * c + a * s) / p.ell * dt, f11 = 1.0f - p.c * dt;
    // P = F P F' + Q, F = [[1 dt 0][f10 f11 0][0 0 1]]
    float FP[3][3];
    for (int j = 0; j < 3; j++) {
      FP[0][j] = P[0][j] + dt * P[1][j];
      FP[1][j] = f10 * P[0][j] + f11 * P[1][j];
      FP[2][j] = P[2][j];
    }
    for (int i = 0; i < 3; i++) {
      P[i][0] = FP[i][0] + dt * FP[i][1];
      P[i][1] = f10 * FP[i][0] + f11 * FP[i][1];
      P[i][2] = FP[i][2];
    }
    P[1][1] += p.q_om * dt;
    P[2][2] += p.q_b * dt;
  }

  // Skalares Update: y gemessen, h Erwartung, H Jacobi-Zeile, R Varianz
  bool update(float y, float h, const float H[3], float R, bool limitTh = false) {
    float PH[3];
    for (int i = 0; i < 3; i++) PH[i] = P[i][0] * H[0] + P[i][1] * H[1] + P[i][2] * H[2];
    const float S = H[0] * PH[0] + H[1] * PH[1] + H[2] * PH[2] + R;
    const float inn = y - h;
    if (inn * inn > p.gate * p.gate * S) { rejected++; return false; }
    float K[3];
    for (int i = 0; i < 3; i++) K[i] = PH[i] / S;
    float sc = 1.0f;                                  // Teil-Update, falls der Winkelschritt zu groß wäre
    if (limitTh && fabsf(K[0] * inn) > dth_max) { sc = dth_max / fabsf(K[0] * inn); clampCnt++; }
    else if (limitTh) clampCnt = 0;
    th += sc * K[0] * inn; om += sc * K[1] * inn; b += sc * K[2] * inn;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] -= sc * K[i] * PH[j];
    for (int i = 0; i < 3; i++)                       // Symmetrie erzwingen
      for (int j = i + 1; j < 3; j++) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
    return true;
  }

  // Messmodelle ohne Annahme "freies Pendel" (gilt auch bei Hand/Stoß am Pendel):
  //   f_r = a sin(th) + g cos(th) - rs om^2
  //   f_t - rs * th_dd = a cos(th) - g sin(th),  th_dd aus der Differenz zweier Gyro-Samples
  void correct(float gyro, float fr, float ft, float a, bool useAcc) {
    const bool haveDd = havePrev && nSince > 0 && nSince <= 4;
    const float dtg = nSince * lastDt;
    const float thdd = haveDd ? (gyro - prevGyro) / dtg : 0.0f;
    prevGyro = gyro; havePrev = true; nSince = 0;

    // Gyro misst om + Bias
    const float Hg[3] = {0, 1, 1};
    update(gyro, om + b, Hg, p.sig_gyro * p.sig_gyro);
    if (!useAcc) return;
    const float R = p.sig_acc * p.sig_acc;
    float s = sinf(th), c = cosf(th);
    // Radialbeschleunigung: hängt vor allem vom Winkel ab (Schwerkraftrichtung)
    const float Hr[3] = {a * c - G * s, -2.0f * p.rs * om, 0};
    update(fr, a * s + G * c - p.rs * om * om, Hr, R, true);
    if (!haveDd) return;
    s = sinf(th); c = cosf(th);
    const float sdd = p.rs * p.sig_gyro * 1.4142f / dtg;      // Rauschen der Gyro-Differenz
    // Tangentialbeschleunigung minus Anteil der Winkelbeschleunigung: ebenfalls Winkelinformation
    const float Ht[3] = {-a * s - G * c, 0, 0};
    update(ft - p.rs * thdd, a * c - G * s, Ht, R + sdd * sdd, true);
  }
};

}  // namespace pc
