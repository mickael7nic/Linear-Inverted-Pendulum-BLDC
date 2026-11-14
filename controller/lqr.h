// lqr.h – LQR-Entwurf (zeitdiskret, Doubling-Algorithmus)
// Teil des Regelungskerns (pendel_core.h), plattformunabhängig, auch in der Host-Simulation genutzt.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "core_math.h"

namespace pc {

// ============================================================================
// LQR-Entwurf (zeitdiskret, exakte ZOH-Diskretisierung, Doubling-Algorithmus)
// ============================================================================
// Rechnet auf dem ESP32 (in double, ~ms) die optimalen Rückführverstärkungen K für a = -K·[x, v, th, om].
// Gewichte Q (qx, qv, qth, qom) bestrafen Abweichungen der Zustände, R (r) den Stellaufwand:
// größeres qx -> Wagen hält die Mitte strenger; größeres r -> sanfter, weniger Beschleunigung.
// Die Riccati-Gleichung wird mit dem Doubling-Verfahren gelöst (konvergiert in wenigen Iterationen).
struct LQRWeights { float qx, qv, qth, qom, r; };

// 4×4-Matrixhelfer (nur hier gebraucht)
namespace detail {
using M4 = double[4][4];
inline void mul(const M4 A, const M4 B, M4 C) {
  M4 T;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      double s = 0;
      for (int k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      T[i][j] = s;
    }
  memcpy(C, T, sizeof(T));
}
inline void transp(const M4 A, M4 At) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) At[j][i] = A[i][j];
}
inline bool inv(const M4 A, M4 Ai) {  // Gauss-Jordan mit Pivotsuche
  double M[4][8];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) { M[i][j] = A[i][j]; M[i][j + 4] = (i == j); }
  for (int c = 0; c < 4; c++) {
    int p = c;
    for (int r = c + 1; r < 4; r++) if (fabs(M[r][c]) > fabs(M[p][c])) p = r;
    if (fabs(M[p][c]) < 1e-300) return false;
    if (p != c) for (int j = 0; j < 8; j++) { double t = M[c][j]; M[c][j] = M[p][j]; M[p][j] = t; }
    double d = M[c][c];
    for (int j = 0; j < 8; j++) M[c][j] /= d;
    for (int r = 0; r < 4; r++)
      if (r != c) {
        double f = M[r][c];
        for (int j = 0; j < 8; j++) M[r][j] -= f * M[c][j];
      }
  }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) Ai[i][j] = M[i][j + 4];
  return true;
}
}  // namespace detail

// Diskretes Modell [x v th om], Eingang a. Linearisiert um th = 0 (oben): th_dd = (g·th − a)/ell.
// Exakt diskretisiert (Halteglied 0. Ordnung), daher cosh/sinh der instabilen Pendelbewegung.
inline void discreteModel(double ell, double Ts, double A[4][4], double B[4]) {
  const double lam = sqrt(9.80665 / ell);
  const double ch = cosh(lam * Ts), sh = sinh(lam * Ts);
  double Ad[4][4] = {{1, Ts, 0, 0}, {0, 1, 0, 0}, {0, 0, ch, sh / lam}, {0, 0, lam * sh, ch}};
  memcpy(A, Ad, sizeof(Ad));
  B[0] = 0.5 * Ts * Ts;
  B[1] = Ts;
  B[2] = -(ch - 1.0) / (lam * lam * ell);
  B[3] = -sh / (lam * ell);
}

// Liefert K mit a = -K * [x v th om]. true bei Erfolg.
inline bool designLQR(float ell, float Ts, const LQRWeights& w, float Kout[4]) {
  using namespace detail;
  double A[4][4], B[4];
  discreteModel(ell, Ts, A, B);
  double Ak[4][4], Gk[4][4], Hk[4][4];
  memcpy(Ak, A, sizeof(Ak));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      Gk[i][j] = B[i] * B[j] / w.r;
      Hk[i][j] = 0;
    }
  Hk[0][0] = w.qx; Hk[1][1] = w.qv; Hk[2][2] = w.qth; Hk[3][3] = w.qom;

  // Doubling-Iteration: Hk konvergiert gegen die Lösung P der diskreten Riccati-Gleichung
  for (int it = 0; it < 60; it++) {
    double GH[4][4], W[4][4], Wi[4][4], WiA[4][4], AkT[4][4], T1[4][4], T2[4][4];
    mul(Gk, Hk, GH);
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) W[i][j] = (i == j) + GH[i][j];
    if (!inv(W, Wi)) return false;
    transp(Ak, AkT);
    mul(Wi, Ak, WiA);                       // W^-1 A
    double Gn[4][4], Hn[4][4], An[4][4];
    mul(Ak, WiA, An);                       // A W^-1 A
    mul(Wi, Gk, T1); mul(T1, AkT, T2); mul(Ak, T2, T1);   // A W^-1 G A'
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) Gn[i][j] = Gk[i][j] + T1[i][j];
    mul(Hk, WiA, T1); mul(AkT, T1, T2);    // A' H W^-1 A
    double dn = 0, hn = 0;
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        Hn[i][j] = Hk[i][j] + T2[i][j];
        dn += fabs(Hn[i][j] - Hk[i][j]);
        hn += fabs(Hn[i][j]);
      }
    memcpy(Ak, An, sizeof(Ak)); memcpy(Gk, Gn, sizeof(Gk)); memcpy(Hk, Hn, sizeof(Hk));
    if (dn <= 1e-13 * hn) break;
  }
  // K = (R + B'PB)^-1 B'PA
  double PB[4], PA_row[4] = {0, 0, 0, 0}, s = w.r;
  for (int i = 0; i < 4; i++) { PB[i] = 0; for (int j = 0; j < 4; j++) PB[i] += Hk[i][j] * B[j]; }
  for (int i = 0; i < 4; i++) s += B[i] * PB[i];
  for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++) PA_row[j] += PB[i] * A[i][j];
  for (int j = 0; j < 4; j++) {
    Kout[j] = (float)(PA_row[j] / s);
    if (!isfinite(Kout[j])) return false;
  }
  return true;
}

}  // namespace pc
