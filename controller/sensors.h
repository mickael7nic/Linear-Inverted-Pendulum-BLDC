// sensors.h – Messwerte: Wagenposition, IMU-Umrechnung, Pendel-Kalibrierung
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Koordinaten: x = Wagenposition [m], 0 = Schienenmitte (vom Homing), + Richtung siehe X_SIGN in config.h.
// Pendelwinkel th: 0 = senkrecht oben, pi = hängend. Die IMU sitzt im Pendel; ihre Achsen werden auf
//   fr = Beschleunigung längs des Pendels (radial), ft = quer dazu (tangential), gyro = Drehrate um die Achse
// umgerechnet. Der Einbauwinkel der Platine (calDelta) wird mit 'c' gemessen und hier herausgedreht.
#pragma once

// Wagenposition/-geschwindigkeit aus dem Motorwinkel: Weg = Winkel × Radius der Riemenscheibe
static inline float cartX() { return X_SIGN * (motor.shaft_angle - xZeroAngle) * PULLEY_RADIUS_M; }
static inline float cartV() { return X_SIGN * motor.shaft_velocity * PULLEY_RADIUS_M; }
static inline float rawX() { return X_SIGN * motor.shaft_angle * PULLEY_RADIUS_M; }   // ohne Nullpunkt

// Rohdaten -> Pendelkoordinaten inkl. Montagewinkel-Korrektur (Drehung um -calDelta)
static pc::ImuMeas convert(const ImuPacket& p) {
  // Rohwert -> m/s² bzw. rad/s; Achsenzuordnung und Vorzeichen aus config.h (AX_R_*, AX_T_*, GYR_*)
  const float ka = pc::G / ACC_LSB_PER_G;
  const float kg = (float)M_PI / 180.0f / GYR_LSB_PER_DPS;
  const float frm = AX_R_SIGN * p.acc[AX_R_IDX] * ka;
  const float ftm = AX_T_SIGN * p.acc[AX_T_IDX] * ka;
  const float d = calDelta - angOfs;           // Offset: gemessener Winkel = kalibrierter Winkel + angOfs
  const float cd = cosf(d), sd = sinf(d);
  pc::ImuMeas m;
  m.fr = frm * cd - ftm * sd;
  m.ft = frm * sd + ftm * cd;
  m.gyro = GYR_SIGN * p.gyr[GYR_IDX] * kg;
  return m;
}

// 5-s-Kalibrierung in 0,5-s-Blöcken; ein unruhiger Block startet die Zählung neu
// Pendel-Kalibrierung ('c'): Pendel hängt ruhig. Aus der mittleren Beschleunigung folgt der Einbauwinkel der IMU
// (Schwerkraft zeigt dann genau längs des Pendels), aus der mittleren Drehrate der Gyro-Offset (Bias).
struct Calib {
  static constexpr int BLOCK = (int)(IMU_RATE_HZ / 2);
  static constexpr int NEED = (int)(CAL_TIME_S * 2.0f + 0.5f);
  bool active = false;
  int okBlocks = 0, total = 0, bn = 0;
  double bg = 0, bg2 = 0, bfr = 0, bft = 0;   // laufender Block
  double sg = 0, sfr = 0, sft = 0; int sn = 0; // ruhige Blöcke
  void start() { active = true; okBlocks = total = 0; resetBlock(); sg = sfr = sft = 0; sn = 0; }
  void resetBlock() { bn = 0; bg = bg2 = bfr = bft = 0; }
} calib;

// Schnell-Init des Schätzers (0,25 s) mit gespeicherter Kalibrierung
struct EstInit {
  int n = 0;
  double fr = 0, ft = 0;
} estInit;

// Je IMU-Sample während der Kalibrierung aufgerufen (Regel-Task). Ergebnis über evtCalib (UI-Task loggt/speichert).
static void calibSample(const pc::ImuMeas& m) {
  calib.total++;
  calib.bn++;
  calib.bg += m.gyro; calib.bg2 += (double)m.gyro * m.gyro; calib.bfr += m.fr; calib.bft += m.ft;
  if (calib.bn < Calib::BLOCK) {
    if (calib.total > CAL_TIMEOUT_S * IMU_RATE_HZ) { calib.active = false; evtCalib = -1; }
    return;
  }
  const double gm = calib.bg / calib.bn, gs = sqrt(fmax(0.0, calib.bg2 / calib.bn - gm * gm));
  const double fr = calib.bfr / calib.bn, ft = calib.bft / calib.bn, g = sqrt(fr * fr + ft * ft);
  // Block ruhig? Drehrate kaum gestreut und Betrag der Beschleunigung ~1 g (keine Bewegung)
  const bool still = gs < CAL_GYRO_STD && fabs(g - pc::G) < 0.5;
  if (still) {
    calib.sg += calib.bg; calib.sfr += calib.bfr; calib.sft += calib.bft; calib.sn += calib.bn;
    calib.okBlocks++;
  } else {
    if (calib.okBlocks > 0) evtCalibMoved = true;
    calib.okBlocks = 0; calib.sg = calib.sfr = calib.sft = 0; calib.sn = 0;
  }
  calib.resetBlock();
  // genug ruhige Blöcke am Stück -> Ergebnis berechnen
  if (calib.okBlocks >= Calib::NEED) {
    calib.active = false;
    const float b = (float)(calib.sg / calib.sn);
    const float frm = (float)(calib.sfr / calib.sn), ftm = (float)(calib.sft / calib.sn);
    const float off = pc::wrapPi(atan2f(-ftm, frm) - pc::PI_F);   // Restwinkel im korrigierten System
    const float dNew = pc::wrapPi(calDelta - angOfs + off);        // Kalibrierung ohne Offset; Offset bleibt erhalten
    if (fabsf(dNew) > CAL_MAX_OFFSET) { evtCalib = -1; return; }
    calDelta = dNew; calBias = b;
    calNewDelta = dNew; calNewBias = b;
    {
      const float gm = sqrtf(frm * frm + ftm * ftm);
      ctrl.calibrate(b, -gm * cosf(angOfs), gm * sinf(angOfs));     // hängend: th = pi + Offset
    }
    evtCalib = 1;
  } else if (calib.total > CAL_TIMEOUT_S * IMU_RATE_HZ) {
    calib.active = false; evtCalib = -1;
  }
}
