// controller.h – Gesamtregler: Modi, Balancieren, Swing-up, Beruhigen, Sicherheit, Wagen-Servo
// Teil des Regelungskerns (pendel_core.h), plattformunabhängig, auch in der Host-Simulation genutzt.
//
// Aufbau eines Regelschritts (step(), 800 Hz):
//   1. Schätzung: EKF (ekf.h) mit dem IMU-Paket, dann Vorausrechnen über die Funklatenz -> th_c, om_c
//   2. Sicherheit: Endlage, Bremsweg; Soll-Mitte xs als Trapezprofil zum Schieberegler-Wert x_set nachführen
//   3. Äußere Regelung je Modus -> Sollbeschleunigung a des Wagens
//        HOLD/WARTET/PERIOD: Wagen zur Soll-Mitte;  BALANCE: LQR a = -K·[x-xs, v, th, om]
//        SWINGUP: Energieregelung;  DAMP: Energie entziehen;  STOPPING: bremsen
//   4. Innere Regelung (Wagen-Servo): a wird zu einer Referenzbahn (x_ref, v_ref) integriert, ein PD-Regler
//      mit Vorsteuerung m_eff·a hält den Wagen darauf -> Sollkraft F (Rückgabewert)
// Winkel: th = 0 oben, pi unten; thw = auf -pi..pi gefaltet. Einheiten SI (m, s, rad, N).
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "lqr.h"
#include "ekf.h"
#include "period_meter.h"

namespace pc {

// ============================================================================
// Gesamtregler
// ============================================================================
// Alle Parameter des Reglers; gefüllt in setup() (Pendel_bldc.ino) aus config.h und NVS
struct CtrlConfig {
  float Ts;               // Abtastzeit = IMU-Periode [s]
  int latency;            // Funk-/Messlatenz in Abtastschritten
  EKFParams ekf;   // Pendelmodell + Rauschannahmen des Schätzers
  LQRWeights lqr;   // Gewichte für den LQR-Entwurf (lqr.h)
  // Wagen-Servo (unterlagerte Positions-/Geschwindigkeitsregelung)
  float m_eff;            // effektive Masse inkl. reflektierter Rotorträgheit [kg]
  float servo_wn;         // Servo-Eigenfrequenz [rad/s]
  float servo_zeta;
  float servo_emax;       // max. Folgefehler, bevor Referenz nachgezogen wird [m]
  float f_coulomb;        // Haftreibungs-Vorsteuerung [N]
  float f_max;            // Kraftbegrenzung [N]
  // Grenzen
  float x_soft;           // Swing-up/LQR-Sollbereich [m]
  float x_hard;           // Notstopp [m]
  float a_max;            // max. Beschleunigung [m/s^2]
  float v_max;            // max. Geschwindigkeit [m/s]
  // Swing-up
  // Energie-Verstärkung, Zentrierung (x, v), max. Beschleunigung, Fangbereich (Winkel, Winkelrate)
  float su_kE, su_kx, su_kv, su_amax, su_catch_th, su_catch_om;
  float su_Emargin;       // Energieüberschuss-Ziel [m^2/s^2], macht Swing-up robust gegen ell-Fehler
  float su_vmax;          // [m/s] Geschwindigkeitsgrenze im Swing-up (begrenzt die Motorleistung)
  float bal_lost_th;      // Balance verloren ab |th| > ...
  int max_lost;           // max. aufeinanderfolgende fehlende Pakete
  // Balancierpunkt-Abgleich: konstanter Winkelfehler (Accel-Offset, Montage) -> Wagen steht versetzt.
  // trim wird aus der Wagenposition nachgeführt, bis der Wagen im Mittel auf x = 0 balanciert.
  float trim_tau;         // Zeitkonstante [s], 0 = aus
  float trim_max;         // Begrenzung [rad]
  float om_lp_hz;         // Tiefpass (2. Ordnung) auf om für die Balancier-Rückführung, 0 = aus
  float rest_th;          // [rad] Ruhezone: Motor stromlos, solange |Winkel| darunter bleibt (0 = aus)
  bool rest_hold = true;  // Ruhezone: Wagen per Servo festhalten (true) statt stromlos (false, braucht Wagenreibung)
  float rest_x = 0.015f;  // [m] Ruhezone nur, wenn der Wagen so nah an der Soll-Mitte steht (Austritt +10 mm)
};

// Betriebsarten (Zahlenwert geht in die Telemetrie, index.html kennt dieselbe Reihenfolge)
enum class Mode : uint8_t { IDLE, HOLD, SWINGUP, BALANCE, STOPPING, FAULT, PERIOD, ARMED, DAMP };

inline const char* modeName(Mode m) {
  switch (m) {
    case Mode::IDLE: return "IDLE";
    case Mode::HOLD: return "HOLD";
    case Mode::SWINGUP: return "SWINGUP";
    case Mode::BALANCE: return "BALANCE";
    case Mode::STOPPING: return "STOPPING";
    case Mode::FAULT: return "FAULT";
    case Mode::PERIOD: return "PERIOD";
    case Mode::ARMED: return "WARTET";
    case Mode::DAMP: return "BERUHIGEN";
  }
  return "?";
}

// Pendel-IMU in Pendelkoordinaten: Drehrate [rad/s], Beschleunigung radial/tangential [m/s²] (sensors.h)
struct ImuMeas { float gyro, fr, ft; };

class Controller {
 public:
  CtrlConfig cfg{};
  EKF ekf;
  PeriodMeter period;
  float K[4] = {0, 0, 0, 0};
  Mode mode = Mode::IDLE;
  const char* faultMsg = "";
  bool autoReswing = true;
  // Hand-Modus: nur Winkelregelung (keine Positions-/Geschwindigkeitsrückführung). Der Wagen fährt dann
  // so, dass das Pendel senkrecht steht – auch wenn man die Spitze festhält. Er bleibt nicht in der Mitte.
  bool handMode = false;
  // Zeitlupe (Richtungstest): kein LQR. Der Wagen fährt langsam in die Richtung, in die die Spitze geneigt ist
  // (v_soll = slow_k * Winkel). Pendel bleibt in der Hand; zeigt nur, ob die Richtungen stimmen.
  bool slowMode = false;
  float slow_k = 0.6f;           // [m/s pro rad]: 10° -> ~0,1 m/s
  float slow_vmax = 0.12f;       // [m/s]
  float slow_amax = 0.6f;        // [m/s^2]
  float slow_tau = 0.25f;        // [s] Anpassung der Geschwindigkeit
  float slow_lost = 0.35f;       // [rad] ~20°: darüber anhalten
  float trim = 0;                // Winkel-Nullpunktkorrektur [rad], wird von außen gespeichert/geladen
  float x_set = 0;               // Soll-Mittelposition [m] (Schieberegler), wird begrenzt
  float xs = 0;                  // rampenförmig nachgeführte Soll-Position [m]
  float xs_rate = 0.10f;         // max. Verschiebegeschwindigkeit der Soll-Mitte [m/s]
  float xs_acc = 0.20f;          // max. Beschleunigung der Soll-Mitte [m/s^2] (Trapezprofil)
  float xs_v = 0;                // aktuelle Geschwindigkeit der Soll-Mitte [m/s]
  float xs_max = 0.35f;          // max. |Soll-Position| [m], zusätzlich <= x_hard - 30 mm
  float balTime = 0;             // Zeit seit dem Fangen [s]
  float armTime = 0;             // Zeit ruhig senkrecht im Wartezustand [s]
  bool softStart = false;        // Start aus WARTET: Verstärkung von soft_k0 auf 1 in soft_t hochfahren
  float soft_k0 = 0.5f, soft_t = 1.0f;

  // Diagnose
  float th_c = PI_F, om_c = 0;   // latenzkompensierte Schätzung
  float a_cmd = 0, F_cmd = 0, x_ref = 0, v_ref = 0;
  float om_f = 0;                // gefiltertes om (Diagnose)
  float thCorr = 0;              // Winkelkorrektur des letzten Messupdates [rad] (Diagnose)
  float aMax = 0, vMax = 0, iqMax = 0;          // Spitzenwerte seit Moduswechsel (Diagnose, vom Regeltask gesetzt)
  int lostCnt = 0, lostMax = 0, gapsLong = 0;   // lostMax: längste Lücke seit Moduswechsel (Diagnose)

  bool begin(const CtrlConfig& c) {
    cfg = c;
    ekf.init(c.ekf);
    memset(aHist, 0, sizeof(aHist));
    return redesign();
  }

  // LQR-Verstärkungen K aus Pendellänge und Gewichten neu berechnen
  bool redesign() { return designLQR(cfg.ekf.ell, cfg.Ts, cfg.lqr, K); }

  // neue Pendelparameter mit LQR-Neuentwurf (in der Simulation genutzt)
  void setPendulum(float ell, float damp) {
    float Kn[4];
    cfg.ekf.ell = ell;
    if (designLQR(ell, cfg.Ts, cfg.lqr, Kn)) applyPendulum(ell, damp, Kn);
  }
  // Übernahme eines extern (z. B. in einem anderen Task) berechneten K, ohne Neuberechnung
  void applyPendulum(float ell, float damp, const float Kn[4]) {
    cfg.ekf.ell = ell; cfg.ekf.c = damp;
    ekf.p.ell = ell; ekf.p.c = damp;
    for (int i = 0; i < 4; i++) K[i] = Kn[i];
  }

  // Statische Kalibrierung: Mittelwerte bei ruhig hängendem Pendel
  bool calibrate(float gyroMean, float frMean, float ftMean) {
    const float g = sqrtf(frMean * frMean + ftMean * ftMean);
    if (fabsf(g - G) > 0.6f) return false;
    ekf.reset(atan2f(-ftMean, frMean), gyroMean);
    calibrated = true;
    return true;
  }
  bool isCalibrated() const { return calibrated; }

  // ---- Kommandos ----
  void cmdIdle() { mode = Mode::IDLE; }
  void cmdHold(float x) { syncRef(x, 0); mode = Mode::HOLD; }   // braucht keinen Pendelwinkel
  // Hängendes Pendel beruhigen: Wagen zur Soll-Mitte und Schwingung aktiv dämpfen, danach HOLD
  void cmdDamp(float x, float v) {
    if (!calibrated) return fault("nicht kalibriert");
    syncRef(x, v); mode = Mode::DAMP; dampCalm = 0;
  }
  float damp_c = 1.0f;           // [m/s^2 pro rad/s] Dämpfung (Sim: 30-90° Ausschlag in 5-7 s ruhig, max. ~0,3 m Weg)
  float damp_kx = 3.0f, damp_kv = 3.0f;   // Zentrierung (langsamer als die Pendelschwingung)
  float damp_amax = 3.0f;        // [m/s^2]
  float damp_calm_t = 1.0f;      // [s] so lange ruhig, dann HALTEN
  float dampCalm = 0;            // Zeit ruhig [s]
  // Weckimpuls (nur in HOLD): kurzer Ruck, den die Bewegungserkennung des schlafenden Senders bemerkt
  volatile float nudgeT = -1;
  float nudge_a = 4.0f;          // [m/s^2] ~0,4 g an der Spitze, Weckschwelle ~0,05 g; Weg ~6 mm
  void nudge() { if (mode == Mode::HOLD) nudgeT = 0; }
  void cmdPeriod(float x) { if (!calibrated) return fault("nicht kalibriert"); syncRef(x, 0); period.start(); mode = Mode::PERIOD; }
  void cmdSwingup(float x, float v) {
    if (!calibrated) return fault("nicht kalibriert");
    syncRef(x, v); kick = (int)(0.06f / cfg.Ts); mode = Mode::SWINGUP;
  }
  void cmdBalance(float x, float v) {
    if (!calibrated) return fault("nicht kalibriert");
    syncRef(x, v); mode = Mode::ARMED; armTime = 0;     // wartet, bis das Pendel von Hand senkrecht steht
  }
  float arm_th = 0.05f;          // Start, wenn |th| < 3° ...
  float arm_om = 0.5f;           // ... und |om| < 0.5 rad/s ...
  float arm_t = 0.3f;            // ... für 0,3 s
  void cmdStop() { if (mode != Mode::IDLE && mode != Mode::FAULT) { mode = Mode::STOPPING; stopReason = "Stopp-Befehl"; } }
  const char* stopReason = "";
  void fault(const char* why) { faultMsg = why; mode = Mode::FAULT; }

  bool motorEnabled() const { return mode != Mode::IDLE && mode != Mode::FAULT; }

  // Ein Regelschritt pro IMU-Periode. imu==nullptr -> Paket fehlt (nur Prädiktion).
  // x, v: Wagenposition/-geschwindigkeit aus dem Motorencoder. Rückgabe: Sollkraft [N].
  float step(const ImuMeas* imu, float x, float v) {
    const float Ts = cfg.Ts;
    const int L = cfg.latency;

    // --- Schätzung (Messung ist L Schritte alt) ---
    // Die Wagenbeschleunigung beeinflusst das Pendel; der EKF braucht die, die zum (L Takte alten) Messzeitpunkt galt
    const float aMeas = aHist[(head + HIST - L) % HIST];   // a zum Messzeitpunkt
    ekf.predict(Ts, aMeas);
    if (imu) {
      lostCnt = 0;
      const float fmag = sqrtf(imu->fr * imu->fr + imu->ft * imu->ft);
      const float th0 = ekf.th;
      // Beschleunigung nur nutzen, wenn plausibel (< 4 g, sonst Stoß)
      ekf.correct(imu->gyro, imu->fr, imu->ft, aMeas, fmag < 4.0f * G);
      thCorr = ekf.th - th0;
    } else {
      if (++lostCnt > lostMax) lostMax = lostCnt;
      if (lostCnt == 40) gapsLong++;                 // Lücke >= 50 ms
      if (lostCnt > (slowMode ? 400 : cfg.max_lost) &&             // Zeitlupe: 0,5 s Toleranz
          (mode == Mode::SWINGUP || mode == Mode::BALANCE || mode == Mode::PERIOD || mode == Mode::DAMP))
        fault("Funkverbindung verloren");          // HOLD/STOPPING/WARTET brauchen den Pendelwinkel nicht
    }
    // Vorwärtsprädiktion über die Latenz mit den seither kommandierten a
    float th = ekf.th, om = ekf.om;
    for (int k = L - 1; k >= 0; k--) {
      const float a = aHist[(head + HIST - k) % HIST];
      const float od = ekf.omDot(th, om, a);
      th += om * Ts + 0.5f * od * Ts * Ts;
      om += od * Ts;
    }
    th_c = th; om_c = om;
    const float thw = wrapPi(th);
    // Tiefpass auf om für die Rückführung: Vibrationen der Pendelspitze (BMI270-Rauschen, Strukturmoden)
    // sollen nicht über K[3] direkt in die Wagenbeschleunigung gehen
    float omf = om;
    if (cfg.om_lp_hz > 0) {
      if (lpCoefTs != Ts || lpCoefHz != cfg.om_lp_hz) lpDesign(cfg.om_lp_hz, Ts);
      const float y = lb0 * om + lz1;
      lz1 = lb1 * om - la1 * y + lz2;
      lz2 = lb2 * om - la2 * y;
      omf = y;
    }
    om_f = omf;

    // Periodenmessung ('p'): Nulldurchgänge der freien Schwingung zählen (period_meter.h)
    if (period.done == false && mode == Mode::PERIOD) {
      period.feed(Ts, ekf.om, wrapPi(ekf.th - PI_F));
      if (period.done || period.timeout() > 30.0f) mode = Mode::HOLD;
    }

    // --- Sicherheit ---
    if (motorEnabled() && fabsf(x) > cfg.x_hard) {
      // HOLD/WARTET dürfen aus dem Notstopp-Bereich langsam zur Mitte zurückfahren (z. B. nach einem FAULT)
      const bool backIn = (mode == Mode::HOLD || mode == Mode::ARMED) && fabsf(x) < cfg.x_hard + 0.05f &&
                          x * v < 0.02f * fabsf(x);
      if (!backIn) fault("Endlage ueberschritten");
    }
    // Bremsweg-Überwachung (alle Fahrmodi): passt der Bremsweg mit 70 % a_max nicht mehr vor die Notstopp-Grenze,
    // sofort kontrolliert bremsen
    if (motorEnabled() && mode != Mode::STOPPING && x * v > 0 &&
        fabsf(x) + v * v / (2.0f * 0.7f * cfg.a_max) > cfg.x_hard - 0.01f) {
      mode = Mode::STOPPING;
      stopReason = "Bremsweg zur Endlage";
    }

    // Soll-Position (Schieberegler): begrenzen und rampenförmig nachführen
    {
      const float lim = fmaxf(0.0f, fminf(xs_max, cfg.x_hard - 0.03f));
      const float tgt = clampf(x_set, -lim, lim), d = tgt - xs;
      // Trapezprofil: beschleunigen mit xs_acc bis xs_rate, rechtzeitig bremsen (Bremsweg v²/2a)
      const float vWant = copysignf(fminf(xs_rate, sqrtf(2.0f * xs_acc * fabsf(d))), d);
      xs_v += clampf(vWant - xs_v, -xs_acc * Ts, xs_acc * Ts);
      xs += xs_v * Ts;
      if (fabsf(d) < 1e-4f && fabsf(xs_v) < 0.01f) { xs = tgt; xs_v = 0; }   // angekommen
    }

    // --- äußere Regelung: Sollbeschleunigung ---
    float a = 0;
    switch (mode) {
      case Mode::IDLE:
      case Mode::FAULT:
        syncRef(x, v);
        pushA(0);
        a_cmd = 0; F_cmd = 0;
        return 0;

      case Mode::ARMED:
        if (lostCnt > cfg.max_lost) armTime = -0.7f;                 // Pendel (noch) nicht da: warten, danach 1 s für den EKF
        else if (fabsf(x - xs) > 0.03f) armTime = fminf(armTime, 0.0f);   // erst zur Soll-Mitte fahren
        else if (fabsf(thw) < arm_th && fabsf(om) < arm_om) armTime += Ts;
        else armTime = fminf(armTime, 0.0f);
        if (armTime >= arm_t) { syncRef(x, v); mode = Mode::BALANCE; balTime = 0; softStart = true; goto balance; }
        [[fallthrough]];   // Wagen hält bis dahin seine Position
      case Mode::HOLD:
      case Mode::PERIOD: {
        if (nudgeT >= 0) {               // Weckimpuls für das schlafende Pendel: +a / -a / +a, netto ~0 Geschwindigkeit
          nudgeT += Ts;
          a = nudgeT < 0.04f ? nudge_a : nudgeT < 0.12f ? -nudge_a : nudgeT < 0.16f ? nudge_a : 0.0f;
          if (nudgeT >= 0.16f) nudgeT = -1;
          break;
        }
        const float ex = x_ref - xs;                                 // langsam zur Soll-Position ziehen
        a = clampf(-25.0f * ex - 10.0f * v_ref, -2.0f, 2.0f);
        if (fabsf(ex) < 0.002f && fabsf(v_ref) < 0.01f) a = -10.0f * v_ref;
        break;
      }

      case Mode::DAMP: {
        // phi = Auslenkung aus der unteren Ruhelage: phi_dd = (-g sin(phi) + a cos(phi)) / ell
        // a = -c * om * cos(phi) entzieht Energie (wirkt wie Reibung), dazu langsam zur Soll-Mitte
        const float ph = wrapPi(thw - PI_F);
        const float cph = cosf(ph);
        float ad = -damp_c * om * cph;
        // nach außen nur so viel, wie Platz ist: im letzten Stück vor dem Arbeitsbereichsende ausblenden
        if (ad * x > 0) ad *= clampf((cfg.x_soft - 0.03f - fabsf(x)) / 0.10f, 0.0f, 1.0f);
        a = ad - damp_kx * (x - xs) - damp_kv * v;
        a = clampf(a, -damp_amax, damp_amax);
        // ruhig = keine Schwingung mehr: 0,5 s lang |omega| < 0,03 rad/s heißt Amplitude < ~0,25°.
        // Bewusst ohne |phi|-Bedingung: ein kleiner Winkel-Offset im Hängen (Kalibrierung) ist keine Bewegung
        // und verhinderte vorher den Übergang zu HALTEN (gemessen: hing ruhig bei 1,1-1,6°).
        const bool calm = fabsf(om) < 0.03f && fabsf(x - xs) < 0.01f && fabsf(v) < 0.02f;
        dampCalm = calm ? dampCalm + Ts : 0.0f;
        if (dampCalm > damp_calm_t) { syncRef(x, v); mode = Mode::HOLD; }   // ruhig -> Position halten
        break;
      }

      case Mode::STOPPING:
        a = clampf(-v / 0.08f, -cfg.a_max, cfg.a_max);
        if (fabsf(v) < 0.02f) { mode = Mode::IDLE; syncRef(x, 0); pushA(0); a_cmd = 0; F_cmd = 0; return 0; }
        break;

      case Mode::SWINGUP: {
        // Fangen nur, wenn das Pendel auf die Senkrechte ZU läuft (oder fast steht). Ein Pendel, das schon über
        // den Punkt hinaus ist und wegläuft, ist mit dem verfügbaren Weg kaum zu halten (gemessen: Fang bei -14°,
        // 1,9 rad/s weg -> Wagen 1,4 m/s -> Endlage). Dann lieber weiter schwingen, es kommt zurück.
        const bool toward = thw * om <= 0.0f || fabsf(om) < 0.8f;
        if (fabsf(thw) < cfg.su_catch_th && fabsf(om) < cfg.su_catch_om && toward) { mode = Mode::BALANCE; balTime = 0; softStart = false; goto balance; }
        // Energie je Masse und Länge: E = ½·ℓ·ω² + g·(cos th − 1); 0 = oben in Ruhe, unten in Ruhe −2g.
        // Beschleunigt der Wagen mit a ~ (E−Ziel)·ω·cos th, steigt E bis zum Ziel (klassisches Energie-Pumpen nach Åström).
        const float E = 0.5f * cfg.ekf.ell * om * om + G * (cosf(th) - 1.0f);   // 0 = oben in Ruhe
        const float s = om * cosf(th);
        a = cfg.su_kE * (E - cfg.su_Emargin) * s / (fabsf(s) + 0.5f);          // weiches sign()
        a = clampf(a, -cfg.su_amax, cfg.su_amax);
        if (kick > 0) { kick--; a = cfg.su_amax * 0.6f; }   // Anstoß beim Start: hängt das Pendel still, ist ω = 0 und das Pumpen käme nie in Gang
        a += -cfg.su_kx * x - cfg.su_kv * v;                // Wagen zentrieren
        // Bremsweg-Begrenzung zur Endlage
        const float room = cfg.x_soft - fabsf(x);
        if (v * x > 0 && v * v > 2.0f * cfg.su_amax * fmaxf(room, 0.0f)) a = -cfg.su_amax * (v > 0 ? 1 : -1);
        a = clampf(a, -cfg.su_amax, cfg.su_amax);
        if (cfg.su_vmax > 0 && fabsf(v_ref) >= cfg.su_vmax && a * v_ref > 0) a = 0;   // Leistung begrenzen
        break;
      }

      case Mode::BALANCE:
      balance: {
        if (slowMode) {
          balTime += Ts;
          const float thb = thw - trim;
          if (fabsf(thb) > slow_lost) { mode = Mode::STOPPING; a = 0; break; }
          float vt = clampf(slow_k * thb, -slow_vmax, slow_vmax);     // +th (Spitze Richtung +x) -> Wagen nach +x
          if (fabsf(thb) < 0.01f || lostCnt > cfg.max_lost) vt = 0;   // Totzone ~0,6°; ohne Daten anhalten
          const float lim = cfg.x_soft - 0.03f;                       // vor der Endlage nur noch nach innen
          if ((x > lim && vt > 0) || (x < -lim && vt < 0)) vt = 0;
          a = clampf((vt - v_ref) / slow_tau, -slow_amax, slow_amax);
          break;
        }
        if (fabsf(thw) > cfg.bal_lost_th) {
          if (autoReswing) { mode = Mode::SWINGUP; kick = 0; a = 0; break; }
          mode = Mode::STOPPING; stopReason = "Pendel umgefallen"; a = 0; break;
        }
        balTime += Ts;
        if (balTime < 1.5f * Ts) { resting = false; restCalm = 0; }   // neu im Balancieren
        const float thb = thw - trim;
        const float exb = x - xs;
        // Ruhezone: steht das Pendel (z. B. durch Lagerreibung) ruhig senkrecht und der Wagen in der Mitte,
        // ruht die Winkelregelung: Wagen wird festgehalten (rest_hold) oder stromlos. Verlässt der Winkel die Zone, übernimmt der Regler.
        if (cfg.rest_th > 0 && !handMode) {
          if (!resting) {
            const bool calm = fabsf(thb) < 0.7f * cfg.rest_th && fabsf(omf) < 0.02f && fabsf(v) < 0.005f &&
                              fabsf(exb) < cfg.rest_x && balTime > 5.0f;
            restCalm = calm ? restCalm + Ts : 0.0f;
            if (restCalm > 0.5f) { resting = true; restCount++; syncRef(x, 0); }   // Halteposition = hier
          } else if (fabsf(thb) > cfg.rest_th || fabsf(omf) > 0.05f || fabsf(exb) > cfg.rest_x + 0.01f) {
            resting = false; restCalm = 0;
            syncRef(x, v);                               // Servo ohne Sprung wieder aufnehmen
          }
          if (resting) {
            restTime += Ts;
            if (!cfg.rest_hold) {                        // stromlos: Wagen hält nur durch seine Reibung
              syncRef(x, v);
              a_cmd = 0; pushA(0); F_cmd = 0;
              return 0;
            }
            a = 0; v_ref = 0;                            // Wagen-Servo hält die Position (wie eine Bremse),
            break;                                       // keine Winkelregelung; Strom nur bei Störung
          }
        }
        // LQR: Sollbeschleunigung aus Positionsfehler, Geschwindigkeit, Winkel (abzüglich trim) und gefilterter Winkelrate
        a = handMode ? -(K[2] * thb + K[3] * omf)
                     : -(K[0] * exb + K[1] * v + K[2] * thb + K[3] * omf);
        if (softStart) a *= fminf(1.0f, soft_k0 + (1.0f - soft_k0) * balTime / soft_t);   // sanft aus der Hand übernehmen
        // kontrolliert bremsen, bevor die Endlage (FAULT) erreicht wird
        // (früher Stopp schon bei x_soft + 50 %: brach auch ruhiges Zurückfangen ab; jetzt nur noch die
        //  Bremsweg-Überwachung oben, die rechtzeitig vor der Notstopp-Grenze bremst)
        a = clampf(a, -cfg.a_max, cfg.a_max);
        // Abgleich nur im ruhigen Balancieren (nicht direkt nach dem Fangen, nicht bei Stößen)
        if (!handMode && cfg.trim_tau > 0 && balTime > 2.0f && fabsf(thb) < 0.05f && fabsf(v) < 0.15f && fabsf(x) < cfg.x_soft &&
            fabsf(xs - x_set) < 1e-4f) {
          const float r = K[2] / K[0];                      // Versatz x = -r * (Winkelfehler - trim)
          if (r > 0.1f) trim = clampf(trim - exb * Ts / (cfg.trim_tau * r), -cfg.trim_max, cfg.trim_max);
        }
        break;
      }
    }

    // Geschwindigkeitsbegrenzung
    if (v_ref >= cfg.v_max && a > 0) a = 0;
    if (v_ref <= -cfg.v_max && a < 0) a = 0;
    a_cmd = a;
    pushA(a);

    // --- Referenzintegrator + Wagen-Servo ---
    // Referenzbahn = doppelt integrierte Sollbeschleunigung; zu großer Folgefehler -> Referenz nachziehen (kein Aufschaukeln)
    x_ref += v_ref * Ts + 0.5f * a * Ts * Ts;
    v_ref += a * Ts;
    const float ex = x_ref - x;
    if (fabsf(ex) > cfg.servo_emax) x_ref = x + (ex > 0 ? cfg.servo_emax : -cfg.servo_emax);
    const float wn = cfg.servo_wn, z = cfg.servo_zeta;
    // Kraft = Masse × (Soll-a + PD auf den Bahnfehler) + Haftreibungs-Vorsteuerung in Fahrtrichtung
    float F = cfg.m_eff * (a + wn * wn * (x_ref - x) + 2.0f * z * wn * (v_ref - v));
    F += cfg.f_coulomb * clampf(v_ref / 0.02f, -1.0f, 1.0f);
    F_cmd = clampf(F, -cfg.f_max, cfg.f_max);
    return F_cmd;
  }

 private:
  static constexpr int HIST = 16;
  float aHist[HIST];   // letzte Sollbeschleunigungen (für Latenzausgleich im EKF)
  int head = 0;
  int kick = 0;   // Swing-up-Anstoß, verbleibende Takte
  bool calibrated = false;
 public:
  bool resting = false;          // Ruhezone aktiv (Wagen gehalten bzw. stromlos, siehe rest_hold)
  float restCalm = 0, restTime = 0;   // Zeit ruhig vor Eintritt / Gesamtzeit in der Ruhezone [s]
  uint32_t restCount = 0;   // Anzahl Eintritte (Status)
 private:

  // Butterworth-Tiefpass 2. Ordnung (bilinear)
  float lb0 = 1, lb1 = 0, lb2 = 0, la1 = 0, la2 = 0, lz1 = 0, lz2 = 0, lpCoefTs = 0, lpCoefHz = 0;
  void lpDesign(float hz, float Ts) {
    const float K = tanf(PI_F * hz * Ts), q = 0.70710678f, n = 1.0f / (1.0f + K / q + K * K);
    lb0 = K * K * n; lb1 = 2.0f * lb0; lb2 = lb0;
    la1 = 2.0f * (K * K - 1.0f) * n; la2 = (1.0f - K / q + K * K) * n;
    lpCoefTs = Ts; lpCoefHz = hz; lz1 = lz2 = 0;
  }
  // Sollbeschleunigung in den Verlauf; Referenz auf den Istwert setzen (stoßfreier Moduswechsel)
  void pushA(float a) { head = (head + 1) % HIST; aHist[head] = a; }
  void syncRef(float x, float v) { x_ref = x; v_ref = v; }
};

}  // namespace pc
