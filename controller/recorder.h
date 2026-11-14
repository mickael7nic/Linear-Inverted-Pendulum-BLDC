// recorder.h – Hochraten-Rekorder (800 Hz / REC_DECIM, letzte REC_N Einträge, übersteht Neustarts)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Ringpuffer mit den letzten 4 s (1600 Einträge à 400 Hz) aller Regelgrößen. Zweck: nach einem Absturz des
// Pendels, einem Fehler oder einem Neustart genau sehen, was davor passiert ist (Taste 'D' / Button "Rekorder"
// -> CSV in der App). Der Puffer liegt in .noinit-RAM und überlebt deshalb sogar Abstürze und Watchdog-Resets.
// "Einfrieren": 0,3 s nach dem Ende von Balancieren/Beruhigen (oder einem Winkelsprung) wird nicht mehr
// überschrieben, bis wieder Balancieren/Swing-up/Beruhigen startet -> das interessante Ende bleibt erhalten.
#pragma once

// Ein Eintrag (25 Byte): skalierte Ganzzahlen, damit 1600 Einträge in ~40 kB passen
struct __attribute__((packed)) RecS {
  uint16_t n;                       // Schrittzähler (1,25 ms)
  int16_t x, v, th, om, gy, fr, ft, a, iq, dc;   // 0,1 mm | mm/s | 0,01° | mrad/s | mrad/s | cm/s² | cm/s² | cm/s² | 10 mA | 0,001° Winkelkorrektur
  uint8_t mode, lost;               // lost: 1 = kein IMU-Paket in diesem Schritt
  uint8_t foc;                      // FOC-Schleifendurchläufe seit dem letzten Regeltakt
};
constexpr int REC_N = 1600;         // Einträge
#ifndef REC_DECIM
#define REC_DECIM 2                 // jeder 2. Regeltakt -> 400 Hz, 4 s (1 = 800 Hz, 2 s)
#endif
// .noinit: Inhalt übersteht Software-Neustart, Absturz und Watchdog -> Daten vor einem Neustart bleiben erhalten
struct RecStore { uint32_t magic; int head, count; RecS buf[REC_N]; };
static __NOINIT_ATTR RecStore recStore;
constexpr uint32_t REC_MAGIC = 0x52454333;   // bei Änderung von RecS hochzählen
static RecS* recBuf = nullptr;
static int& recHead = recStore.head;
static int& recCount = recStore.count;
static int recFreezeIn = -1;   // Countdown bis zum Einfrieren (in Regeltakten), -1 = nicht aktiv
static uint16_t recStep = 0;   // läuft mit jedem Regeltakt hoch (Zeitbasis, 1,25 ms)
// recFrozen: Puffer gesperrt; evt*: Meldungen für den UI-Task; recDumpReq: 'D' angefordert
static volatile bool recFrozen = false, evtRecFrozen = false, recDumpReq = false, evtRecJump = false;

// float -> skalierter int16 mit Begrenzung
static inline int16_t rq(float v, float k) { return (int16_t)pc::clampf(v * k, -32767, 32767); }
static void recRestart();
// Jeden Regeltakt aus runStep() (control_task.h). m = nullptr bei fehlendem Paket.
static void recPush(const pc::ImuMeas* m, float x, float v, pc::Mode before) {
  using pc::Mode;
  static Mode last = Mode::IDLE;               // eigener Vergleich: Moduswechsel per Befehl (STOPP, Swing-up)
  static uint8_t lostAcc = 0;                  // passieren außerhalb von ctrl.step() und wären in 'before' unsichtbar
  (void)before;
  const Mode now = ctrl.mode, prev = last;
  last = now;
  recStep++;
  // eingefrorene Aufnahme bleibt erhalten, bis wirklich neu balanciert bzw. aufgeschwungen wird
  if (recFrozen && now != prev && (now == Mode::BALANCE || now == Mode::SWINGUP || now == Mode::DAMP)) recRestart();
  if (!recBuf || recFrozen) return;
  // Ende von Balancieren (außer Wechsel zu Beruhigen, das wird weiter aufgezeichnet) bzw. von Beruhigen
  const bool ended = (prev == Mode::BALANCE && now != Mode::BALANCE && now != Mode::DAMP) ||
                     (prev == Mode::DAMP && now != Mode::DAMP) ||
                     (now == Mode::FAULT && prev != Mode::FAULT && prev != Mode::IDLE);
  // Winkelsprung: Messung weicht > ~4° von der Schätzung ab (20 begrenzte Updates in Folge) -> festhalten
  const bool jump = now == Mode::BALANCE && ctrl.ekf.clampCnt == 20;
  if (jump) evtRecJump = true;
  if ((ended || jump) && recFreezeIn < 0) recFreezeIn = (int)(0.3f * IMU_RATE_HZ);
  if (recFreezeIn >= 0 && --recFreezeIn < 0) { recFrozen = true; evtRecFrozen = true; }
  if (!m && lostAcc < 255) lostAcc++;
  if (recStep % REC_DECIM != 0) return;        // Ausdünnen (Zeit kommt aus n -> Web-App rechnet richtig)
  RecS& r = recBuf[recHead];
  r.n = recStep;
  r.x = rq(x, 1e4f); r.v = rq(v, 1e3f);
  r.th = rq(pc::wrapPi(ctrl.th_c), 5729.58f); r.om = rq(ctrl.om_c, 1e3f);
  r.gy = m ? rq(m->gyro, 1e3f) : 0; r.fr = m ? rq(m->fr, 100.0f) : 0; r.ft = m ? rq(m->ft, 100.0f) : 0;
  r.a = rq(ctrl.a_cmd, 100.0f); r.iq = rq(X_SIGN * motor.current.q, 100.0f);
  r.dc = m ? rq(ctrl.thCorr, 57295.8f) : 0;
  r.mode = (uint8_t)now | (ctrl.resting && ctrl.mode == pc::Mode::BALANCE ? 0x80 : 0); r.lost = lostAcc;   // Bit 7: Ruhezone aktiv (Motor stromlos)
  r.foc = focPerTick;   // lost: fehlende IMU-Pakete seit dem letzten Eintrag
  lostAcc = 0;
  recHead = (recHead + 1) % REC_N;
  if (recCount < REC_N) recCount++;
}
// Puffer leeren und wieder aufnehmen
static void recRestart() { recFrozen = false; recFreezeIn = -1; recCount = 0; recHead = 0; }
// Beim Start: Rekorder-Inhalt aus der Zeit vor einem Neustart (nicht nach Einschalten) einfrieren
static bool recInit() {
  recBuf = recStore.buf;
  const esp_reset_reason_t rr = esp_reset_reason();
  (void)rr;   // auch EN-Reset meldet POWERON; bei echtem Spannungsausfall ist der RAM-Inhalt ungültig (Magic)
  const bool keep = recStore.magic == REC_MAGIC && recCount > 0 && recCount <= REC_N &&
                    recHead >= 0 && recHead < REC_N;
  recStore.magic = REC_MAGIC;
  if (keep) { recFrozen = true; return true; }
  recRestart();
  return false;
}

// Ausgabe (UI-Task): nur über Serial + BLE, nicht in die Log-Historie
// Puffer als CSV-Zeilen "REC;..." ausgeben; index.html sammelt sie zwischen REC_BEGIN und REC_END ein.
static void recDump() {
  if (!recBuf || recCount == 0) { logf(">> Rekorder leer"); return; }
  if (ctrl.motorEnabled()) { logf(">> Rekorder: erst stoppen (x)"); return; }
  recFrozen = true;
  char line[120];
  int n = snprintf(line, sizeof(line), "REC_BEGIN;%d;%.0f\n", recCount, IMU_RATE_HZ / REC_DECIM);
  Serial.print(line); bleSendText(line, n);
  n = snprintf(line, sizeof(line), "REC;n;x;v;th;om;gy;fr;ft;a;iq;mode;lost;dc;foc\n");
  Serial.print(line); bleSendText(line, n);
  const int start = (recHead - recCount + REC_N) % REC_N;
  for (int i = 0; i < recCount; i++) {
    const RecS& r = recBuf[(start + i) % REC_N];
    n = snprintf(line, sizeof(line), "REC;%u;%d;%d;%d;%d;%d;%d;%d;%d;%d;%u;%u;%d;%u\n", r.n, r.x, r.v, r.th, r.om, r.gy,
                 r.fr, r.ft, r.a, r.iq, r.mode, r.lost, r.dc, r.foc);
    Serial.print(line); bleSendText(line, n);
    if ((i & 15) == 15) vTaskDelay(1);   // anderen Tasks Luft lassen (Ausgabe dauert einige Sekunden)
  }
  n = snprintf(line, sizeof(line), "REC_END\n");
  Serial.print(line); bleSendText(line, n);
  logf(">> Rekorder: %d Schritte ausgegeben", recCount);
}
