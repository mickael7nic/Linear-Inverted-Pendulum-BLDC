// control_task.h – Regel-Task: fester 800-Hz-Takt, Befehle an den Regler, Schritt
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Ablauf je Takt (alle 1250 us, ausgelöst von ctrlTick über einen esp_timer):
//   1. neuestes IMU-Paket aus rxQueue holen (ältere verwerfen, verlorene Pakete zählen)
//   2. runStep(): Wagenposition/-geschwindigkeit lesen, ausstehenden Befehl ausführen, ctrl.step()
//      (EKF + Regler, controller.h) -> Wagenkraft F -> Sollstrom targetCurrent für loop()
//   3. Rekorder und Telemetrie-Variablen füttern
// Kommt kein Paket, läuft der Schritt trotzdem (m = nullptr): der EKF rechnet dann nur vorwärts,
// der Regler erkennt längere Funklücken selbst (lostCnt).
#pragma once

// Befehle, die den Regler betreffen. Sie kommen als pendingCmd vom UI-Task und werden HIER (im Regel-Task)
// ausgeführt, damit der Regler-Zustand nie gleichzeitig von zwei Kernen geändert wird.
// x, v = aktuelle Wagenposition/-geschwindigkeit (Startwerte für den neuen Modus, stoßfreier Übergang).
static void handleCommand(char c, float x, float v) {
  using pc::Mode;
  const bool idle = (ctrl.mode == Mode::IDLE || ctrl.mode == Mode::FAULT || ctrl.mode == Mode::HOLD);
  // Motor-Befehle nur mit funktionierendem FOC; Fahrbefehle nur, wenn die Position bekannt ist (gehomt).
  // Die evt*-Flags meldet der UI-Task danach im Log (hier kein logf: das wäre im 800-Hz-Takt zu langsam).
  if (strchr("psbhZG0", c) && !focOk) { evtFocBlocked = true; return; }
  if (strchr("psbhZG", c) && (!homed || homing.active())) { evtNotHomed = true; return; }
  switch (c) {
    case 'c': if (idle && !homing.active()) calib.start(); break;
    case 'p': ctrl.cmdPeriod(x); break;
    // autoReswing: fällt das Pendel beim Balancieren, schwingt der Wagen es (nur nach 's') selbst wieder hoch
    case 's': ctrl.slowMode = false; ctrl.autoReswing = true; ctrl.cmdSwingup(x, v); break;
    case 'b': ctrl.slowMode = false; ctrl.autoReswing = false; ctrl.cmdBalance(x, v); break;
    case 'Z':
      ctrl.slowMode = true; ctrl.autoReswing = false; ctrl.cmdBalance(x, v);
      logf(">> Zeitlupe: Pendel senkrecht halten -> Wagen faehrt langsam Richtung Spitze (max %.0f mm/s), x = Stopp",
           ctrl.slow_vmax * 1000.0f);
      break;
    case 'h': ctrl.cmdHold(x); break;
    case 'G': ctrl.slowMode = false; ctrl.cmdDamp(x, v); break;
    case 'x':
      ctrl.cmdStop();
      if (calib.active) { calib.active = false; evtCalib = -2; }
      break;
    case '0':                                                    // Nullpunkt von Hand: aktuelle Lage = Mitte
      if (!ctrl.motorEnabled() && !homing.active()) {
        xZeroAngle = motor.shaft_angle;
        ctrl.cfg.x_hard = X_HARD_M; ctrl.cfg.x_soft = X_SOFT_M;
        homed = true;
      }
      break;
  }
}

// Ein Regelschritt. m = neuestes IMU-Paket (bereits in physikalische Einheiten umgerechnet) oder nullptr.
static void runStep(const pc::ImuMeas* m) {
  const float x = cartX(), v = cartV();
  char c = pendingCmd;
  if (c) { pendingCmd = 0; handleCommand(c, x, v); }
  // z. B. vor OTA/Tests: Regler sicher aus
  if (forceIdle || otaActive) { forceIdle = false; ctrl.cmdIdle(); calib.active = false; }

  if (m) {
    if (calib.active) calibSample(*m);                         // 'c': Messwerte für die Pendel-Kalibrierung sammeln
    else if (!ctrl.isCalibrated() && !homing.active()) {      // Schnell-Init nach dem Start
      estInit.fr += m->fr; estInit.ft += m->ft;
      if (++estInit.n >= (int)(IMU_RATE_HZ / 4)) {
        // 1/4 s Mittelwert der Beschleunigung -> Startwinkel des EKF (Montagewinkel aus der gespeicherten Kalibrierung)
        if (ctrl.calibrate(calBias, (float)(estInit.fr / estInit.n), (float)(estInit.ft / estInit.n))) evtInit = 1;
        estInit = EstInit();
      }
    }
  }

  // 'p'/'Y': neue Pendelparameter übernehmen (LQR neu)
  if (pendApply) { ctrl.applyPendulum(pendEll, pendDamp, pendK); pendApply = false; }

  const pc::Mode before = ctrl.mode;
  const float F = ctrl.step(m, x, v);                          // Kern der Regelung: Kraft auf den Wagen [N]
  recPush(m, x, v, before);
  if (fabsf(ctrl.a_cmd) > ctrl.aMax) ctrl.aMax = fabsf(ctrl.a_cmd);   // Spitzenwerte fürs Log beim Moduswechsel
  if (fabsf(v) > ctrl.vMax) ctrl.vMax = fabsf(v);
  if (fabsf(motor.current.q) > ctrl.iqMax) ctrl.iqMax = fabsf(motor.current.q);
  if (before == pc::Mode::PERIOD && ctrl.mode != pc::Mode::PERIOD) evtPeriod = true;

  // Kraft -> Motorstrom: F * r = Moment, Moment / Kt = Strom. X_SIGN passt die Drehrichtung an die x-Achse an.
  targetCurrent = X_SIGN * F * PULLEY_RADIUS_M / KT_NM_PER_A;
  wantEnable = ctrl.motorEnabled() && !otaActive;
  tx_x = x; tx_v = v;
  if (m) { tx_fr = m->fr; tx_ft = m->ft; tx_gyro = m->gyro; }
}

static void controlTask(void*) {
  ImuPacket p;
  uint16_t expected = 0;
  bool first = true;
  // Fester 800-Hz-Takt (Timer), unabhängig vom Paketeingang. Früher lief je Paket ein Schritt und nach einer
  // Funklücke wurden alle fehlenden Schritte auf einmal nachgeholt: in null Echtzeit, mit veraltetem x/v ->
  // Wagen-Servo bekam einen Sprung (Strom an der Grenze), der Beschleunigungssensor sah den Ruck -> Winkel-"Spike".
  // Jetzt: pro Takt genau ein Schritt mit dem neuesten Paket; ältere/verspätete Pakete werden verworfen.
  (void)expected;
  uint16_t lastSeq = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));               // auf den Timer-Takt warten (5 ms Notfall-Timeout)
    ImuPacket best;
    bool have = false;
    while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
      const int d = (int16_t)(p.seq - lastSeq);                // Abstand der Paketnummern (mit Überlauf bei 65535)
      if (first || d > 0 || d < -16) {          // neuer (oder Sender neu gestartet)
        if (!first && d > 1 && d < 4000) lostTotal = lostTotal + (d - 1);
        best = p; have = true; lastSeq = p.seq; first = false;
      }
    }
    // FOC-Durchläufe seit dem letzten Takt (Diagnose: zeigt, ob loop() genug Rechenzeit bekommt)
    { const uint32_t n = focLoops; focLoops = 0; focPerTick = n > 255 ? 255 : (uint8_t)n; }
    if (have) {
      tx_vbat = best.vbat_mV; tx_flags = best.flags;
      const pc::ImuMeas m = convert(best);
      runStep(&m);
    } else {
      runStep(nullptr);
    }
  }
}
static TaskHandle_t ctrlTaskH = nullptr;
// Timer-Callback alle 1250 us: weckt nur den Regel-Task (die Arbeit selbst gehört nicht in den Timer)
static void IRAM_ATTR ctrlTick(void*) { if (ctrlTaskH) xTaskNotifyGive(ctrlTaskH); }
