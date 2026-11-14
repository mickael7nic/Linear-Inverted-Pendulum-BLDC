// ui_task.h – UI-Task: Befehle, Ereignisse, Log-Meldungen, Telemetrie, OTA
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

static void uiTask(void*) {
  bool monitor = false;
  pc::Mode lastMode = ctrl.mode;
  pc::Homing::State lastHome = homing.st;
  uint32_t lastRx = 0, lastRate = 0, lastResume = 0;
  for (;;) {
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == 'X') {                       // "X<mm>" setzt die Soll-Mittelposition
        setXSet(Serial.parseInt() * 1e-3f);
        logf(">> Soll-Mitte %.0f mm", ctrl.x_set * 1e3f);
      } else if (c > ' ') handleUiCommand(c, monitor);
    }
    char bc;
    while (xQueueReceive(cmdQueue, &bc, 0) == pdTRUE) handleUiCommand(bc, monitor);
    if (recDumpReq) { recDumpReq = false; recDump(); }
    wakeService();
#ifdef LED_STRIP
    ledShowService();
#endif
    {   // per Schieberegler geänderten Winkel-Offset im Ruhezustand speichern
      static uint32_t lastOfsChk = 0;
      static float savedOfs = NAN;
      if (isnan(savedOfs)) savedOfs = prefs.getFloat("ang_o", 0);
      if (!ctrl.motorEnabled() && millis() - lastOfsChk > 2000) {
        lastOfsChk = millis();
        if (fabsf(angOfs - savedOfs) > 1e-5f) {
          savedOfs = angOfs; prefs.putFloat("ang_o", savedOfs);
          logf(">> Winkel-Offset gespeichert: %+.2f deg", savedOfs * 57.2958f);
        }
      }
    }
    if (evtRecJump) { evtRecJump = false; logf(">> Winkelmessung weicht stark ab (Sprung) - Rekorder haelt die Stelle fest"); }
    if (evtRecFrozen) { evtRecFrozen = false; logf(">> Rekorder: letzte %.0f s (%.0f Hz) gespeichert - Button 'Rekorder' zum Herunterladen", REC_N * REC_DECIM / IMU_RATE_HZ, IMU_RATE_HZ / REC_DECIM); }

    if (bleSendHistory) {
      bleSendHistory = false;
      logSendHistory();
    }

    if (otaActive) {                        // OTA selbst läuft in loop() auf Kern 1
      if (bleRelReq && !bleReleased) {      // Upload startet: Bluetooth abschalten -> RAM frei (alle BLE-Zugriffe nur hier)
        bleShutdown = true; bleConnected = false;
        chLog = nullptr; chTel = nullptr;
        NimBLEDevice::deinit(true);
        bleServer = nullptr;
        bleReleased = true;
      }
      static uint32_t lastTel = 0;
      if (millis() - lastTel >= 1000) { lastTel = millis(); sendTelemetry(); }   // selten: WLAN hat Vorrang
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (ctrl.mode != lastMode) {
      if (lastMode == pc::Mode::BALANCE) trimSave(true);   // Abgleich nach dem Balancieren speichern
      const pc::Mode prevMode = lastMode;
      lastMode = ctrl.mode;
      logf(">> %s %s", pc::modeName(lastMode), lastMode == pc::Mode::FAULT ? ctrl.faultMsg :
                                                lastMode == pc::Mode::STOPPING ? ctrl.stopReason : "");
      if (lastMode == pc::Mode::FAULT || prevMode == pc::Mode::BALANCE)
        logf("   Funk seit letztem Moduswechsel: laengste Luecke %.0f ms, Luecken >= 50 ms: %d", ctrl.lostMax * 1.25f,
             ctrl.gapsLong);
      if (prevMode != pc::Mode::IDLE && prevMode != pc::Mode::FAULT)
        logf("   Spitzen in %s: Beschleunigung %.1f m/s^2 (Soll), Geschwindigkeit %.2f m/s, Strom %.1f A",
             pc::modeName(prevMode), ctrl.aMax, ctrl.vMax, ctrl.iqMax);
      ctrl.lostMax = 0; ctrl.gapsLong = 0;
      ctrl.aMax = ctrl.vMax = ctrl.iqMax = 0;
    }
    if (homing.st != lastHome) {
      lastHome = homing.st;
      switch (lastHome) {
        case pc::Homing::SEEK_NEG: logf(">> Homing: suche Anschlag -x"); break;
        case pc::Homing::SEEK_POS: logf(">> Homing: -x bei %.1f mm, suche Anschlag +x", homing.xNeg * 1e3f); break;
        case pc::Homing::CENTER:
          if (homing.centerRun) logf(">> Mittensuche: Kante gefunden, fahre zur Mitte");
          else if (homing.edgeLearned) logf(">> Mittenkontakt: Kante %+.1f mm neben der Mitte, fahre zurueck", homing.edgeOfs * 1e3f);
          else logf(">> Homing: +x bei %.1f mm, Weg %.1f mm, fahre zur Mitte", homing.xPos * 1e3f, homing.travel() * 1e3f);
          break;
        case pc::Homing::EDGE_FAST:
          if (!homing.centerRun) logf(">> Mittenkontakt erkannt (Band auf der %s Seite) - vermesse die Kante", homing.bandDir > 0 ? "+x" : "-x");
          break;
        case pc::Homing::DONE:
          logf(">> Homing ok. Notstopp +-%.1f mm, Swing-up +-%.1f mm, Iq frei max %.2f A (Schwelle %.2f A)",
               ctrl.cfg.x_hard * 1e3f, ctrl.cfg.x_soft * 1e3f, homing.iqFreeMax, HOME_I_STALL);
          break;
        case pc::Homing::FAILED: logf(">> Homing FEHLER: %s", homing.msg); break;
        default: break;
      }
    }
#ifdef BELT_SENSOR
    if (beltNoContrastEvt) {
      beltNoContrastEvt = false;
      logf(">> Riemenmarke: kein Hell/Dunkel-Unterschied an den Anschlaegen (%u / %u) - nicht eingelernt", beltNegVal, beltPosVal);
    }
    if (csLearnedEvt) { csLearnedEvt = false; logf(">> Riemenmarke gespeichert: 'H' sucht ab jetzt nur die Kante, 'F' = Homing ueber die Anschlaege"); }
    {   // Plausibilität im Betrieb: Riemenmarke muss zur Encoder-Position passen (5 mm Hysterese um die Kante)
      static uint32_t badMs = 0; static bool warned = false;
      const int8_t bd = homing.bandDir;
      if (homed && bd != 0 && !homing.active()) {
        const float rel = bd * (tx_x - homing.edgeOfs);               // > 0: auf der Bandseite
        const bool sw = centerSw();
        const bool bad = (rel > 0.005f && !sw) || (rel < -0.005f && sw);
        if (bad) { if (!badMs) badMs = millis(); } else { badMs = 0; warned = false; }
        if (badMs && millis() - badMs > 300 && !warned) {
          warned = true;
          logf(">> WARNUNG: Riemenmarke passt nicht zur Position (x=%.1f mm, Sensor %s) - Riemen uebersprungen? Neu homen",
               tx_x * 1e3f, sw ? "hell" : "dunkel");
        }
      }
    }
#endif
    // Pendel-Verbindung
    const uint32_t silent = msSinceRx();
    if (senderLocked && silent > 2000) { senderLocked = false; logf(">> Pendel getrennt"); }
    if (millis() - lastResume >= 500) {        // Heartbeat für die Pendel-LED, RESUME weckt pausierten Sender
      lastResume = millis();
      const bool busy = ctrl.motorEnabled() || homing.active() || calib.active || otaActive;
      sendPendelCmd(silent > 1000 ? CMD_RESUME : CMD_ALIVE, 1, busy ? CMD_ARG_BUSY : 0);
    }
    if (evtSender) {
      evtSender = false;
      char a[18];
      macStr(senderMac, a);
      logf(">> Pendel verbunden: %s", a);
    }
    {   // Pendel meldet kritischen Akku -> sauber stoppen, bevor die Funkverbindung abreißt
      static bool shutdownSeen = false;
      const bool sd = (tx_flags & PKT_FLAG_SHUTDOWN) && msSinceRx() < 500;
      if (sd && !shutdownSeen) {
        logf(">> Pendel-Akku kritisch (%u mV) - Pendel geht in Tiefschlaf, Stopp", tx_vbat);
        if (ctrl.motorEnabled()) pendingCmd = 'x';
      }
      shutdownSeen = sd;
      static bool sleepSeen = false;
      const bool sl = (tx_flags & PKT_FLAG_SLEEP) && msSinceRx() < 500;
      if (sl && !sleepSeen) logf(">> Pendel schlaeft (keine Bewegung) - zum Aufwecken kurz anstossen");
      sleepSeen = sl;
    }
    if (evtFocBlocked) { evtFocBlocked = false; logf(">> Motor gesperrt (%s) - 'f' + Neustart fuer neue FOC-Ausrichtung", focErr); }
    if (evtNotHomed) { evtNotHomed = false; logf(">> Erst homen (H) bzw. Homing abwarten"); }
    if (evtCalibMoved) { evtCalibMoved = false; logf(">> Kalibrierung: Bewegung erkannt, Zeit startet neu"); }
    if (evtInit) {
      evtInit = 0;
      logf(">> Schaetzer initialisiert (%s)", calStored ? "gespeicherte Kalibrierung" : "OHNE Kalibrierung - 'c'");
    }
    if (evtCalib) {
      const int e = evtCalib;
      evtCalib = 0;
      if (e > 0) {
        prefs.putFloat("cal_d", calNewDelta);
        prefs.putFloat("cal_b", calNewBias);
        prefs.putBool("cal_ok", true);
        calStored = true;
        logf(">> Kalibrierung ok, gespeichert: delta=%.2f deg  bias=%.4f rad/s", calNewDelta * 57.2958f, calNewBias);
      } else if (e == -2) {
        logf(">> Kalibrierung abgebrochen");
      } else {
        logf(">> Kalibrierung fehlgeschlagen (nicht %.0f s ruhig oder Achsen/Montagewinkel > %.0f deg)", CAL_TIME_S,
             CAL_MAX_OFFSET * 57.2958f);
      }
    }
    if (ellReq != 0) {
      const float e = ellReq;
      ellReq = 0;
      float Kn[4];
      if (e < 0.08f || e > PENDULUM_ELL_MAX_M || !pc::designLQR(e, ctrl.cfg.Ts, ctrl.cfg.lqr, Kn)) {
        logf(">> ell=%.3f m ungueltig (0,08..%.2f m)", e, PENDULUM_ELL_MAX_M);
      } else {
        memcpy(pendK, Kn, sizeof(Kn)); pendEll = e; pendDamp = ctrl.cfg.ekf.c; pendApply = true;
        prefs.putFloat("ell", e);
        logf(">> ell=%.3f m gesetzt und gespeichert -> K=[%.1f %.1f %.1f %.1f]", e, Kn[0], Kn[1], Kn[2], Kn[3]);
      }
    }
    if (lqrReq) {
      lqrReq = false;
      const pc::LQRWeights w = lqrNew;
      float Kn[4];
      if (w.qx < 0 || w.qv < 0 || w.qth <= 0 || w.qom < 0 || w.r <= 0 || !pc::designLQR(ctrl.cfg.ekf.ell, ctrl.cfg.Ts, w, Kn)) {
        logf(">> LQR: ungueltige Gewichte bzw. Entwurf fehlgeschlagen");
      } else {
        ctrl.cfg.lqr = w;
        memcpy(pendK, Kn, sizeof(Kn)); pendEll = ctrl.cfg.ekf.ell; pendDamp = ctrl.cfg.ekf.c; pendApply = true;
        const bool def = w.qx == LQR_QX && w.qv == LQR_QV && w.qth == LQR_QTH && w.qom == LQR_QOM && w.r == LQR_R;
        if (def) prefs.remove("lqr"); else prefs.putBytes("lqr", &w, sizeof(w));
        logf(">> LQR Q=[%g %g %g %g] R=%g%s -> K=[%.1f %.1f %.1f %.1f]", w.qx, w.qv, w.qth, w.qom, w.r,
             def ? " (Standard)" : " (gespeichert)", Kn[0], Kn[1], Kn[2], Kn[3]);
      }
    }
    if (evtPeriod) {
      evtPeriod = false;
      const auto& pm = ctrl.period;
      if (pm.done && pm.valid && (pm.ell > PENDULUM_ELL_MAX_M || pm.ell < 0.08f)) {
        logf(">> Periodenmessung unplausibel: ell=%.3f m (Pendel %.2f m) - nicht uebernommen. Kleiner auslenken (10-20 Grad), "
             "Wagen nicht beruehren", pm.ell, PENDULUM_ELL_MAX_M);
      } else if (pm.done && pm.valid) {
        logf(">> T=%.4f s  Amplitude %.1f deg  ell=%.4f m  c=%.3f 1/s -> gespeichert", pm.T, pm.amp * 57.2958f, pm.ell,
             pm.c);
        pendEll = pm.ell; pendDamp = pm.c;
        if (pc::designLQR(pm.ell, ctrl.cfg.Ts, ctrl.cfg.lqr, pendK)) pendApply = true;
        else logf(">> LQR-Entwurf fehlgeschlagen");
        prefs.putFloat("ell", pm.ell);
        prefs.putFloat("damp", pm.c);
      } else {
        logf(">> Periodenmessung ungueltig (Amplitude 2..30 deg, frei schwingen lassen)");
      }
    }
    if (millis() - lastRate >= 5000) {
      const uint32_t r = rxCount;
      if (!monitor) Serial.printf("ESP-NOW %.0f Pakete/s%s\n", (r - lastRx) / 5.0f,
                                  (tx_flags & PKT_FLAG_VBAT_LOW) ? "  AKKU LEER" : "");
      lastRx = r; lastRate = millis();
    }
    if (monitor) {
      Serial.printf("x:%.4f v:%.3f th:%.2f om:%.3f b:%.4f a:%.2f F:%.2f fr:%.2f ft:%.2f mode:%d\n", tx_x, tx_v,
                    ctrl.th_c * 57.2958f, ctrl.om_c, ctrl.ekf.b, ctrl.a_cmd, ctrl.F_cmd, tx_fr, tx_ft,
                    (int)ctrl.mode);
    }
    sendTelemetry();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
