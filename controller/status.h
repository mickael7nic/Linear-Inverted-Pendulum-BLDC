// status.h – Statusausgabe (Befehl ?)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
// Gibt alle wichtigen Zustände als Textzeilen ins Log (App und Serial). Erste Anlaufstelle bei Problemen.
#pragma once

static void printStatus() {
  if (otaActive) {
    const IPAddress addr = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();
    logf("OTA-IP: %s", addr.toString().c_str());
  }
  char a[18];
  macStr(ownMac, a);
  logf("Wagen-MAC %s  BLE '%s'", a, BLE_NAME);
  logMacLine();
  if (senderLocked) { macStr(senderMac, a); logf("Pendel-MAC %s  Akku %u mV", a, tx_vbat); }
  else logf("Pendel: keine Daten");
  if (!focOk) logf("Motor GESPERRT: %s", focErr);
  logf("Homing: %s  Notstopp +-%.1f mm  Swing-up +-%.1f mm", homed ? "ok" : "NEIN", ctrl.cfg.x_hard * 1e3f,
       ctrl.cfg.x_soft * 1e3f);
  logf("Kalibrierung: %s  delta=%.2f deg  bias=%.4f rad/s  Schaetzer %s", calStored ? "gespeichert" : "KEINE",
       calDelta * 57.2958f, calBias, ctrl.isCalibrated() ? "ok" : "nicht init.");
  logf("Winkel-Offset: %+.2f deg  Balancierpunkt-Abgleich: %.2f deg", angOfs * 57.2958f, ctrl.trim * 57.2958f);
#ifdef BELT_SENSOR
  logf("Riemenmarke: %s, Wert %u, Schwelle %u -> %s, %s", beltOk ? "Sensor ok" : "Sensor fehlt", beltClear, beltThr,
       beltBright ? "hell" : "dunkel", prefs.getChar("cs_dir", 0) && beltThr ? "eingelernt" : "nicht eingelernt (F)");
#endif
  logf("Ruhezone +-%.1f deg / +-%.0f mm: %s, %lu x, gesamt %.0f s", REST_DEG, REST_X_MM, ctrl.resting ? "AKTIV (Motor stromlos)" : "aus",
       (unsigned long)ctrl.restCount, ctrl.restTime);
  logf("Modus %s  ell=%.4f m  c=%.3f 1/s  K=[%.1f %.1f %.1f %.1f]", pc::modeName(ctrl.mode), ctrl.cfg.ekf.ell,
       ctrl.cfg.ekf.c, ctrl.K[0], ctrl.K[1], ctrl.K[2], ctrl.K[3]);
  logf("LQR Q=[%g %g %g %g] R=%g  (aendern: Q<qx>,<qv>,<qth>,<qom>,<r>  Standard: Q0)", ctrl.cfg.lqr.qx, ctrl.cfg.lqr.qv,
       ctrl.cfg.lqr.qth, ctrl.cfg.lqr.qom, ctrl.cfg.lqr.r);
  logf("Pakete %lu  verworfen %lu  verloren %lu  EKF-Rej %lu", (unsigned long)rxCount, (unsigned long)rxBad,
       (unsigned long)lostTotal, (unsigned long)ctrl.ekf.rejected);
  logf("FOC-Schleife ~%u Hz (%u je Regeltakt)", (unsigned)(focPerTick * IMU_RATE_HZ), (unsigned)focPerTick);
}
