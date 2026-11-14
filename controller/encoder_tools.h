// encoder_tools.h – Encoder-Test (S) und MT6835-Selbstkalibrierung (K)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

// Encoder-Diagnose (läuft in loop(), damit der SPI-Zugriff nicht mit loopFOC() kollidiert)
static void encoderDiag() {
  const bool crcWas = encoder.checkcrc;
  logf(">> Encoder-Test MT6835 (CS=GPIO%d SCK=%d MOSI=%d MISO=%d, %lu Hz):", PIN_ENC_CS, PIN_SPI_SCK, PIN_SPI_MOSI,
       PIN_SPI_MISO, (unsigned long)ENC_SPI_HZ);
  int crcBad = 0;
  uint32_t mn = 0xFFFFFFFF, mx = 0;
  for (int i = 0; i < 8; i++) {
    const uint32_t raw = encoder.readRawAngle21();
    const uint8_t st = encoder.getStatus();
    encoder.checkcrc = true;                     // CRC über getCurrentAngle prüfen (liest erneut)
    const float ang = encoder.getCurrentAngle();
    encoder.checkcrc = crcWas;
    if (ang < 0) crcBad++;
    mn = raw < mn ? raw : mn; mx = raw > mx ? raw : mx;
    logf("   roh=%7lu (%.2f deg) Status=0x%X %s", (unsigned long)raw, raw * 360.0f / 2097152.0f, st,
         ang < 0 ? "CRC-FEHLER" : "CRC ok");
    delay(150);
  }
  logf("   Register: Nullpunkt=%u  Bandbreite=%u  Hysterese=%u  ABZ-Aufl.=%u", encoder.getZeroPosition(),
       encoder.getBandwidth(), encoder.getHysteresis(), encoder.getABZResolution());
  if (mx == 0) logf("   -> liest nur 0: MISO (GPIO%d) haengt auf LOW oder Chip antwortet nicht", PIN_SPI_MISO);
  else if (mn == 2097151) logf("   -> liest nur 1: MISO offen/HIGH, CS oder SCK pruefen");
  else if (crcBad == 8) logf("   -> nur CRC-Fehler: Signal gestoert (Leitungen, Masse, SPI-Takt)");
  else if (mn == mx) logf("   -> Wert aendert sich nicht: waehrend des Tests Motor von Hand drehen! Sonst Magnet/Chip pruefen");
  else logf("   -> Encoder antwortet plausibel");
}

// MT6835-Selbstkalibrierung (Datenblatt "User Auto-Calibration"): Drehzahlbereich in AUTOCAL_FREQ setzen,
// Motor gleichmäßig drehen (gesteuert, ohne Encoder), CAL_EN high, > 64 Umdrehungen weiterdrehen,
// Status 0x113[7:6]: 01 läuft, 10 Fehler, 11 ok. RIEMEN MUSS AB SEIN (64 U. = ~2,6 m Riemenweg)!
static void encoderSelfCal() {
  const float rpm = ENC_CAL_RPM;
  uint8_t code = 7;
  for (int c = 0; c < 8; c++) {                          // Bereich c: [6400/2^(c+1), 6400/2^c)
    const float lo = 6400.0f / (float)(1 << (c + 1)), hi = 6400.0f / (float)(1 << c);
    if (rpm >= lo && rpm < hi) { code = c; break; }
  }
  logf(">> Encoder-Kalibrierung: %.0f rpm (AUTOCAL_FREQ=%u), Dauer ca. %.0f s", rpm, code, 64.0f * 60.0f / rpm + 8.0f);
  MT6835Options4 o4 = encoder.getOptions4();
  o4.autocal_freq = code;
  encoder.setOptions4(o4);
  if (encoder.getOptions4().autocal_freq != code) { logf("   FEHLER: AUTOCAL_FREQ nicht geschrieben"); return; }
#if PIN_ENC_CAL_EN >= 0
#ifdef BELT_SENSOR
  beltStop();                                      // GPIO22 vom I2C freigeben (Riemensensor abgesteckt, CAL_EN dran)
  logf("   I2C beendet: Riemensensor muss abgesteckt und CAL_EN an GPIO%d angeschlossen sein", PIN_ENC_CAL_EN);
#endif
  pinMode(PIN_ENC_CAL_EN, OUTPUT); digitalWrite(PIN_ENC_CAL_EN, LOW);
#endif
  const bool wasEn = motor.enabled;
  motor.enable();
  const float wTarget = rpm * 2.0f * (float)M_PI / 60.0f;          // [rad/s] Welle
  float w = 0, ang = 0;
  uint32_t t0 = millis(), tPrev = micros(), tLog = 0, tCal = 0;
  bool calOn = false;
  uint8_t st = 0, lastSt = 255;
  const uint32_t TMAX = (uint32_t)((64.0f * 60.0f / rpm) * 1000.0f * 1.6f) + 4000;
  for (;;) {
    const uint32_t now = micros();
    const float dt = (now - tPrev) * 1e-6f;
    tPrev = now;
    const float el = millis() - t0;
    w = fminf(wTarget, w + wTarget / 3.0f * dt);                      // 3 s hochfahren
    ang += w * dt;
    motor.setPhaseVoltage(ENC_CAL_UQ, 0, _electricalAngle(ang, motor.pole_pairs));
    if (!calOn && el > 4000) {                                       // Drehzahl konstant -> Kalibrierung starten
      calOn = true; tCal = millis();
#if PIN_ENC_CAL_EN >= 0
      digitalWrite(PIN_ENC_CAL_EN, HIGH);
      logf("   CAL_EN (GPIO%d) = HIGH, Kalibrierung laeuft ...", PIN_ENC_CAL_EN);
#else
      logf("   JETZT CAL_EN (Pin 4 des MT6835) auf 3,3 V legen und halten ...");
#endif
    }
    if (calOn && millis() - tLog > 500) {
      tLog = millis();
      st = encoder.getCalibrationStatus();
      if (st != lastSt) {
        lastSt = st;
        logf("   Status: %s", st == 0 ? "keine Kalibrierung (CAL_EN noch low?)" : st == 1 ? "laeuft" : st == 2 ? "FEHLGESCHLAGEN" : "ERFOLGREICH");
      }
      if (st == 2 || st == 3) break;
      if (millis() - tCal > TMAX) { logf("   Zeitueberschreitung"); break; }
    }
    if (encCalAbort) { logf("   abgebrochen (STOPP)"); st = 0; break; }
    vTaskDelay(1);
  }
  for (float u = ENC_CAL_UQ; w > 0.5f; ) {                          // auslaufen lassen
    const uint32_t now = micros();
    const float dt = (now - tPrev) * 1e-6f;
    tPrev = now;
    w = fmaxf(0.0f, w - wTarget / 2.0f * dt);
    ang += w * dt;
    motor.setPhaseVoltage(u, 0, _electricalAngle(ang, motor.pole_pairs));
    vTaskDelay(1);
  }
  motor.setPhaseVoltage(0, 0, 0);
#if PIN_ENC_CAL_EN >= 0
  digitalWrite(PIN_ENC_CAL_EN, LOW);
#else
  if (calOn) logf("   CAL_EN wieder loesen (nicht mit 3,3 V verbunden lassen)");
#endif
  if (!wasEn) motor.disable();
  encCalReq = false; encCalAbort = false;
  if (st == 3) {
    // Datenblatt 9.2: nach Erfolg > 6 s warten, dann den Chip AUSSCHALTEN; bis zum Wiedereinschalten keine
    // weiteren Operationen. Kein eigener EEPROM-Befehl nötig.
    prefs.putBool("foc_ok", false);                                  // neue FOC-Ausrichtung beim naechsten Start
    logf("   Bitte >6 s warten, dann Versorgung AUS und wieder EIN (R reicht nicht, der Encoder muss stromlos werden).");
    logf("   Beim Start wird die FOC-Ausrichtung neu gemacht.");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));                         // Motor bleibt aus bis zum Aus-/Einschalten
  }
}
