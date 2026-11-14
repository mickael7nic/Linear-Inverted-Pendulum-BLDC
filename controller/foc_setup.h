// foc_setup.h – Start: FOC/Motor/Encoder/Strommessung einrichten, Startfehler
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

// Startfehler: Motor bleibt aus, BLE bleibt erreichbar (Log-Verlauf, '?' und 'R' = Neustart)
static void halt(const char* msg) {
  logf("FEHLER: %s - angehalten. R = Neustart, f = FOC neu, O = OTA", msg);
  pinMode(PIN_FOC_EN, OUTPUT);   // Endstufe sicher aus (auch wenn driver.init() fehlschlug)
  digitalWrite(PIN_FOC_EN, LOW);
  ctrl.fault(msg);               // Web-App zeigt FAULT, Befehle bleiben bedienbar
  uint32_t lastTel = 0;
  for (;;) {
    if (bleSendHistory) { bleSendHistory = false; logSendHistory(); }
    char c = 0;
    if (Serial.available()) c = Serial.read();
    else if (xQueueReceive(cmdQueue, &c, 0) != pdTRUE) c = 0;
    if (c == 'R') { logf(">> Neustart"); delay(100); ESP.restart(); }
    else if (c == 'f') { prefs.putBool("foc_ok", false); prefs.putBool("rl_ok", false); logf(">> FOC + R/L beim naechsten Start neu"); }
    else if (c == '?') { logf("FEHLER: %s", msg); logMacLine(); }
    else if (c == 'O') startCartOta();
    if (otaActive) {
      otaService();
    }
    if (millis() - lastTel >= 200) { lastTel = millis(); sendTelemetry(); }
    delay(otaActive ? 5 : 20);
  }
}

static bool setupFoc() {
#if PIN_ENC_CAL_EN >= 0 && !defined(BELT_SENSOR)
  pinMode(PIN_ENC_CAL_EN, OUTPUT);                 // CAL_EN fest LOW: keine ungewollte Selbstkalibrierung
  digitalWrite(PIN_ENC_CAL_EN, LOW);
#endif
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ENC_CS);
  encoder.init(&SPI);
  motor.linkSensor(&encoder);
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE;
  driver.pwm_frequency = PWM_FREQ_HZ;
  if (!driver.init()) { focErr = "Treiber-Init fehlgeschlagen"; return false; }
  motor.linkDriver(&driver);
#if USE_CURRENT_SENSE
  currentSense.linkDriver(&driver);
  if (!currentSense.init()) { focErr = "Strommessung-Init fehlgeschlagen"; return false; }
  motor.linkCurrentSense(&currentSense);
  motor.torque_controller = TorqueControlType::foc_current;
  {   // Schutz: Stromregelung nur im Messbereich (übersteuerte Messung -> Regler würde Strom hochtreiben)
    const float range = 0.9f * 1.6f / (SHUNT_OHM * INA240_GAIN);
    if (CURRENT_LIMIT_A > range) logf("WARNUNG: Strommessung nur bis ~%.1f A -> Strombegrenzung %.1f statt %.1f A", range, range, CURRENT_LIMIT_A);
  }
#else
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.LPF_current_q.Tf = 0.0f;      // Default 10 ms wäre Totzeit im Kraftpfad!
  motor.phase_resistance = prefs.getFloat("R", MOTOR_PHASE_RESISTANCE_OHM);   // aus Strom-Test 'I'
  deadtimeU = prefs.getFloat("dtU", 0.10f);   // 0,10 V am 01.11. gemessen
  logf("Geschaetzter Strom: R=%.4f Ohm, Totzeit-Kompensation %.3f V", motor.phase_resistance, deadtimeU);
#endif
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = SUPPLY_VOLTAGE / _SQRT3;
#if USE_CURRENT_SENSE
  motor.current_limit = fminf(CURRENT_LIMIT_A, 0.9f * 1.6f / (SHUNT_OHM * INA240_GAIN));
#else
  motor.current_limit = CURRENT_LIMIT_A;
#endif
  motor.velocity_limit = 1e4f;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.LPF_velocity.Tf = 0.001f;
  const bool focStored = prefs.getBool("foc_ok", false);
  if (focStored) {
    motor.zero_electric_angle = prefs.getFloat("zea", 0);
    motor.sensor_direction = (Direction)prefs.getChar("dir", 1);
  }
  SimpleFOCDebug::enable(&logPrint);   // Fehlerursachen von init/initFOC im Log
  motor.init();
  logf(focStored ? "FOC: gespeicherte Ausrichtung" : "FOC: Ausrichtung (Motor dreht kurz)...");
  if (!motor.initFOC()) { focErr = "initFOC fehlgeschlagen"; return false; }
  if (!focStored) {
    prefs.putFloat("zea", motor.zero_electric_angle);
    prefs.putChar("dir", (int8_t)motor.sensor_direction);
    prefs.putBool("foc_ok", true);
  }
#if USE_CURRENT_SENSE
  {   // Kanal-Abgleich der Strommessung aus Strom-Test 'I'
    const float gB = prefs.getFloat("gB", 1.0f);
    if (CS_GAIN_CORR && gB > 0.6f && gB < 1.6f) currentSense.gain_b *= gB;
    logf("Strommessung: Kanal-B-Korrektur %.3f %s", gB, CS_GAIN_CORR ? "aktiv" : "(gespeichert, aber AUS: CS_GAIN_CORR)");
  }
  // R/L einmalig messen (Motor bestromt kurz), danach aus dem NVS
  if (prefs.getBool("rl_ok", false)) {
    motor.phase_resistance = prefs.getFloat("R", MOTOR_PHASE_RESISTANCE_OHM);
    motor.axis_inductance.d = prefs.getFloat("Ld", 3e-5f);
    motor.axis_inductance.q = prefs.getFloat("Lq", 3e-5f);
  } else {
    logf("Messe R/L ...");
    if (motor.characteriseMotor(ALIGN_VOLTAGE) == 0) {
      prefs.putFloat("R", motor.phase_resistance);
      prefs.putFloat("Ld", motor.axis_inductance.d);
      prefs.putFloat("Lq", motor.axis_inductance.q);
      prefs.putBool("rl_ok", true);
    } else {
      logf("R/L-Messung fehlgeschlagen - Werte aus config.h");
      motor.phase_resistance = MOTOR_PHASE_RESISTANCE_OHM;
      motor.axis_inductance.d = motor.axis_inductance.q = 3e-5f;
    }
  }
  logf("R=%.4f Ohm  Ld=%.1f uH  Lq=%.1f uH", motor.phase_resistance, motor.axis_inductance.d * 1e6f,
       motor.axis_inductance.q * 1e6f);
  // FOC-Schleifenzeit messen (Motor aus), dann Stromregler auf Bandbreite auslegen
  motor.disable();
  for (int i = 0; i < 2000; i++) { motor.loopFOC(); motor.move(); }
  float bw = CURRENT_BW_HZ;
  while (motor.tuneCurrentController(bw) != 0 && bw > 50.0f) bw *= 0.8f;
  logf("Stromregler: BW=%.0f Hz  Pq=%.3f Iq=%.1f", bw, motor.PID_current_q.P, motor.PID_current_q.I);
#endif
  motor.target = 0;
  motor.disable();
  motor.loopFOC();
  motor.move();
  xZeroAngle = motor.shaft_angle;     // vorläufig, bis Homing fertig
  return true;
}
