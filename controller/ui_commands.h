// ui_commands.h – Befehle aus App/Serial (UI-Task)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

static void handleUiCommand(char c, bool& monitor) {
  switch (c) {
    case 'D': recDumpReq = true; break;
    case 'S': encDiagReq = true; break;
    case 'K':
      if (ctrl.motorEnabled() || homing.active() || otaActive) { logf(">> Encoder-Kalibrierung: erst stoppen (x)"); break; }
      encCalReq = true;
      break;
    case 'L':
      telQuiet = !telQuiet;
      logf(">> Telemetrie waehrend Motorbetrieb %s (Funktest: BLE-Last %s)", telQuiet ? "AUS" : "AN",
           telQuiet ? "minimal" : "normal");
      break;
    case 'E': {   // Endstufen-Test: alle drei Phasen auf GLEICHE Spannung -> kein Motorstrom, Phasen mit Multimeter messen
      if (ctrl.motorEnabled() || homing.active() || otaActive) { logf(">> Endstufen-Test: erst stoppen (x)"); break; }
      const bool wasEn = motor.enabled;
      if (wasEn) motor.disable();
      logf(">> Endstufen-Test: SD#/EN (GPIO%d) = HIGH, alle Phasen 6 V fuer 8 s -> jede Phase muss ~6 V zeigen", PIN_FOC_EN);
      driver.enable();
      driver.setPwm(6.0f, 6.0f, 6.0f);
      vTaskDelay(pdMS_TO_TICKS(8000));
      logf(">> Endstufen-Test: alle Phasen 0 V fuer 8 s -> jede Phase muss ~0 V zeigen");
      driver.setPwm(0.0f, 0.0f, 0.0f);
      vTaskDelay(pdMS_TO_TICKS(8000));
      driver.setPwm(0.0f, 0.0f, 0.0f);
      driver.disable();
      logf(">> Endstufen-Test Ende (Endstufe aus, Ruhe ~2-3 V ist normal: schwebend)");
      if (wasEn) motor.enable();
      break;
    }
    case 'm': monitor = !monitor; break;
    case '?': printStatus(); break;
    case 'f':
      prefs.putBool("foc_ok", false); prefs.putBool("rl_ok", false);
      logf(">> FOC-Ausrichtung + R/L-Messung beim naechsten Start");
      break;
    case 'U':
#ifdef LED_STRIP
      ledTestStart();
#else
      logf(">> LED-Test: LED_STRIP in config.h nicht aktiv");
#endif
      break;
    case 'H':
    case 'F':                                     // F: immer normales Homing über die Anschläge
      if (otaActive) break;
      if (!focOk) { logf(">> Motor gesperrt (%s)", focErr); break; }
      if (ctrl.motorEnabled()) logf(">> Erst stoppen (x)");
      else { homeFullReq = (c == 'F'); homeReq = true; }
      break;
    case 'c':
      if (ctrl.motorEnabled() || homing.active()) { logf(">> Kalibrieren nur bei stehendem Motor"); break; }
      logf(">> Kalibrierung: Pendel %.0f s ruhig haengen lassen ...", CAL_TIME_S);
      pendingCmd = c;
      break;
    case 'O': startCartOta(); break;
    case 'I':
#if USE_CURRENT_SENSE
      if (!focOk || ctrl.motorEnabled() || homing.active() || otaActive) { logf(">> Strom-Test nur bei stehendem Motor"); break; }
      csTestReq = true;
#else
      logf(">> Strommessung ist aus (USE_CURRENT_SENSE 0)");
#endif
      break;
    case 'M':
      if (!focOk || ctrl.motorEnabled() || homing.active() || otaActive) { logf(">> Rastmoment lernen nur bei stehendem Motor (STOPP)"); break; }
      if (!homed) { logf(">> Rastmoment lernen: erst Homing (H)"); break; }
      if (fabsf(tx_x) > ctrl.cfg.x_soft - 0.09f) { logf(">> Rastmoment lernen: Wagen zu nah am Ende - erst zur Mitte (Halten)"); break; }
      cogCalReq = true;
      break;
    case 'N':
      ctrl.handMode = !ctrl.handMode;
      logf(">> Hand-Modus %s (%s)", ctrl.handMode ? "AN" : "AUS",
           ctrl.handMode ? "nur Winkelregelung, Wagen folgt der Pendelspitze" : "volle Regelung mit Position");
      break;
    case 'T':
      if (ctrl.motorEnabled()) { logf(">> Erst stoppen (x)"); break; }
      ctrl.trim = 0; prefs.putFloat("trim", 0);
      logf(">> Balancierpunkt-Abgleich zurueckgesetzt");
      break;
    case 'R': logf(">> Neustart"); vTaskDelay(pdMS_TO_TICKS(100)); ESP.restart(); break;
    case 'x':                                   // STOPP: auch Homing (laufend oder angefordert) abbrechen
      homeReq = false;
      encCalAbort = true; cogAbort = true;
      wakeState = 0;
      if (homing.active()) homeAbort = true;
      pendingCmd = c;
      break;
    default:
      if (otaActive) { logf(">> OTA aktiv - nur R (Neustart) moeglich"); break; }
      if (strchr("psbZG", c) && msSinceRx() > 1500) {   // Pendel schläft/fehlt -> erst wecken
        if (!homed || !focOk || homing.active()) { logf(">> Pendel sendet nicht (schlaeft?) - kurz anstossen"); break; }
        wakeCmd = c; wakeState = 1; wakeMs = millis();
        logf(">> Pendel schlaeft - wecke es mit einem kurzen Ruck ...");
        if (!ctrl.motorEnabled()) pendingCmd = 'h';
        break;
      }
      if (strchr("psbhZG0", c)) pendingCmd = c;
      break;
  }
}
