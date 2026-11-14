// wake.h – Schlafendes Pendel wecken (kurzer Ruck des Wagens)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

// Weckablauf (UI-Task): HOLD abwarten -> Ruck -> auf Pakete warten -> eigentlichen Befehl ausführen
static void wakeService() {
  if (!wakeState) return;
  const uint32_t el = millis() - wakeMs;
  if (wakeState == 1) {
    if (ctrl.mode == pc::Mode::HOLD && el > 300) { ctrl.nudge(); wakeState = 2; wakeMs = millis(); }
    else if (el > 2000) { logf(">> Wecken: Wagen nicht in HALTEN - abgebrochen"); wakeState = 0; }
  } else if (wakeState == 2) {
    if (msSinceRx() < 50 && senderLocked) {
      logf(">> Pendel ist wach (%.1f s)", el / 1000.0f);
      wakeState = 3; wakeMs = millis();                      // Schätzer kurz einschwingen lassen
    } else if (el > 2500 && el < 2600 && ctrl.nudgeT < 0) {  // zweiter Versuch
      ctrl.nudge();
    } else if (el > 6000) {
      logf(">> Pendel wacht nicht auf (Akku leer? ausser Reichweite?)"); wakeState = 0;
    }
  } else if (wakeState == 3 && el > 1000) {
    pendingCmd = wakeCmd; wakeState = 0;
  }
}
