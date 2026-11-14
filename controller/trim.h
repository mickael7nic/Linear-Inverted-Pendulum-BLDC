// trim.h – Balancierpunkt-Abgleich im NVS sichern
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

// Balancierpunkt-Abgleich im NVS sichern (nicht während der Motor läuft: Flash-Schreiben blockiert kurz)
static void trimSave(bool log) {
  static float savedTrim = NAN;
  if (isnan(savedTrim)) savedTrim = prefs.getFloat("trim", 0);
  if (fabsf(ctrl.trim - savedTrim) <= 0.0002f) return;
  savedTrim = ctrl.trim;
  prefs.putFloat("trim", savedTrim);
  if (log) logf(">> Balancierpunkt-Abgleich gespeichert: %.2f deg", savedTrim * 57.2958f);
}
