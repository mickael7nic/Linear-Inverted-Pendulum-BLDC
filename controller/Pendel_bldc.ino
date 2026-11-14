// Pendel_bldc.ino – Inverses Pendel auf Schiene (Wagen)
// ESP32 (SimpleFOC High-Power 20A) + IR2104 + INA240 + BE5208-300KV + MT6835 (SPI) + SimpleFOC 2.4
// Pendel-IMU (BMI270) kommt per ESP-NOW mit 800 Hz, Regelung im festen 800-Hz-Takt.
// Bedienung per Web-App (index.html, Web Bluetooth) oder Serial (115200).
//
// Bibliotheken: "Simple FOC" (>= 2.4.0), "SimpleFOCDrivers" (>= 1.0.9), "NimBLE-Arduino" (>= 2.1),
// Board: ESP32 Dev Module, Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA)"
//
// Architektur
//   Core 1, loop():        loopFOC() + move() so schnell wie möglich (dq-Stromregelung über INA240), Homing
//   Core 1, Task "ctrl":   800-Hz-Timer: EKF -> LQR / Swing-up / Beruhigen -> Sollbeschleunigung -> Servo -> Iq
//   Core 0, Task "ui":     Befehle (Serial/BLE), Log, Telemetrie, OTA-Vorbereitung, LED-Anzeige
//   Core 0, Task "belt":   Riemensensor TCS34725 auslesen (nur mit BELT_SENSOR)
//
// Datenfluss
//   Pendel-Sender --ESP-NOW--> espnow.h (Empfangs-Callback) --rxQueue--> control_task.h (800 Hz)
//   control_task.h: Wagenposition aus dem Encoder (sensors.h) + IMU-Paket -> ctrl.step() (controller.h)
//                   -> Sollstrom targetCurrent -> loop() gibt ihn an SimpleFOC (motor.target)
//   ui_task.h: Befehle aus Serial/BLE (cmdQueue) -> ui_commands.h setzt Anforderungs-Flags (globals.h),
//              die der Regel-Task bzw. loop() abholen. So greift nur ein Kern auf Motor und Regler zu.
//
// Gespeichertes (NVS, Namensraum "pendel"): Pendel-Kalibrierung, ell, LQR-Gewichte, Trim, Homing-Ergebnisse,
// Riemenmarke, Rastmoment-Tabelle, Winkel-Offset. Bleibt beim Flashen erhalten.
//
// Dateien
//   config.h          Pins und alle Parameter              secrets.h        WLAN-Zugangsdaten
//   protocol.h        ESP-NOW-Pakete, Telemetrie (identisch im Sender)
//   pendel_core.h     Regelungskern: core_math.h, lqr.h, ekf.h, period_meter.h, controller.h
//   homing.h          Homing-Zustandsautomat (Anschläge über den Strom, Riemenmarke)
//   globals.h         Hardware-Objekte, gemeinsamer Zustand
//   log.h             Log (Serial + BLE), Log-Verlauf über Neustarts
//   ble.h             Bluetooth           espnow.h      Funk zum Pendel
//   sensors.h         Wagenposition, IMU-Umrechnung, Pendel-Kalibrierung
//   recorder.h        800-Hz-Rekorder     control_task.h  Regel-Task
//   ota_web.h, ota.h  Firmware-Update     status.h      Statusausgabe (?)
//   belt_sensor.h     Riemenmarke (TCS34725) für die Mittensuche (BELT_SENSOR)
//   led_strip.h       WS2812-Leiste: Live-Anzeige, LED-Test (LED_STRIP)
//   ui_commands.h     Befehle             wake.h        Pendel wecken
//   trim.h            Abgleich speichern  telemetry.h   Telemetrie
//   ui_task.h         UI-Task             foc_setup.h   Motor-Initialisierung
//   homing_step.h     Homing in loop()    current_test.h  Strom-Test (I)
//   cogging.h         Rastmoment-Kompensation (M)
//   encoder_tools.h   Encoder-Test (S), MT6835-Selbstkalibrierung (K)
//
// Befehle (je ein Zeichen; Liste mit Erklärung in der Dokumentation)
//   H Homing  h Halten  s Swing-up  b Balancieren  G Beruhigen  x Stopp  0 Nullpunkt
//   c Kalibrieren  p Pendel vermessen  Z Zeitlupe  N Hand-Modus  T Abgleich zurücksetzen
//   D Rekorder  L Funk-Test  S Encoder-Test  E Endstufen-Test  K MT6835 kalibrieren  I Strom-Test
//   U LED-Test  F Homing ueber Anschlaege  M Rastmoment lernen  f FOC neu  R Neustart  O OTA  ? Status  m Monitor
//   mit Wert: X<mm> Soll-Mitte  Q<qx>,<qv>,<qth>,<qom>,<r> LQR  Y<mm> ell  W<0,01°> Winkel-Offset
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_coexist.h>
#include <esp_timer.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "config.h"
#include "protocol.h"
#include "pendel_core.h"
#include "homing.h"
#ifdef BELT_SENSOR
#include <Wire.h>
#endif

// ---- Module (Reihenfolge wichtig: jede Datei nutzt die vorherigen) ----
#include "globals.h"
#include "log.h"
#include "belt_sensor.h"
#include "ble.h"
#include "espnow.h"
#include "sensors.h"
#include "recorder.h"
#include "control_task.h"
#include "ota_web.h"
#include "status.h"
#include "ota.h"
#include "led_strip.h"
#include "ui_commands.h"
#include "wake.h"
#include "trim.h"
#include "telemetry.h"
#include "ui_task.h"
#include "foc_setup.h"
#include "homing_step.h"
#include "current_test.h"
#include "cogging.h"
#include "encoder_tools.h"

// ============================================================================
// setup / loop
// ============================================================================
// Reihenfolge: Log/BLE zuerst (damit alle Startmeldungen in der App ankommen), dann gespeicherte Werte,
// Regler, Motor, Homing-Anforderung, LED, Funk und zuletzt die Tasks (die greifen auf alles davor zu).
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(200);
  prefs.begin("pendel", false);
  cmdQueue = xQueueCreate(32, sizeof(char));   // Ein-Zeichen-Befehle von Serial/BLE an den UI-Task
  const bool logKept = logInit();   // true: Log aus dem RTC-RAM hat einen Neustart überlebt
  const bool recKept = recInit();   // true: Rekorder-Aufnahme von vor dem Neustart ist noch da
  setupBle();                     // zuerst: Startmeldungen/Fehler landen im Log (Verlauf beim Verbinden)
  logf("Start, BLE '%s' bereit", BLE_NAME);
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const char* why = rr == ESP_RST_POWERON ? "Einschalten" : rr == ESP_RST_SW ? "Software (Neustart/OTA)" :
                      rr == ESP_RST_PANIC ? "ABSTURZ" : rr == ESP_RST_BROWNOUT ? "BROWNOUT (Versorgung eingebrochen)" :
                      rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT ? "WATCHDOG" : "sonstiger";
    logf("Neustart-Grund: %s (%d)", why, (int)rr);
    if (recKept) logf(">> Rekorder enthaelt die Aufnahme VOR dem Neustart - Button 'Rekorder' zum Herunterladen");
    if (logKept) logf(">> Log vor dem Neustart steht oben (=== Log VOR dem Neustart ===)");
  }
  {   // MAC schon hier (aus eFuse, ohne WLAN-Init) -> auch bei Startfehlern im Log
    esp_read_mac(ownMac, ESP_MAC_WIFI_STA);
    logf("Wagen-MAC fuer ESP-NOW, Zeile in PendelSender.ino uebernehmen:");
    logMacLine();
  }

  // --- gespeicherte Pendel-Kalibrierung ---
  calStored = prefs.getBool("cal_ok", false);
  if (calStored) { calDelta = prefs.getFloat("cal_d", 0); calBias = prefs.getFloat("cal_b", 0); }
  angOfs = pc::clampf(prefs.getFloat("ang_o", 0), -ANG_OFS_MAX, ANG_OFS_MAX);

  // --- Regler ---
  // Alle Parameter stehen in config.h; einige können zur Laufzeit geändert werden und liegen dann im NVS
  // (ell = Ersatzlänge, damp = Pendeldämpfung, lqr = Gewichte, trim = Balancierpunkt).
  pc::CtrlConfig c{};
  c.Ts = 1.0f / IMU_RATE_HZ;
  c.latency = LATENCY_SAMPLES;
  float ellStored = prefs.getFloat("ell", PENDULUM_ELL_DEFAULT_M);
  if (ellStored < 0.08f || ellStored > PENDULUM_ELL_MAX_M) {
    Serial.printf("gespeichertes ell=%.3f unplausibel -> %.3f\n", ellStored, PENDULUM_ELL_DEFAULT_M);
    ellStored = PENDULUM_ELL_DEFAULT_M;
    prefs.putFloat("ell", ellStored);
  }
  c.ekf = {ellStored, IMU_RADIUS_M, prefs.getFloat("damp", PENDULUM_DAMP_DEFAULT),
           EKF_SIG_GYRO, EKF_SIG_ACC, EKF_Q_OM, EKF_Q_B, EKF_GATE};
  c.lqr = {LQR_QX, LQR_QV, LQR_QTH, LQR_QOM, LQR_R};
  if (prefs.getBytesLength("lqr") == sizeof(c.lqr)) prefs.getBytes("lqr", &c.lqr, sizeof(c.lqr));   // per 'Q' gesetzt
  // Wagen-Servo (Beschleunigung -> Kraft -> Strom), Grenzen, Soll-Mitte-Profil, Beruhigen
  c.m_eff = CART_MASS_EFF_KG; c.servo_wn = SERVO_WN; c.servo_zeta = SERVO_ZETA; c.servo_emax = SERVO_EMAX_M;
  c.f_coulomb = F_COULOMB_N; c.f_max = CURRENT_LIMIT_A * KT_NM_PER_A / PULLEY_RADIUS_M;
  c.x_soft = X_SOFT_M; c.x_hard = X_HARD_M; ctrl.xs_max = XS_MAX_M; ctrl.xs_rate = XS_VMAX; ctrl.xs_acc = XS_AMAX;
  ctrl.damp_c = DAMP_C; ctrl.damp_kx = DAMP_KX; ctrl.damp_kv = DAMP_KV; ctrl.damp_amax = DAMP_AMAX; ctrl.damp_calm_t = DAMP_CALM_S; c.a_max = A_MAX; c.v_max = V_MAX; c.om_lp_hz = OM_LP_HZ;
  // Ruhezone, Swing-up, Ausfallgrenzen, Trim
  c.rest_th = REST_DEG * DEG_TO_RAD; c.rest_x = REST_X_MM * 1e-3f; c.rest_hold = REST_HOLD;
  c.su_kE = SU_KE; c.su_kx = SU_KX; c.su_kv = SU_KV; c.su_amax = SU_AMAX; c.su_vmax = SU_VMAX;
  c.su_catch_th = SU_CATCH_TH; c.su_catch_om = SU_CATCH_OM; c.su_Emargin = SU_EMARGIN;
  c.bal_lost_th = BAL_LOST_TH; c.max_lost = MAX_LOST;
  c.trim_tau = TRIM_TAU_S; c.trim_max = TRIM_MAX_RAD;
  ctrl.trim = prefs.getFloat("trim", 0);
  if (!ctrl.begin(c)) logf("LQR-Entwurf fehlgeschlagen!");

  // --- FOC (Fehler sperren nur den Motor, Funk/Schätzer/App laufen weiter) ---
  focOk = setupFoc();
  if (focOk) cogLoad();
  if (!focOk) {
    pinMode(PIN_FOC_EN, OUTPUT);
    digitalWrite(PIN_FOC_EN, LOW);
    logf("FEHLER: %s - Motor gesperrt, Pendel/ESP-NOW laufen weiter. f = FOC neu, R = Neustart", focErr);
    ctrl.fault(focErr);
  }

  // --- Homing ---
  // Parameter für den Zustandsautomaten in homing.h. Ausgeführt wird er in loop() (homing_step.h),
  // gestartet über homeReq (hier: automatisch nach dem Einschalten, wenn HOME_ON_START).
  homing.cfg = {HOME_SPEED, HOME_ACCEL, HOME_I_MAX, HOME_I_STALL, HOME_V_STALL, HOME_T_STALL,
                HOME_MIN_TRAVEL, HOME_MAX_TRAVEL, HOME_TIMEOUT, CART_MASS_EFF_KG, 25.0f, 0.9f,
                KT_NM_PER_A / PULLEY_RADIUS_M};
#ifdef BELT_SENSOR
  homing.cfg.e_fast = CSW_FAST; homing.cfg.e_slow = CSW_SLOW; homing.cfg.e_back = CSW_BACK;
  beltSetup();
#endif
  homeReq = HOME_ON_START && focOk;
#ifdef LED_STRIP
  ledStripSetup();
#endif

  // --- Funk + Tasks ---
  rxQueue = xQueueCreate(32, sizeof(ImuPacket));
  if (!setupEspNow()) { halt("ESP-NOW Init fehlgeschlagen"); }
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);   // ESP-NOW vor BLE
  if (!calStored) logf("Pendel noch nicht kalibriert: Pendel ruhig haengen lassen, dann 'c'");
  // Regel-Task auf Kern 1 mit hoher Priorität; ein esp_timer weckt ihn alle 1250 us (ctrlTick -> Notify).
  // skip_unhandled_events: verpasste Takte werden NICHT nachgeholt (sonst Rucke nach Verzögerungen).
  xTaskCreatePinnedToCore(controlTask, "ctrl", 8192, nullptr, 5, &ctrlTaskH, 1);
  {
    const esp_timer_create_args_t ta = {.callback = ctrlTick, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK,
                                        .name = "ctrl", .skip_unhandled_events = true};
    esp_timer_handle_t th;
    esp_timer_create(&ta, &th);
    esp_timer_start_periodic(th, (uint64_t)(1e6f / IMU_RATE_HZ));   // 1250 us
  }
  xTaskCreatePinnedToCore(uiTask, "ui", 12288, nullptr, 1, nullptr, 0);   // WLAN-Start für OTA braucht Stack
  logf("Bereit. ell=%.4f m, Kt=%.4f Nm/A, Fmax=%.1f N", ctrl.cfg.ekf.ell, KT_NM_PER_A, c.f_max);
}

// loop() läuft auf Kern 1 so schnell wie möglich (~14–20 kHz): je Durchlauf ein FOC-Schritt.
// Langlaufende Sonderfunktionen (Encoder-Test/-Kalibrierung, Strom-Test, Rastmoment lernen, OTA) laufen
// blockierend hier, weil sie den Motor direkt brauchen; der Regel-Task ruht in dieser Zeit (Motor aus).
void loop() {
  if (encDiagReq) { encDiagReq = false; encoderDiag(); }
  if (encCalReq) { encCalReq = false; encCalAbort = false; encoderSelfCal(); }
  if (otaActive) {                // OTA läuft hier (Kern 1): der HTTP-Upload liest in einer Schleife ohne Pause,
    if (focOk && motor.enabled) motor.disable();   // im UI-Task (Kern 0) verhungerte dabei der Idle-Task -> Watchdog-Reset
    otaService();
    delay(1);
    return;
  }
  if (!focOk) { delay(5); return; }
  focLoops = focLoops + 1;   // FOC nicht initialisiert: Motor bleibt aus
#if USE_CURRENT_SENSE
  if (csTestReq) { currentSenseTest(); return; }
#endif
  if (cogCalReq) { cogCalibrate(); return; }
  homingStep();                   // Homing-Zustandsautomat (tut nichts, wenn kein Homing läuft)
  // Wer gibt den Strom vor? Während des Homings der Homing-Automat, sonst der Regel-Task (targetCurrent).
  bool en;
  float iq;
  if (otaActive) { en = false; iq = 0; }
  else if (homing.active()) { en = true; iq = X_SIGN * homing.iq; }
  else { en = wantEnable; iq = targetCurrent; }
  if (en && !motor.enabled) motor.enable();
  else if (!en && motor.enabled) motor.disable();
  motor.target = iq + ((en && !homing.active()) ? cogFF() : 0.0f);   // Rastmoment vorsteuern
#if !USE_CURRENT_SENSE
  // Totzeit-Kompensation: ohne Strommessung fehlt sonst ~0,1 V -> kleine Ströme kommen kaum an
  motor.feed_forward_voltage.q = deadtimeU * pc::clampf(iq / 0.3f, -1.0f, 1.0f);
#endif
  motor.loopFOC();
  motor.move();
}
