// config.h – Hardware- und Reglerparameter Wagen
// Alle Einstellungen an einer Stelle. Werte, die zur Laufzeit gelernt oder per Befehl gesetzt werden
// (ell, LQR-Gewichte, Kalibrierung, Trim, Homing/Riemenmarke, R/L, Rastmoment), liegen zusätzlich im NVS und
// überschreiben dort die Vorgaben von hier. Optionale Hardware wird mit #define an/aus geschaltet
// (LED_STRIP, BELT_SENSOR, HOME_ON_START).
#pragma once
#include <math.h>

// ---------------- Pins Treiberplatine ----------------
#define PIN_FOC_IN1     16      // IR2104 Phase A, IN
#define PIN_FOC_IN2     17      // IR2104 Phase B, IN
#define PIN_FOC_IN3     5       // IR2104 Phase C, IN
#define PIN_FOC_EN      4       // IR2104 SD# (gemeinsam, aktiv HIGH)

#define PIN_ENC_CS      13      // MT6835 CSN
#define PIN_SDA         21      // I2C-Anschluss der Platine (Riemensensor TCS34725, wenn BELT_SENSOR)
#define PIN_SCL         22      // I2C-Anschluss der Platine (Riemensensor) oder MT6835 CAL_EN (ohne BELT_SENSOR)
#define PIN_ENC_CAL_EN  PIN_SCL // MT6835 CAL_EN (Pin 4) -> Selbstkalibrierung per Befehl K automatisch;
                                // -1 = nicht verbunden: CAL_EN bei Aufforderung von Hand kurz auf 3,3 V legen
constexpr float ENC_CAL_RPM   = 150.0f;   // Drehzahl der Selbstkalibrierung (Bereich 100-200 rpm -> AUTOCAL_FREQ 5)
constexpr float ENC_CAL_UQ    = 0.5f;     // [V] Spannung im gesteuerten Betrieb (ohne Riemen kaum Last)
#define PIN_SPI_MOSI    23
#define PIN_SPI_MISO    19
#define PIN_SPI_SCK     18
constexpr uint32_t ENC_SPI_HZ = 8000000;     // MT6835 bis 16 MHz

// ---------------- Strommessung INA240 (inline, ADC1 -> WLAN-verträglich) ----------------
#define USE_CURRENT_SENSE 1     // 1 = echte dq-Stromregelung über INA240 (seit 04.11. mit 2-mOhm-Shunts parallel)
                                // 0 = geschätzter Strom (R + Totzeit aus Strom-Test 'I')
#define PIN_CUR_A       39      // INA240 Phase A
#define PIN_CUR_B       36      // INA240 Phase B
constexpr float SHUNT_OHM      = 0.00182f; // R020 || R002 = 1,82 mOhm (mit Strom-Test 'I' prüfen/korrigieren)
constexpr float INA240_GAIN    = 50.0f;    // I240A2 -> ~0,09 V/A, Messbereich ca. +-18 A
constexpr float CURRENT_BW_HZ  = 300.0f;   // Stromregler-Bandbreite (wird bei zu langsamer FOC-Schleife reduziert)

// ---------------- Motor BE5208-300KV ----------------
constexpr int   POLE_PAIRS                 = 11;     // 22 Magnete (24N22P)
constexpr float MOTOR_PHASE_RESISTANCE_OHM = 0.053f; // Datenblatt ~0,106 Ohm Leiter-Leiter; wird beim 1. Start gemessen
constexpr float MOTOR_KV                   = 300.0f;
constexpr float SUPPLY_VOLTAGE             = 12.0f;  // ANPASSEN
constexpr float CURRENT_LIMIT_A            = 12.0f;  // Iq-Begrenzung
constexpr float ALIGN_VOLTAGE              = 0.5f;   // bei ~0,05 Ohm/Phase schon ~6-9 A! Nicht erhöhen
constexpr float PWM_FREQ_HZ                = 20000.0f;
// Drehmomentkonstante für Iq-Amplitude: Kt = 0.866 / KV[rad/s/V]
constexpr float KT_NM_PER_A = 0.866f / (MOTOR_KV * 2.0f * (float)M_PI / 60.0f);

// ---------------- Mechanik ----------------
constexpr float PULLEY_RADIUS_M   = 0.00637f;  // GT2, 20 Zähne: 20*2mm/(2*pi). ANPASSEN
constexpr int   X_SIGN            = 1;         // Wagen von Hand Richtung +x schieben -> x muss steigen
constexpr float CART_MASS_EFF_KG  = 2.5f;      // Wagen+Pendel ~0,4 kg + Rotor BE5208 J/r^2 ~2 kg (geschätzt)
constexpr float X_SOFT_M          = 0.30f;     // Arbeitsbereich ab Mitte (Obergrenze; nach Homing Weg/2 - HOME_MARGIN - HOME_SOFT_GAP)
constexpr float XS_MAX_M          = 0.35f;     // Soll-Mitte (Schieberegler) max. ±350 mm, zusätzlich <= Notstopp - 30 mm
constexpr float X_HARD_M          = 0.36f;     // Notstopp ab Mitte (nur ohne Homing; danach Weg/2 - HOME_MARGIN)

// ---------------- LED-Leiste: Anzeige (LED_STRIP) ----------------
// WS2812-Leiste parallel zur Schiene zeigt Wagen/Modus an, LED-Test ('U').
#define LED_STRIP
constexpr int     LED_DIR          = -1;        // LED 0 am linken Ende (+1) bzw. am rechten (-1)
constexpr float   LED_OFS_M        = -0.03f;     // Feinjustage Anzeige [m], falls Leisten- und Wegmitte nicht übereinstimmen (+ = Richtung LED-Ende)
constexpr float   LED_RUN_S        = 4.0f;      // LED-Test: Dauer für die ganze Leiste [s]
#define PIN_LED_STRIP   0       // WS2812-Daten (GPIO0 = Strapping-Pin: beim Start HIGH, Leiste über Pegelwandler oder mit Strom versorgt)
constexpr float   LED_PER_M        = 144.0f;    // LEDs pro Meter
constexpr float   LED_STRIP_M      = 1.0f;      // Länge der Leiste [m] (= Schiene, Mitte der Leiste = Mitte des Verfahrwegs)
constexpr int     LED_COUNT        = (int)(LED_STRIP_M * LED_PER_M + 0.5f);   // -> 144
constexpr float   LED_PITCH_M      = 1.0f / LED_PER_M;   // LED-Abstand [m] (6,94 mm)
constexpr uint8_t LED_SHOW_LEVEL   = 40;        // Helligkeit der Anzeige (0 = Anzeige aus)
constexpr float   LED_IDLE_SHOW_S  = 10.0f;     // Ruhe-Show: Wagen steht, Pendel hängt -> alle x s ein Lauflicht vom Wagen aus (0 = aus)
constexpr float   LED_IDLE_SPEED   = 50.0f;     // Ruhe-Show: Geschwindigkeit des Lauflichts [LEDs/s]
constexpr float   LED_IDLE_LEVEL   = 0.5f;      // Ruhe-Show: Helligkeit 0..1

// ---------------- Homing (sensorlos: Anschlag über Phasenstrom) ----------------
constexpr bool  HOME_ON_START   = false;   // true = beim Einschalten automatisch homen (mit Riemenmarke: nur Mittensuche)
// Riemenmarke (optional): TCS34725 (I2C SDA 21 / SCL 22, Modul-LED an 3,3 V = immer an) schaut nahe der Motorwelle
// auf die glatte Riemenrückseite. Ein Stück des Riemens (gut ein halber Verfahrweg) ist hell markiert, der Rest dunkel;
// die Hell-Dunkel-Grenze dient als Referenz. Einlernen: einmal 'F' (Homing über die Anschläge) -> Schwelle,
// helle Seite und Lage der Kante werden gespeichert. Danach sucht 'H' (und HOME_ON_START) nur noch die Kante.
// CAL_EN des MT6835 dann nur zum Kalibrieren ('K') an GPIO22 stecken (Sensor abziehen).
// #define BELT_SENSOR
constexpr uint8_t BELT_ATIME  = 0xFF;      // Integrationszeit (256-ATIME) x 2,4 ms -> 2,4 ms (schnelle Abtastung)
constexpr uint8_t BELT_GAIN   = 1;         // 0=1x 1=4x 2=16x 3=60x (Übersteuern ist unkritisch)
constexpr float   BELT_MIN_CONTRAST = 0.25f;   // (hell - dunkel) / hell mindestens, sonst nicht eingelernt
constexpr float CSW_FAST = 0.10f;          // [m/s] Suchfahrt zur Kante
constexpr float CSW_SLOW = 0.01f;          // [m/s] letzte Anfahrt, immer von hell nach dunkel
constexpr float CSW_BACK = 0.01f;          // [m] so weit zurück ins Helle vor der langsamen Anfahrt
constexpr float HOME_SPEED      = 0.20f;   // [m/s] Suchgeschwindigkeit
constexpr float HOME_ACCEL      = 0.5f;    // [m/s^2]
constexpr float HOME_I_MAX      = 2.0f;    // [A] Strombegrenzung beim Homing (~4 N)
constexpr float HOME_I_STALL    = 0.8f;    // [A] Anschlag ab |Iq| > ..., muss klar über "Iq frei max" liegen
constexpr float HOME_V_STALL    = 0.01f;   // [m/s]
constexpr float HOME_T_STALL    = 0.15f;   // [s]
// Schiene 900 mm - Wagen 45 mm = ~855 mm Verfahrweg (abzüglich Anschläge/Riemenklemmen)
constexpr float HOME_MIN_TRAVEL = 0.70f;   // [m] Plausibilität Verfahrweg (Schiene 900 mm - Wagen 45 mm)
constexpr float HOME_MAX_TRAVEL = 0.95f;
constexpr float HOME_MARGIN     = 0.06f;   // [m] Notstopp-Abstand zum Anschlag
constexpr float HOME_SOFT_GAP   = 0.08f;   // [m] Arbeitsbereich (Swing-up/Balancieren) innerhalb des Notstopps
constexpr float HOME_TIMEOUT    = 20.0f;   // [s] je Suchfahrt (0,86 m / 0,08 m/s ~ 11 s)

// ---------------- Pendel ----------------
constexpr float PENDULUM_ELL_MAX_M     = 0.27f;  // Plausibilität: ell kann nicht länger als das Pendel sein
constexpr float PENDULUM_ELL_DEFAULT_M = 0.21f;  // Pendel 27 cm + Elektronik bei 22,5 cm (Schätzung), mit 'p' messen!
constexpr float PENDULUM_DAMP_DEFAULT  = 0.05f;  // [1/s], wird mit 'p' gemessen
constexpr float IMU_RADIUS_M           = 0.225f; // Drehpunkt -> BMI270-Chip

// ---------------- IMU-Achsen (Rohachse 0=X 1=Y 2=Z, Vorzeichen +-1) ----------------
// Prüfen mit 'm' (Monitor):
//   Pendel hängt ruhig:          fr ~ -9.8, ft ~ 0, th ~ ±180°
//   Spitze statisch Richtung +x: ft wird negativ, th wird kleiner (180 -> 170)
//   Schwingt Spitze Richtung +x: om negativ (hängend), b bleibt ruhig
constexpr int AX_R_IDX = 1, AX_R_SIGN = -1;    // radial: Drehpunkt -> Spitze
constexpr int AX_T_IDX = 0, AX_T_SIGN = -1;    // tangential: Richtung +th (Zeitlupe-Test 02.11. bestätigt)
constexpr int GYR_IDX  = 2, GYR_SIGN  = -1;    // Drehachse, + = +th

// ---------------- Funk ----------------
constexpr int LATENCY_SAMPLES = 2;             // ~2,5 ms bei 800 Hz
constexpr int MAX_LOST        = 120;           // 150 ms ohne Daten -> Fault (Funklücken bis ~90 ms gemessen, EKF überbrückt)

// ---------------- Pendel-Kalibrierung (Ergebnis im NVS) ----------------
constexpr float CAL_TIME_S        = 5.0f;      // so lange muss das Pendel ruhig hängen
constexpr float CAL_GYRO_STD      = 0.01f;     // [rad/s] max. Streuung je 0,5-s-Block
constexpr float CAL_TIMEOUT_S     = 30.0f;
constexpr float CAL_MAX_OFFSET    = 0.5f;      // [rad] Plausibilität Montagewinkel (sonst Achsen falsch)

// ---------------- Bluetooth (Web-App) ----------------
#define BLE_NAME        "Pendel-Wagen"

// ---------------- OTA (WLAN nur während OTA aktiv, ESP-NOW dann aus) ----------------
#include "secrets.h"            // WIFI_SSID, WIFI_PASS
#ifndef AP_PASS
#define AP_PASS         "pendel123"        // Notfall-AP, falls der Router nicht erreichbar ist
#endif
#define OTA_HOSTNAME    "pendel-wagen"
#define OTA_AP_CHANNEL  1                  // Kanal des Notfall-AP (unabhängig vom ESP-NOW-Kanal)
constexpr uint32_t OTA_TIMEOUT_MS = 300000;    // 5 min ohne Upload -> Neustart

// ---------------- Regler ----------------
// LQR auf [x v th om] mit Stellgröße Wagenbeschleunigung (wird bei Start/nach 'p' neu berechnet)
constexpr float LQR_QX = 400, LQR_QV = 10, LQR_QTH = 400, LQR_QOM = 10, LQR_R = 2.0f;   // R größer = sanfter (0.05 -> 20 Hz, 0.5 -> 10 Hz Schwingen beim Start aus der Hand)
// Wagen-Servo (unterlagert)
constexpr float SERVO_WN = 40.0f, SERVO_ZETA = 0.8f, SERVO_EMAX_M = 0.02f;
constexpr float F_COULOMB_N = 0.0f;            // Haftreibungs-Vorsteuerung, ggf. 0.5..2 N
constexpr float REST_DEG = 0.3f;              // Ruhezone ±0,3°: Pendel steht still -> Motor stromlos (0 = aus)
// Beruhigen (G): a = -DAMP_C * omega * cos(phi) entzieht dem hängenden Pendel Energie, dazu langsam zur Soll-Mitte.
// Sim (l = 0,21 m): 40° Ausschlag in ~3 s ruhig. Größeres DAMP_C dämpft stärker, braucht aber mehr Weg (Arbeitsbereich).
constexpr float DAMP_C      = 1.0f;          // [m/s^2 pro rad/s]  sinnvoll 0,6 ... 1,5
constexpr float DAMP_KX     = 3.0f;          // Zentrierung [1/s^2] (steifer = langsamer, regt das Pendel an)
constexpr float DAMP_KV     = 3.0f;          // [1/s]
constexpr float DAMP_AMAX   = 3.0f;          // [m/s^2]
constexpr float DAMP_CALM_S = 0.5f;          // [s] ruhig -> HALTEN
// Kanal-Abgleich der Strommessung aus dem Strom-Test 'I'. Wiederholte Tests haben sich auf 1,23 aufsummiert
// (Messung streut ~ +-4 %) -> vorerst AUS: gespeicherter Wert wird ignoriert und 'I' misst nur noch.
constexpr bool CS_GAIN_CORR = false;
// Rastmoment-Kompensation (Motor + Riemen): einmal mit 'M' lernen (Wagen fährt ~80 mm langsam vor und zurück)
constexpr bool COG_COMP = true;
constexpr float XS_VMAX = 0.10f;              // Soll-Mitte (Schieberegler): max. Geschwindigkeit [m/s]
constexpr float XS_AMAX = 0.20f;              //   und Beschleunigung [m/s^2]; der Wagen folgt mit Pendel-Verzögerung
constexpr bool  REST_HOLD = true;             // Ruhezone: Wagen festhalten (für reibungsarme Lager); false = stromlos
constexpr float REST_X_MM = 15.0f;            // Ruhezone nur bis ±15 mm um die Soll-Mitte (Austritt bei ±25 mm)
constexpr float OM_LP_HZ = 25.0f;             // Tiefpass auf om in der Rückführung (Spitzen-Vibration 60-400 Hz), 0 = aus
constexpr float A_MAX = 15.0f, V_MAX = 1.5f;    // 12 A ~ 52 N ~ 21 m/s^2 bei 2,5 kg: unter der Stromgrenze bleiben
// Swing-up
constexpr float SU_KE = 1.0f, SU_KX = 15.0f, SU_KV = 4.0f, SU_AMAX = 5.0f;   // 8 -> 5: halbe Spitzenleistung
constexpr float SU_VMAX = 1.0f;                // [m/s] Swing-up-Geschwindigkeit (Netzteil! Leistung ~ F*v)
constexpr float SU_CATCH_TH = 0.35f, SU_CATCH_OM = 4.0f, SU_EMARGIN = 0.4f;   // 0.8 -> 0.4: mit reibungsarmen Lagern kam das Pendel zu schnell oben an (3,8 rad/s bei 9°)
constexpr float BAL_LOST_TH = 0.35f;          // ~20°: Pendel verloren -> bremsen (bei 's' neu aufschwingen)
// Balancierpunkt-Abgleich (gleicht konstanten Winkelfehler aus, Ergebnis im NVS)
constexpr float TRIM_TAU_S = 10.0f;            // Zeitkonstante, 0 = aus
constexpr float TRIM_MAX_RAD = 0.1f;           // max. ±5,7°
// EKF
constexpr float EKF_SIG_GYRO = 0.01f, EKF_SIG_ACC = 0.5f, EKF_Q_OM = 20.0f, EKF_Q_B = 1e-6f, EKF_GATE = 6.0f;
