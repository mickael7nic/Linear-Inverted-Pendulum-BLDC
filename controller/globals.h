// globals.h – Hardware-Objekte und gemeinsamer Zustand (Tasks, Flags, Messwerte)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Kommunikation zwischen den Tasks läuft fast nur über einfache volatile-Variablen:
//   *Req   = Anforderung ("bitte ausführen"), gesetzt vom UI-Task, abgeholt und zurückgesetzt vom Ausführenden
//   evt*   = Ereignis ("ist passiert"), gesetzt im Regel-Task/loop(), abgeholt vom UI-Task, der es ins Log schreibt
//   tx_*   = Messwerte für die Telemetrie (Regel-Task schreibt, UI-Task liest)
// So muss im schnellen Regel-Task nie gewartet oder geloggt werden. Einzelne bool/float sind auf dem ESP32
// atomar; mehrteilige Daten (z. B. pendK) werden erst geschrieben und dann per Flag (pendApply) freigegeben.
#pragma once

// ---------------- Hardware ----------------
BLDCMotor motor(POLE_PAIRS, MOTOR_PHASE_RESISTANCE_OHM, MOTOR_KV);                  // SimpleFOC-Motor
BLDCDriver3PWM driver(PIN_FOC_IN1, PIN_FOC_IN2, PIN_FOC_IN3, PIN_FOC_EN);            // 3 × IR2104, gemeinsames SD#
MagneticSensorMT6835 encoder(PIN_ENC_CS, SPISettings(ENC_SPI_HZ, MT6835_BITORDER, SPI_MODE3));
#if USE_CURRENT_SENSE
InlineCurrentSense currentSense(SHUNT_OHM, INA240_GAIN, PIN_CUR_A, PIN_CUR_B);     // INA240 an Phase A und B
#endif
Preferences prefs;          // NVS (Flash), Namensraum "pendel": alles, was einen Neustart überleben soll
pc::Controller ctrl;        // Regler: EKF, LQR, Swing-up, Modi (controller.h)
pc::Homing homing;          // Homing-Zustandsautomat (homing.h)

// ---------------- Funk ----------------
static QueueHandle_t rxQueue;                // ESP-NOW-Empfang -> Regel-Task (ImuPacket)
static volatile uint32_t focLoops = 0;     // loop()-Durchläufe (FOC) seit dem letzten Regeltakt
static volatile uint8_t focPerTick = 0;    //   ... davon der letzte Wert (Diagnose, Status '?')
static volatile bool encDiagReq = false;     // UI -> loop(): Encoder-Test
static volatile bool encCalReq = false;      // UI -> loop(): MT6835-Selbstkalibrierung
static volatile bool encCalAbort = false;    // STOPP während der Kalibrierung
static volatile bool telQuiet = false;       // Test: keine Telemetrie, solange der Motor läuft (BLE-Funklast weg)
// Pendel wecken (wake.h): Befehl merken, Wagen kurz rucken lassen, bis der Sender wieder sendet
static char wakeCmd = 0;                     // nach dem Wecken auszuführender Befehl
static int wakeState = 0;                    // 0 aus, 1 HOLD abwarten, 2 Ruck/warten, 3 wach
static uint32_t wakeMs = 0;                  // Zeitpunkt des letzten Zustandswechsels
static volatile uint32_t rxCount = 0, rxBad = 0, lostTotal = 0;   // Pakete empfangen / ungültig / verloren
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // ESP-NOW-Broadcast-Adresse
static uint8_t ownMac[6], senderMac[6];   // eigene MAC -> im Sender als RECEIVER_MAC eintragen
static volatile bool senderLocked = false, evtSender = false;   // erster Sender wird gemerkt, andere ignoriert
static volatile uint32_t lastRxMs = 0;       // millis() des letzten gültigen Pendel-Pakets
// Zeit seit dem letzten Pendel-Paket. lastRxMs ZUERST lesen: wird es (WLAN-Task, anderer Kern) erst nach millis()
// aktualisiert, ergäbe millis() - lastRxMs einen Unterlauf (~49 Tage) -> falsches "Pendel getrennt".
static inline uint32_t msSinceRx() { const uint32_t r = lastRxMs; const uint32_t n = millis(); return (int32_t)(n - r) > 0 ? n - r : 0; }

// ---------------- Zustand ----------------
static volatile char pendingCmd = 0;       // UI -> ctrl-Task: Ein-Zeichen-Befehl (s, b, h, G, x, ...)
static volatile float targetCurrent = 0;   // ctrl-Task -> loop(): Sollstrom Iq [A]
static volatile bool wantEnable = false;   // ctrl-Task -> loop(): Motor einschalten
static volatile float xZeroAngle = 0;      // Motorwinkel [rad] bei x = 0 (Schienenmitte), vom Homing gesetzt
// homed: Position bekannt; homeReq: Homing starten; homeAbort: Homing abbrechen (STOPP)
static volatile bool homed = false, homeReq = false, homeAbort = false;
static volatile bool homeFullReq = false;      // 'F': Homing über die Anschläge erzwingen, lernt die Riemenmarke neu
static volatile bool csLearnedEvt = false;    // Riemenmarke gelernt -> Log
static volatile bool beltNoContrastEvt = false;    // Riemenmarke: Hell/Dunkel zu ähnlich -> Log
static volatile uint16_t beltNegVal = 0, beltPosVal = 0;   //   ... die beiden Messwerte dazu (Anschlag - / +)

static volatile bool evtFocBlocked = false;  // Fahrbefehl abgelehnt, weil FOC nicht läuft
static volatile bool cogCalReq = false, cogAbort = false;   // 'M': Rastmoment lernen (läuft in loop())
static volatile bool csTestReq = false;   // 'I': Test der Strommessung (läuft in loop())
[[maybe_unused]] static float deadtimeU = 0;   // Spannungsverlust durch Brücken-Totzeit [V] (aus 'I')
static bool focOk = false;            // FOC erfolgreich initialisiert (sonst Motor gesperrt)
static const char* focErr = "";       // Fehlertext, falls nicht
// evtNotHomed: Fahrbefehl ohne Homing abgelehnt; evtCalibMoved: Pendel bewegte sich beim Kalibrieren;
// forceIdle: Regler sofort in IDLE (vor Tests/OTA)
static volatile bool evtNotHomed = false, evtCalibMoved = false, forceIdle = false;
static volatile bool otaActive = false;    // Wagen-OTA: Motor aus, ESP-NOW aus
static volatile int evtCalib = 0;          // 1 ok, -1 Timeout/unplausibel, -2 abgebrochen
static volatile int evtInit = 0;           // 1 Schätzer initialisiert
static volatile bool evtPeriod = false;    // Periodenmessung ('p') fertig
static volatile bool pendApply = false;    // neue Pendelparameter (UI berechnet, ctrl übernimmt)
static float pendEll, pendDamp, pendK[4];  //   ... Ersatzlänge [m], Dämpfung, neue LQR-Verstärkungen
// LQR-Gewichte zur Laufzeit ("Q<qx>,<qv>,<qth>,<qom>,<r>", "Q0" = Standard aus config.h), im NVS "lqr"
static volatile bool lqrReq = false;
static volatile float ellReq = 0;            // "Y<mm>" -> ell setzen
static pc::LQRWeights lqrNew;

// Pendel-Kalibrierung: Montagewinkel delta (IMU gegen Pendelachse) und Gyro-Bias, im NVS
static volatile float calDelta = 0, calBias = 0, calNewDelta = 0, calNewBias = 0;
static volatile float angOfs = 0;            // Winkel-Offset zur Kalibrierung [rad] (Schieberegler, NVS "ang_o")
constexpr float ANG_OFS_MAX = 3.0f * DEG_TO_RAD;
static volatile bool calStored = false;      // Kalibrierung im NVS vorhanden

// Telemetrie (Regel-Task -> UI-Task -> BLE): Wagenposition/-geschwindigkeit, Pendel-Rohwerte, Akku, Sender-Flags
static volatile float tx_x, tx_v, tx_fr, tx_ft, tx_gyro;
static volatile uint16_t tx_vbat = 0;
static volatile uint8_t tx_flags = 0;
