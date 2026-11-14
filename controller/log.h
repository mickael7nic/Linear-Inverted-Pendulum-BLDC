// log.h – Log: Serial + BLE, Log-Verlauf (übersteht Neustarts), SimpleFOC-Debug, kleine Helfer
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// logf() ist DIE Ausgabefunktion der Firmware: wie printf, schreibt die Zeile auf Serial, in den Log-Verlauf
// und per BLE an die Web-App. Der Verlauf (3000 Zeichen) liegt in RAM, das ein Neustart nicht löscht
// (.noinit) -> nach einem Absturz/Watchdog sieht man in der App die letzten Meldungen davor.
// Achtung: logf() kann bei BLE-Last bis ~200 ms blockieren -> nicht im 800-Hz-Regeltakt aufrufen,
// dort stattdessen ein evt*-Flag setzen (globals.h), der UI-Task loggt dann.
#pragma once

// BLE-Objekte (angelegt in ble.h, hier schon gebraucht, weil logf() direkt sendet)
static NimBLEServer* bleServer = nullptr;
static NimBLECharacteristic *chLog = nullptr, *chTel = nullptr;
static volatile bool bleConnected = false, bleSendHistory = false;   // verbunden / Verlauf beim nächsten UI-Durchlauf senden
static volatile bool bleShutdown = false;   // BLE für OTA beendet
static volatile uint16_t bleMtu = 23;   // ausgehandelte Paketgröße (23 = Standard, bis 247)
static QueueHandle_t cmdQueue;   // Ein-Zeichen-Befehle von Serial/BLE an den UI-Task
// Log-Verlauf in .noinit-RAM: übersteht Neustarts (auch EN-Reset), solange die Versorgung bleibt
constexpr size_t LOG_HIST_SZ = 3000;
struct LogStore { uint32_t magic; uint32_t len; char buf[LOG_HIST_SZ]; };
static __NOINIT_ATTR LogStore logStore;
constexpr uint32_t LOG_MAGIC = 0x4C4F4731;   // "LOG1": Kennung, dass der Puffer gültige Daten enthält
static char* const logHist = logStore.buf;
static uint32_t& logHistLen = logStore.len;
// Schutz des Log-Puffers: logf() kommt aus UI-Task (Kern 0), Regeltask und loop()/OTA (Kern 1).
// Gesperrt wird nur das Kopieren in den Puffer (Mikrosekunden), nicht Serial/BLE.
static SemaphoreHandle_t logMutex = nullptr;

// Beim Start: Ende des alten Logs (vor dem Neustart) behalten und kennzeichnen.
// Nach dem Einschalten ist der Puffer Zufall (magic stimmt nicht) -> leeren. Rückgabe: alter Inhalt vorhanden.
static bool logInit() {
  logMutex = xSemaphoreCreateMutex();
  const bool keep = logStore.magic == LOG_MAGIC && logStore.len <= LOG_HIST_SZ;
  logStore.magic = LOG_MAGIC;
  if (!keep) { logStore.len = 0; return false; }
  static char tmp[1500];
  size_t n = logStore.len, start = n > sizeof(tmp) ? n - sizeof(tmp) : 0;
  // nur die letzten 1500 Zeichen übernehmen, Schnitt an einem Zeilenanfang
  while (start > 0 && start < n && logHist[start - 1] != '\n') start++;   // an Zeilenanfang
  const size_t tl = n - start;
  memcpy(tmp, logHist + start, tl);
  // Steuerzeichen (z. B. halb geschriebene Zeile beim Absturz) unschädlich machen
  for (size_t i = 0; i < tl; i++) if (tmp[i] != '\n' && (tmp[i] < 32 && tmp[i] != '\t')) tmp[i] = '?';
  const char* h1 = "=== Log VOR dem Neustart (Ende) ===\n";
  const char* h2 = "=== Neustart ===\n";
  size_t p = 0;
  memcpy(logHist + p, h1, strlen(h1)); p += strlen(h1);
  memcpy(logHist + p, tmp, tl); p += tl;
  memcpy(logHist + p, h2, strlen(h2)); p += strlen(h2);
  logStore.len = p;
  return tl > 0;
}

// Text in MTU-großen Stücken als BLE-Notification senden. Ist der Sendepuffer voll, bis 40 × 5 ms nachversuchen.
static void bleSendText(const char* s, size_t n) {
  if (bleShutdown || !bleConnected || !chLog) return;
  const size_t chunk = bleMtu > 23 ? bleMtu - 3 : 20;
  for (size_t i = 0; i < n; i += chunk) {
    const size_t k = (n - i < chunk) ? n - i : chunk;
    for (int r = 0; r < 40 && !chLog->notify((const uint8_t*)s + i, k); r++) vTaskDelay(pdMS_TO_TICKS(5));   // bis 200 ms (WLAN-Koexistenz)
  }
}

// printf-artige Log-Ausgabe (Zeilenende wird ergänzt, max. ~200 Zeichen). Aus jedem Task aufrufbar, aber nicht
// aus Interrupts und nicht aus dem Regel-Task-Takt (blockiert).
static void logf(const char* fmt, ...) {
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(line) - 2) n = sizeof(line) - 2;
  if (n == 0 || line[n - 1] != '\n') { line[n++] = '\n'; line[n] = 0; }
  Serial.print(line);
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  if (logHistLen + n > LOG_HIST_SZ) {            // älteste Hälfte verwerfen
    const size_t drop = LOG_HIST_SZ / 2;
    memmove(logHist, logHist + drop, logHistLen - drop);
    logHistLen -= drop;
  }
  memcpy(logHist + logHistLen, line, n);
  logHistLen += n;
  if (logMutex) xSemaphoreGive(logMutex);
  bleSendText(line, n);
}

// Verlauf an die Web-App (beim Verbinden): Kopie unter Sperre ziehen, dann ohne Sperre senden
// (das Senden kann bis zu Sekunden dauern und darf andere logf()-Aufrufe nicht blockieren)
static void logSendHistory() {
  char tmp[LOG_HIST_SZ];
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  const size_t n = logHistLen <= LOG_HIST_SZ ? logHistLen : LOG_HIST_SZ;
  memcpy(tmp, logHist, n);
  if (logMutex) xSemaphoreGive(logMutex);
  bleSendText(tmp, n);
}

// Zeile zum Kopieren in PendelSender.ino
static void logMacLine() {
  logf("static const uint8_t RECEIVER_MAC[6] = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};", ownMac[0],
       ownMac[1], ownMac[2], ownMac[3], ownMac[4], ownMac[5]);
}

// SimpleFOC-Debugausgaben (z. B. Gründe für initFOC-Fehler) zeilenweise ins Log
class LogPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (c == '\r') return 1;
    if (c == '\n' || n >= sizeof(buf) - 1) { buf[n] = 0; if (n) logf("FOC: %s", buf); n = 0; }
    else buf[n++] = (char)c;
    return 1;
  }
 private:
  char buf[160];
  size_t n = 0;
} logPrint;

// Schieberegler: Soll-Mitte setzen; im Ruhezustand (gehomt) Wagen dafür in HOLD nehmen
// Wird aus ble.h ("X<mm>") und Serial aufgerufen, deshalb schon hier.
static void setXSet(float x) {
  ctrl.x_set = x;
  const bool rest = ctrl.mode == pc::Mode::IDLE || ctrl.mode == pc::Mode::FAULT;
  if (rest && homed && focOk && !homing.active() && !otaActive && !pendingCmd) pendingCmd = 'h';
}

// MAC-Adresse als Text "AA:BB:CC:DD:EE:FF" (out braucht 18 Zeichen)
static void macStr(const uint8_t* m, char* out) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}
