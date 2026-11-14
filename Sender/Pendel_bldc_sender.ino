// PendelSender.ino – ESP32-C6 Super Mini + BMI270 (SPI) in der Pendelspitze
// Liest BMI270 mit 800 Hz und sendet Rohdaten per ESP-NOW (Unicast) an den Wagen.
// Standard: Broadcast ohne ACK (ESPNOW_UNICAST). Der Wagen filtert auf die Absender-MAC.
// Die gesamte Sensorfusion läuft auf dem Wagen-ESP32 (dort ist die Wagenbeschleunigung bekannt).
// Kommandos vom Wagen (nur von RECEIVER_MAC):
//   PAUSE  -> nicht mehr senden (Wagen-OTA), RESUME bzw. nach 10 min wieder senden
// Kein Bluetooth, kein OTA (Flashen per USB).
//
// Bibliothek: "SparkFun BMI270 Arduino Library" (nur für Init/Config-Upload)
// Board: ESP32C6 Dev Module, USB CDC On Boot: Enabled, CPU Frequency: 80 MHz (wird auch im Code gesetzt)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <soc/soc_caps.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "protocol.h"

// ---------------- Pins ESP32-C6 Super Mini (Strapping-Pins 8, 9, 15 nur für LEDs) ----------------
constexpr uint32_t CPU_MHZ = 80;       // reicht für 800 Hz SPI + ESP-NOW, spart einige mA
#define IMU_USE_SPI    1       // 1 = SPI (empfohlen), 0 = I2C
// SPI (BMI270: SDx=MOSI, SDO=MISO, SCx=SCK, CSB=CS)
#define PIN_SPI_SCK    5
#define PIN_SPI_MISO   3
#define PIN_SPI_MOSI   4
#define PIN_IMU_CS     6
constexpr uint32_t SPI_CLOCK_HZ = 10000000;  // BMI270 max. 10 MHz
// I2C (nur bei IMU_USE_SPI 0)
#define PIN_SDA        3
#define PIN_SCL        5
#define PIN_IMU_INT1   7       // BMI270 INT1 (Data-Ready). -1 = Polling
#define PIN_VBAT       0       // ADC1_CH0, Teiler am Akku (VBAT_DIVIDER), -1 = keiner
#define PIN_LED        15      // einfache LED: kurzer Blitz 1x/s, solange gesendet wird
constexpr bool LED_ACTIVE_LOW = false;       // PRÜFEN: leuchtet sie dauerhaft -> true
#define PIN_RGB        8       // WS2812: Status (Farbcodes siehe unten)
constexpr uint8_t RGB_BRIGHT = 20;           // 0..255, niedrig halten (Strom, Blendung)
// RGB-Status (Priorität von oben nach unten):
//   rot, schnell blinkend      BMI270 Init fehlgeschlagen (Sender bleibt stehen)
//   magenta, schnell blinkend  ESP-NOW Init fehlgeschlagen (Sender bleibt stehen)
//   rot, langsam blinkend      BMI270 liefert keine Daten mehr
//   blau, langsam blinkend     kein Heartbeat vom Wagen (aus, falsche MAC/Kanal, empfängt nichts)
//   cyan, dauerhaft            pausiert (Wagen im OTA-Modus)
//   orange, langsam blinkend   Akku schwach
//   grün, kurzer Blitz alle 2 s  alles ok
//   weiß                       Start/Initialisierung
constexpr float   VBAT_DIVIDER   = 2.03f;    // Teiler 2,0, kalibriert: 4,250 V gemessen / 4,19 V angezeigt
constexpr uint16_t VBAT_LOW_MV   = 3450;
constexpr uint16_t VBAT_OK_MV    = 3550;     // Hysterese
constexpr uint16_t VBAT_CRIT_MV  = 3300;     // darunter (5 s lang) -> Tiefschlaf, Schutz vor Tiefentladung
constexpr uint16_t VBAT_RESTART_MV = 3700;   // nach Tiefschlaf erst ab dieser Spannung wieder starten (geladen)
constexpr uint16_t VBAT_NONE_MV  = 2500;     // darunter: kein Akku/Teiler -> nie schlafen
constexpr uint32_t SLEEP_CHECK_MIN = 10;     // im Tiefschlaf alle n min kurz Spannung prüfen (0 = nie, nur Reset)
// Ruhe-Tiefschlaf: Pendel bewegt sich IDLE_SLEEP_S lang nicht (und der Wagen ist nicht aktiv) -> Tiefschlaf.
// Aufwachen: BMI270 Any-Motion an INT1 (Lagewechsel um ein paar Grad bzw. Anstoßen).
constexpr uint32_t IDLE_SLEEP_S   = 60;       // 0 = aus
constexpr float    IDLE_TILT_DEG  = 2.0f;     // Ruhe: Lage bleibt innerhalb dieses Winkels ...
constexpr float    IDLE_GYRO_DPS  = 8.0f;     // ... und Drehrate darunter
constexpr uint16_t WAKE_THRESH    = 100;      // Any-Motion-Schwelle, 0,48 mg/LSB: 100 = 48 mg ~ 2,8° Lageänderung
constexpr uint16_t WAKE_DURATION  = 2;        // x 20 ms über der Schwelle
constexpr int      HANG_AXIS = 1, HANG_SIGN = 1;   // Rohachse, die beim hängenden Pendel +1 g zeigt (aufrecht -1 g)
constexpr uint8_t BMI270_ADDR    = 0x68;     // 0x69 wenn SDO = High
constexpr uint32_t I2C_CLOCK_HZ  = 1000000;  // BMI270 kann 1 MHz (Fast-mode Plus)

// ---------------- MAC des Wagens (Ausgabe "Wagen-MAC:" beim Start des Wagens) ----------------
static const uint8_t RECEIVER_MAC[6] = {0x38, 0x3E, 0x51, 0xC4, 0x34, 0x40};
// 0 = Broadcast: kein ACK, keine Wiederholungen -> konstante Latenz, verlorene Pakete überbrückt der EKF
// 1 = Unicast an RECEIVER_MAC: MAC-ACK + automatische Wiederholungen (zuverlässiger, aber Latenz-Jitter)
#define ESPNOW_UNICAST 0
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t* const DEST = ESPNOW_UNICAST ? RECEIVER_MAC : BCAST;

constexpr uint32_t PAUSE_TIMEOUT_MS = 600000;   // Pause (Wagen-OTA) endet spätestens nach 10 min

// ---------------- BMI270-Register ----------------
enum : uint8_t {
  REG_ERR = 0x02, REG_STATUS = 0x03, REG_DATA_ACC = 0x0C,
  REG_ACC_CONF = 0x40, REG_ACC_RANGE = 0x41, REG_GYR_CONF = 0x42, REG_GYR_RANGE = 0x43,
  REG_INT1_IO_CTRL = 0x53, REG_INT_MAP_DATA = 0x58, REG_PWR_CONF = 0x7C, REG_PWR_CTRL = 0x7D
};

BMI270 imu;
static TaskHandle_t imuTask = nullptr;
static volatile uint32_t sendErr = 0, sent = 0;

static volatile uint32_t ackOk = 0, ackFail = 0;   // ACKs vom Wagen
static volatile bool paused = false;
static volatile uint32_t pauseMs = 0;
static volatile uint32_t lastAliveMs = 0;    // letzter ALIVE vom Wagen
static volatile bool cartBusy = false;       // Wagen aktiv (Motor/Homing/Kalibrierung) -> nicht einschlafen
static volatile uint32_t lastBusyMs = 0;     // letzte "aktiv"-Meldung des Wagens
static volatile bool idleSleep = false;      // -> loop() schickt den ESP in den Ruhe-Tiefschlaf
static volatile uint32_t lastImuMs = 0;      // letztes neues IMU-Sample
static volatile bool vbatLow = false;
static volatile bool vbatCrit = false;   // -> loop() schickt den ESP in den Tiefschlaf
static volatile int16_t dbgAcc[3], dbgGyr[3];   // letztes Sample für die Testausgabe
static volatile uint16_t dbgVbat = 0;

#include "status_led.h"   // WS2812-Statusfarben (eigene Datei wegen Arduino-Prototypen)

static void led(bool on) { digitalWrite(PIN_LED, (on != LED_ACTIVE_LOW) ? HIGH : LOW); }

// Init-Fehler: Farbcode blinken, nie zurückkehren
static void fatal(St st, const char* msg) {
  Serial.println(msg);
  led(false);
  for (;;) { showStatus(st); delay(20); }
}

static void onCmd(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(CmdPacket) || memcmp(info->src_addr, RECEIVER_MAC, 6) != 0) return;
  CmdPacket c;
  memcpy(&c, data, sizeof(c));
  if (c.magic != CMD_MAGIC) return;
  if (c.cmd == CMD_PAUSE) { if (!paused) { paused = true; pauseMs = millis(); } }
  else if (c.cmd == CMD_RESUME) paused = false;
  else if (c.cmd == CMD_ALIVE || c.cmd == CMD_RESUME) {
    if (c.cmd == CMD_ALIVE) lastAliveMs = millis();
    paused = false;
    cartBusy = c.arg & CMD_ARG_BUSY;
    if (cartBusy) lastBusyMs = millis();
  }
}

#if IMU_USE_SPI
static const SPISettings imuSpi(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0);

static void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(imuSpi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(450);   // BMI270 braucht >450 us zwischen Schreibzugriffen im Low-Power-Zustand
}

// SPI-Lesen: Adresse | 0x80, dann ein Dummy-Byte, dann Daten
static bool readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  uint8_t tx[2 + 32] = {(uint8_t)(reg | 0x80), 0};
  uint8_t rx[2 + 32];
  if (n > 32) return false;
  SPI.beginTransaction(imuSpi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transferBytes(tx, rx, n + 2);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  memcpy(buf, rx + 2, n);
  return true;
}
#else
static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI270_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(450);
}

static bool readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(BMI270_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMI270_ADDR, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
#endif

static void IRAM_ATTR onDrdy() {
  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR(imuTask, &hp);
  if (hp) portYIELD_FROM_ISR();
}

static bool setupImu() {
#if IMU_USE_SPI
  if (imu.beginSPI(PIN_IMU_CS, SPI_CLOCK_HZ, SPI) != BMI2_OK) return false;   // lädt Config-Datei (8 kB)
#else
  if (imu.beginI2C(BMI270_ADDR, Wire) != BMI2_OK) return false;
#endif
  writeReg(REG_PWR_CONF, 0x00);       // Advanced Power Save aus
  writeReg(REG_ACC_CONF, 0x8B);       // filter_perf=1, bwp=OSR4 (~80 Hz statt ~320 Hz: Vibrationen nicht einfalten), ODR 800 Hz
  writeReg(REG_ACC_RANGE, 0x03);      // ±16 g (Zentripetal beim Swing-up)
  writeReg(REG_GYR_CONF, 0xCB);       // filter_perf=1, noise_perf=1, bwp=OSR4 (~75 Hz statt ~300 Hz), ODR 800 Hz
  writeReg(REG_GYR_RANGE, 0x00);      // ±2000 °/s
  writeReg(REG_PWR_CTRL, 0x0E);       // Acc + Gyro + Temp an
  if (PIN_IMU_INT1 >= 0) {
    writeReg(REG_INT1_IO_CTRL, 0x0A); // Ausgang an, Push-Pull, aktiv High
    writeReg(REG_INT_MAP_DATA, 0x04); // DRDY -> INT1
  }
  delay(50);
  uint8_t chk[4];
  readRegs(REG_ACC_CONF, chk, 4);
  Serial.printf("BMI270 ACC_CONF=%02X ACC_RANGE=%02X GYR_CONF=%02X GYR_RANGE=%02X\n", chk[0], chk[1], chk[2], chk[3]);
  return chk[0] == 0x8B && chk[1] == 0x03 && chk[2] == 0xCB && chk[3] == 0x00;
}

static void onSendStatus(esp_now_send_status_t st) {
  if (st == ESP_NOW_SEND_SUCCESS) ackOk = ackOk + 1;
  else ackFail = ackFail + 1;
}

static bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);                     // kein Modem-Sleep -> keine Latenzspitzen
  esp_wifi_set_country_code("DE", true);             // Kanäle 1-13 (Standard "01" erlaubt nur 1-11)
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    Serial.printf("WARNUNG: ESP-NOW-Kanal %d nicht gesetzt\n", ESPNOW_CHANNEL);
  WiFi.setTxPower(WIFI_POWER_15dBm);                 // 8,5 dBm: ~20 % Paketverlust beim Balancieren
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onCmd);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)   // Callback-Signatur ab Arduino-ESP32 3.3
  esp_now_register_send_cb([](const esp_now_send_info_t*, esp_now_send_status_t st) { onSendStatus(st); });
#else
  esp_now_register_send_cb([](const uint8_t*, esp_now_send_status_t st) { onSendStatus(st); });
#endif
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DEST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  // 12 Mbit/s OFDM statt 1 Mbit/s: ~10x kürzere Luftzeit pro Paket
  esp_now_rate_config_t rc = {};
  rc.phymode = WIFI_PHY_MODE_11G;
  rc.rate = WIFI_PHY_RATE_12M;
  if (esp_now_set_peer_rate_config(DEST, &rc) != ESP_OK)
    Serial.println("Warnung: PHY-Rate nicht gesetzt (1 Mbit/s)");
  return true;
}

static void imuLoop(void*) {
  ImuPacket pkt = {};
  pkt.magic = PENDEL_MAGIC;
  uint16_t seq = 0;
  uint16_t vbat = 0;
  uint32_t lastVbat = 0;
  uint8_t buf[21];   // 0x03 (STATUS) .. 0x17 (GYR_Z_MSB)
  for (;;) {
    if (PIN_IMU_INT1 >= 0) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3));
    else delayMicroseconds(150);

    if (!readRegs(REG_STATUS, buf, sizeof(buf))) { pkt.flags |= PKT_FLAG_IMU_ERR; continue; }
    if (!(buf[0] & 0x40)) continue;       // drdy_gyr: noch keine neuen Daten
    const uint8_t* d = buf + (REG_DATA_ACC - REG_STATUS);
    for (int i = 0; i < 3; i++) {
      pkt.acc[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
      pkt.gyr[i] = (int16_t)(d[6 + 2 * i] | (d[6 + 2 * i + 1] << 8));
    }
    pkt.seq = seq++;
    lastImuMs = millis();
    for (int i = 0; i < 3; i++) { dbgAcc[i] = pkt.acc[i]; dbgGyr[i] = pkt.gyr[i]; }
    if (PIN_VBAT >= 0 && millis() - lastVbat > 100) {
      lastVbat = millis();
      static float vf = 0;
      static int critCnt = 0;
      const float raw = analogReadMilliVolts(PIN_VBAT) * VBAT_DIVIDER;
      vf = vf ? vf + 0.2f * (raw - vf) : raw;          // glätten (Sendespitzen)
      vbat = (uint16_t)vf;
      critCnt = (vbat > VBAT_NONE_MV && vbat < VBAT_CRIT_MV) ? critCnt + 1 : 0;
      if (critCnt >= 50) vbatCrit = true;             // 5 s ununterbrochen kritisch
    }
    if (vbatCrit) pkt.flags |= PKT_FLAG_SHUTDOWN;
    if (idleSleep) pkt.flags |= PKT_FLAG_SLEEP;
    if (paused && !vbatCrit) continue;
    pkt.vbat_mV = vbat;
    dbgVbat = vbat;
    if (vbat && vbat < VBAT_LOW_MV) vbatLow = true;
    else if (vbat > VBAT_OK_MV) vbatLow = false;
    if (vbatLow) pkt.flags |= PKT_FLAG_VBAT_LOW;
    else pkt.flags &= ~PKT_FLAG_VBAT_LOW;

    if (esp_now_send(DEST, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) sent = sent + 1;
    else sendErr = sendErr + 1;
    pkt.flags &= ~PKT_FLAG_IMU_ERR;
  }
}

// C6: gpio_hold_en() hält auch im Tiefschlaf; ältere Chips brauchen zusätzlich den globalen Schalter
static void dslpHold(bool en) {
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
  if (en) gpio_deep_sleep_hold_en(); else gpio_deep_sleep_hold_dis();
#else
  (void)en;
#endif
}

static uint16_t readVbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(PIN_VBAT);
  return (uint16_t)(mv / 16 * VBAT_DIVIDER);
}

// BMI270 für den Ruhe-Tiefschlaf: nur Acc (Low-Power, 50 Hz), Any-Motion -> INT1 (gelatcht, aktiv High)
static bool imuMotionWake() {
  writeReg(REG_INT_MAP_DATA, 0x00);                 // Data-Ready von INT1 nehmen
  writeReg(REG_PWR_CTRL, 0x04);                     // nur Acc (Gyro/Temp aus)
  writeReg(REG_ACC_CONF, 0x17);                     // filter_perf=0 (Low-Power), avg4, ODR 50 Hz
  bool ok = imu.enableFeature(BMI2_ANY_MOTION) == BMI2_OK;
  bmi2_sens_config mc;
  mc.type = BMI2_ANY_MOTION;
  ok &= imu.getConfig(&mc) == BMI2_OK;
  mc.cfg.any_motion.duration = WAKE_DURATION;
  mc.cfg.any_motion.threshold = WAKE_THRESH;
  mc.cfg.any_motion.select_x = mc.cfg.any_motion.select_y = mc.cfg.any_motion.select_z = BMI2_ENABLE;
  ok &= imu.setConfig(mc) == BMI2_OK;
  ok &= imu.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1) == BMI2_OK;
  bmi2_int_pin_config pc = {};
  pc.pin_type = BMI2_INT1;
  pc.int_latch = BMI2_INT_LATCH;                    // bleibt High bis zum Auslesen -> sicherer Weckimpuls
  pc.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  pc.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  pc.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  pc.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
  ok &= imu.setInterruptPinConfig(pc) == BMI2_OK;
  delay(100);                                        // Bezugswert setzen lassen
  uint16_t st;
  imu.getInterruptStatus(&st);                       // evtl. alten Latch löschen
  writeReg(REG_PWR_CONF, 0x01);                      // Advanced Power Save
  return ok;
}

// Tiefschlaf: Funk aus, LEDs aus, Pins gehalten.
// motionWake = false (Akku kritisch): BMI270 aus, Aufwachen nur per Reset bzw. Timer-Prüfung.
// motionWake = true (Ruhe): BMI270 überwacht Bewegung und weckt über INT1.
static void deepSleepNow(bool imuRunning, bool motionWake) {
  if (imuRunning) {
    if (PIN_IMU_INT1 >= 0) detachInterrupt(digitalPinToInterrupt(PIN_IMU_INT1));
    vTaskSuspend(imuTask);
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
  }
  if (motionWake && PIN_IMU_INT1 >= 0) {
    if (!imuMotionWake()) Serial.println("Any-Motion-Konfiguration fehlgeschlagen");
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_IMU_INT1, ESP_EXT1_WAKEUP_ANY_HIGH);
  } else if (imuRunning) {
#if IMU_USE_SPI
    writeReg(REG_PWR_CTRL, 0x00);   // Acc/Gyro/Temp aus
    writeReg(REG_PWR_CONF, 0x03);   // Advanced Power Save -> ~3,5 uA
#endif
  }
  const Rgb c = motionWake ? Rgb{0, 0, 255} : Rgb{255, 0, 0};   // blau = Ruhe, rot = Akku
  for (int i = 0; i < 3; i++) { rgbRaw(c); delay(150); rgbRaw({0, 0, 0}); delay(150); }
  rgbRaw({0, 0, 0});
  led(false);
  Serial.flush();
  // Pegel im Tiefschlaf halten: CS high (BMI270 bleibt ruhig), LEDs aus
  pinMode(PIN_IMU_CS, OUTPUT); digitalWrite(PIN_IMU_CS, HIGH);
  pinMode(PIN_RGB, OUTPUT);    digitalWrite(PIN_RGB, LOW);
  const int keep[] = {PIN_IMU_CS, PIN_RGB, PIN_LED};
  for (int p : keep) gpio_hold_en((gpio_num_t)p);
  dslpHold(true);
  if (!motionWake && SLEEP_CHECK_MIN) esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_CHECK_MIN * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  // gehaltene Pins aus dem Tiefschlaf freigeben
  dslpHold(false);
  const int held[] = {PIN_IMU_CS, PIN_RGB, PIN_LED};
  for (int p : held) gpio_hold_dis((gpio_num_t)p);
  // Akku-Prüfung vor allem anderen: nach Timer-Weckruf erst ab VBAT_RESTART_MV wieder starten
  if (PIN_VBAT >= 0) {
    const bool timerWake = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
    const uint16_t mv = readVbat();
    if (mv > VBAT_NONE_MV && mv < (timerWake ? VBAT_RESTART_MV : VBAT_CRIT_MV)) {
      pinMode(PIN_LED, OUTPUT);
      if (timerWake) {        // ohne LED-Signal sofort wieder schlafen
        gpio_hold_en((gpio_num_t)PIN_IMU_CS); gpio_hold_en((gpio_num_t)PIN_RGB); gpio_hold_en((gpio_num_t)PIN_LED);
        dslpHold(true);
        if (SLEEP_CHECK_MIN) esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_CHECK_MIN * 60ULL * 1000000ULL);
        esp_deep_sleep_start();
      }
      Serial.begin(115200);
      delay(300);
      Serial.printf("Akku kritisch (%u mV) - Tiefschlaf\n", mv);
      deepSleepNow(false, false);
    }
  }
  setCpuFrequencyMhz(CPU_MHZ);
  const bool motionWoke = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
  pinMode(PIN_LED, OUTPUT);
  led(false);
  showStatus(St::BOOT);
  Serial.begin(115200);
  delay(300);
  if (motionWoke) Serial.println("Aufgewacht durch Bewegung");
  if (PIN_VBAT >= 0) {   // zum Prüfen von VBAT_DIVIDER mit dem Multimeter (am USB: Ladespannung!)
    uint32_t adc = 0;
    for (int i = 0; i < 16; i++) adc += analogReadMilliVolts(PIN_VBAT);
    adc /= 16;
    Serial.printf("Akku: %lu mV (ADC %lu mV x %.2f)\n", (unsigned long)(adc * VBAT_DIVIDER), (unsigned long)adc,
                  VBAT_DIVIDER);
  }
#if IMU_USE_SPI
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
#else
  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLOCK_HZ);
#endif
  if (!setupImu()) fatal(St::ERR_IMU_INIT, "BMI270 Init fehlgeschlagen - Verdrahtung pruefen");
  if (!setupEspNow()) fatal(St::ERR_NOW_INIT, "ESP-NOW Init fehlgeschlagen");
  Serial.printf("Sender bereit, CPU %lu MHz, eigene MAC %s, Kanal %d\n", (unsigned long)getCpuFrequencyMhz(),
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
  if (PIN_VBAT >= 0) {
    uint32_t mv = 0;
    for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(PIN_VBAT);
    const float v = mv / 16.0f * VBAT_DIVIDER / 1000.0f;
    Serial.printf("LiPo: %.2f V%s\n", v, v * 1000.0f < VBAT_LOW_MV ? "  (SCHWACH)" : "");
  } else {
    Serial.println("LiPo: keine Messung (PIN_VBAT = -1)");
  }
  Serial.printf("Ziel (Wagen): %02X:%02X:%02X:%02X:%02X:%02X\n", RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
                RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);
  xTaskCreate(imuLoop, "imu", 4096, nullptr, 10, &imuTask);   // C6: ein HP-Kern
  if (PIN_IMU_INT1 >= 0) {
    pinMode(PIN_IMU_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT1), onDrdy, RISING);
  }
}

// Ruhe erkennen: Lage (Beschleunigungsrichtung) und Drehrate praktisch konstant, Wagen nicht aktiv
static void checkIdle() {
  if (!IDLE_SLEEP_S || PIN_IMU_INT1 < 0) return;
  static uint32_t stillSince = 0, lastChk = 0;
  static float ref[3] = {0, 0, 0};
  const uint32_t now = millis();
  if (now - lastChk < 20) return;
  lastChk = now;
  float a[3], n = 0, g = 0;
  for (int i = 0; i < 3; i++) { a[i] = dbgAcc[i]; n += a[i] * a[i]; g = fmaxf(g, fabsf(dbgGyr[i] / GYR_LSB_PER_DPS)); }
  n = sqrtf(n);
  if (n < 100) return;                                  // noch keine Daten
  for (int i = 0; i < 3; i++) a[i] /= n;
  const float dot = a[0] * ref[0] + a[1] * ref[1] + a[2] * ref[2];
  // nicht schlafen, solange der Wagen in den letzten 30 s "aktiv" gemeldet hat (einzelne verlorene Heartbeats egal)
  const bool busy = lastBusyMs != 0 && now - lastBusyMs < 30000;
  // nie aufrecht einschlafen (Balancieren): Radialachse (Y) zeigt dann nach oben -> ay ~ -1 g (hängend +1 g).
  // Jede andere Lage (hängend, zur Seite gedreht zum Laden, flach) darf schlafen.
  const bool upright = dbgAcc[HANG_AXIS] * HANG_SIGN < -0.7f * ACC_LSB_PER_G;
  if (busy || upright || g > IDLE_GYRO_DPS || dot < cosf(IDLE_TILT_DEG * DEG_TO_RAD)) {
    for (int i = 0; i < 3; i++) ref[i] = a[i];          // Bewegung: neuer Bezug, Zeit neu
    stillSince = now;
    return;
  }
  if (now - stillSince >= IDLE_SLEEP_S * 1000UL && !idleSleep) {
    Serial.printf("Keine Bewegung seit %lu s - Tiefschlaf (Aufwachen durch Bewegung)\n", (unsigned long)IDLE_SLEEP_S);
    idleSleep = true;                                   // ~0,2 s lang SLEEP-Flag an den Wagen
    delay(200);
    deepSleepNow(true, true);
  }
}

void loop() {
  if (vbatCrit) {
    Serial.printf("Akku kritisch (%u mV) - Wagen informiert, Tiefschlaf\n", (unsigned)dbgVbat);
    delay(200);                      // ~160 Pakete mit SHUTDOWN-Flag an den Wagen
    deepSleepNow(true, false);
  }
  if (paused && millis() - pauseMs > PAUSE_TIMEOUT_MS) paused = false;
  checkIdle();
  // Status (Priorität: IMU, Wagen, Pause, Akku)
  const uint32_t now = millis();
  St st = St::OK;
  if (now - lastImuMs > 100) st = St::ERR_IMU;
  else if (paused) st = St::PAUSED;
  else if (now - lastAliveMs > 2000) st = St::NO_CART;
  else if (vbatLow) st = St::VBAT_LOW;
  showStatus(st);
  // Testausgabe IMU (10 Hz, Serial-Plotter-Format), solange der Wagen nicht antwortet
  static uint32_t lastDbg = 0;
  if (now - lastAliveMs > 2000 && now - lastDbg >= 100) {
    lastDbg = now;
    const float ka = 1.0f / ACC_LSB_PER_G, kg = 1.0f / GYR_LSB_PER_DPS;
    Serial.printf("ax:%.3f ay:%.3f az:%.3f gx:%.2f gy:%.2f gz:%.2f vbat:%u\n", dbgAcc[0] * ka, dbgAcc[1] * ka,
                  dbgAcc[2] * ka, dbgGyr[0] * kg, dbgGyr[1] * kg, dbgGyr[2] * kg, dbgVbat);
  }
  // einfache LED: Sendeaktivität (30-ms-Blitz pro Sekunde, solange Pakete rausgehen)
  static uint32_t actSent = 0, actMs = 0;
  if (now - actMs >= 1000) { actMs = now; led(sent != actSent); actSent = sent; }
  else if (now - actMs >= 30) led(false);
  static uint32_t last = 0, lastSent = 0, lastOk = 0, lastFail = 0;
  if (millis() - last >= 2000) {
    last = millis();
    uint32_t s = sent;
    const uint32_t ok = ackOk, fail = ackFail;
    if (paused) Serial.println("pausiert (Wagen im OTA-Modus)");
    else if (!ESPNOW_UNICAST) Serial.printf("Pakete/s: %.0f (Broadcast)  Sendefehler: %lu\n", (s - lastSent) / 2.0f,
                                            (unsigned long)sendErr);
    else Serial.printf("Pakete/s: %.0f  ACK-Quote: %.1f %%%s\n", (s - lastSent) / 2.0f,
                  (ok + fail - lastOk - lastFail) ? 100.0f * (ok - lastOk) / (ok + fail - lastOk - lastFail) : 0.0f,
                  ok == lastOk ? "  WAGEN ANTWORTET NICHT (MAC/Kanal pruefen)" : "");
    lastOk = ok; lastFail = fail;
    lastSent = s;
  }
  delay(10);
}
