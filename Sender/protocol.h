// protocol.h – ESP-NOW-Paket Pendel -> Wagen (identische Kopie in beiden Sketch-Ordnern)
#pragma once
#include <stdint.h>

#define PENDEL_MAGIC     0xA55A
#define ESPNOW_CHANNEL   13         // fester Kanal, auf beiden Seiten gleich (13: Abstand zu FRITZ!Box 1-7 und Nachbarn 4-12)

constexpr float IMU_RATE_HZ      = 800.0f;   // BMI270-ODR = Paketrate = Reglertakt
constexpr float ACC_LSB_PER_G    = 2048.0f;  // ±16 g
constexpr float GYR_LSB_PER_DPS  = 16.384f;  // ±2000 °/s

enum : uint8_t { PKT_FLAG_VBAT_LOW = 0x01, PKT_FLAG_IMU_ERR = 0x02, PKT_FLAG_SHUTDOWN = 0x04, PKT_FLAG_SLEEP = 0x08 };
// SHUTDOWN: Akku kritisch -> Tiefschlaf.  SLEEP: keine Bewegung -> Tiefschlaf, Aufwachen durch Bewegung

struct __attribute__((packed)) ImuPacket {
  uint16_t magic;
  uint16_t seq;        // +1 pro IMU-Sample -> Empfänger erkennt Verluste
  int16_t  acc[3];     // Rohwerte Sensorachsen X,Y,Z
  int16_t  gyr[3];
  uint16_t vbat_mV;    // 0 = nicht gemessen
  uint8_t  flags;
  uint8_t  reserved;
};
static_assert(sizeof(ImuPacket) == 20, "ImuPacket Groesse");

// Kommandos Wagen -> Pendel (Broadcast, Absender wird im Pendel geprüft)
#define CMD_MAGIC        0xC0DE
enum : uint8_t { CMD_PAUSE = 1, CMD_RESUME = 2, CMD_ALIVE = 3 };   // ALIVE: Wagen empfängt Daten (2 Hz)
struct __attribute__((packed)) CmdPacket {
  uint16_t magic;
  uint8_t  cmd;
  uint8_t  arg;         // bei ALIVE: CMD_ARG_BUSY = Wagen aktiv (Motor/Homing/Kalibrierung) -> Pendel schläft nicht
};
#define CMD_ARG_BUSY     0x01
static_assert(sizeof(CmdPacket) == 4, "CmdPacket Groesse");

// Telemetrie Wagen -> Web-App (BLE-Notify, 50 Hz)
enum : uint8_t {
  TF_HOMED = 0x01, TF_EST_OK = 0x02, TF_CAL_STORED = 0x04, TF_PENDEL_LINK = 0x08,
  TF_HOMING = 0x10, TF_CALIBRATING = 0x20, TF_OTA = 0x40, TF_VBAT_LOW = 0x80
};
struct __attribute__((packed)) Telemetry {
  uint32_t t_ms;
  int16_t  x;          // 0,1 mm
  int16_t  v;          // mm/s
  int16_t  th;         // 0,01 deg, 0 = oben
  int16_t  om;         // mrad/s
  int16_t  a;          // cm/s^2 Sollbeschleunigung
  int16_t  iq;         // 10 mA
  uint8_t  mode;       // pc::Mode, Bit 7 = Ruhezone aktiv
  uint8_t  flags;      // TF_*
  uint16_t vbat;       // mV Pendel-Akku
  int16_t  xs;         // 0,1 mm Soll-Mittelposition (Schieberegler, rampengeführt)
  int16_t  trim;       // 0,01 deg Winkel-Offset zur Kalibrierung (Schieberegler)
};
static_assert(sizeof(Telemetry) == 24, "Telemetry Groesse");
