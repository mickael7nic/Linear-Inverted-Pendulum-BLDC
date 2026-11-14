// ble.h – Bluetooth (NimBLE): Befehle, Log- und Telemetrie-Kanal
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Ein BLE-Dienst nach dem Muster "Nordic UART" mit drei Kanälen (Characteristics):
//   RX  (App -> Wagen, write):  Ein-Zeichen-Befehle oder Befehle mit Wert ("X120", "Q400,10,400,10,2", ...)
//   LOG (Wagen -> App, notify): Textzeilen aus logf(), beim Abonnieren zuerst der Verlauf
//   TEL (Wagen -> App, notify): binäre Telemetrie (struct Telemetry, protocol.h), siehe telemetry.h
// Die Web-App (index.html) verbindet sich per Web Bluetooth mit genau diesen UUIDs.
// Die Callbacks laufen im NimBLE-Task: hier nur Werte/Flags setzen, nichts Langsames.
#pragma once

// UUIDs: müssen mit index.html übereinstimmen
#define BLE_SVC "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_RX  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_LOG "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TEL "6e400004-b5a3-f393-e0a9-e50e24dcca9e"

// Verbindungsereignisse: nach dem Trennen sofort wieder sichtbar werden (Advertising)
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    bleConnected = true;
    // Verbindungsintervall 30..50 ms, Timeout 3 s. Kurze Intervalle würden dem ESP-NOW-Empfang
    // (gleiche Funkhardware) Sendezeit wegnehmen -> Paketverluste beim Pendel.
    s->updateConnParams(ci.getConnHandle(), 24, 40, 0, 300);   // 30..50 ms: wenig Funkzeit neben ESP-NOW
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    bleConnected = false;
    bleMtu = 23;
    if (!bleShutdown) NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override { bleMtu = mtu; }
};

// Eingang von der App. Befehle mit Wert werden hier direkt ausgewertet (ein BLE-Schreibvorgang = ein Befehl),
// Ein-Zeichen-Befehle gehen in die cmdQueue (ui_task.h -> ui_commands.h). Mehrere Zeichen = mehrere Befehle.
class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    if (v.size() >= 2 && v.data()[0] == 'Y') {   // "Y<mm>": Ersatzlänge ell von Hand setzen
      char buf[16] = {0};
      memcpy(buf, v.data() + 1, v.size() - 1 < 15 ? v.size() - 1 : 15);
      ellReq = atoi(buf) * 1e-3f;
      return;
    }
    if (v.size() >= 2 && v.data()[0] == 'Q') {   // "Q<qx>,<qv>,<qth>,<qom>,<r>" bzw. "Q0"
      char buf[64] = {0};
      memcpy(buf, v.data() + 1, v.size() - 1 < 63 ? v.size() - 1 : 63);
      pc::LQRWeights w{LQR_QX, LQR_QV, LQR_QTH, LQR_QOM, LQR_R};
      if (!(buf[0] == '0' && buf[1] == 0)) sscanf(buf, "%f,%f,%f,%f,%f", &w.qx, &w.qv, &w.qth, &w.qom, &w.r);
      lqrNew = w; lqrReq = true;
      return;
    }
    if (v.size() >= 2 && v.data()[0] == 'W') {   // "W<0,01 deg>": Winkel-Offset zur Kalibrierung (Schieberegler)
      char buf[16] = {0};
      memcpy(buf, v.data() + 1, v.size() - 1 < 15 ? v.size() - 1 : 15);
      angOfs = pc::clampf(atoi(buf) * 0.01f * DEG_TO_RAD, -ANG_OFS_MAX, ANG_OFS_MAX);
      return;
    }
    if (v.size() >= 2 && v.data()[0] == 'X') {   // "X<mm>": Soll-Mittelposition (Schieberegler)
      char buf[16] = {0};
      memcpy(buf, v.data() + 1, v.size() - 1 < 15 ? v.size() - 1 : 15);
      setXSet(atoi(buf) * 1e-3f);
      return;
    }
    // sonst: jedes Zeichen ist ein eigener Befehl (Leer- und Steuerzeichen ignorieren)
    for (size_t i = 0; i < v.size(); i++) {
      const char ch = (char)v.data()[i];
      if (ch > ' ') xQueueSend(cmdQueue, &ch, 0);
    }
  }
};

// App abonniert den Log-Kanal -> UI-Task schickt den gespeicherten Verlauf (log.h)
class LogCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t sub) override {
    if (sub) bleSendHistory = true;
  }
};

// Dienst anlegen und sichtbar machen. MTU 247 = große Pakete (Log/Telemetrie in einem Stück).
static void setupBle() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setMTU(247);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());
  NimBLEService* svc = bleServer->createService(BLE_SVC);
  NimBLECharacteristic* rx = svc->createCharacteristic(BLE_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCB());
  chLog = svc->createCharacteristic(BLE_LOG, NIMBLE_PROPERTY::NOTIFY);
  chLog->setCallbacks(new LogCB());
  chTel = svc->createCharacteristic(BLE_TEL, NIMBLE_PROPERTY::NOTIFY);
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_NAME);
  adv->addServiceUUID(BLE_SVC);
  adv->enableScanResponse(true);
  adv->start();
}
