// espnow.h – ESP-NOW: Empfang der Pendel-IMU, Befehle an den Sender
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// ESP-NOW = direkter WLAN-Funk zwischen ESPs ohne Router. Der Pendel-Sender schickt 800 ImuPakete/s als
// Broadcast auf Kanal ESPNOW_CHANNEL (protocol.h, muss in beiden Programmen gleich sein).
// Der erste gültige Sender wird gemerkt ("gelockt"), Pakete anderer Sender werden verworfen.
// Rückkanal: Befehle an den Sender (CmdPacket: Pause/Weiter für OTA, ALIVE-Heartbeat 2 Hz mit "aktiv"-Bit,
// das den Sender am Einschlafen hindert), ebenfalls Broadcast.
#pragma once

// Empfangs-Callback (läuft im WLAN-Task auf Kern 0): nur prüfen und in die Queue, Auswertung im Regel-Task.
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(ImuPacket)) { rxBad = rxBad + 1; return; }
  ImuPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != PENDEL_MAGIC) { rxBad = rxBad + 1; return; }
  // erster Sender: MAC merken (UI-Task meldet sie im Log)
  if (!senderLocked) { memcpy(senderMac, info->src_addr, 6); senderLocked = true; evtSender = true; }
  else if (memcmp(senderMac, info->src_addr, 6) != 0) { rxBad = rxBad + 1; return; }
  lastRxMs = millis();
  rxCount = rxCount + 1;
  xQueueSend(rxQueue, &p, 0);
}

static volatile bool espNowUp = false;

// Befehl an den Pendel-Sender, repeat-mal im Abstand von 20 ms (Broadcast ohne Quittung -> Wiederholung
// statt Bestätigung). Blockiert repeat × 20 ms -> nur aus dem UI-Task aufrufen.
static void sendPendelCmd(uint8_t cmd, int repeat, uint8_t arg = 0) {
  if (!espNowUp) return;
  CmdPacket c = {CMD_MAGIC, cmd, arg};
  for (int i = 0; i < repeat; i++) {
    esp_now_send(BCAST, (const uint8_t*)&c, sizeof(c));
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// WLAN im Stationsmodus ohne Verbindung, fester Kanal, Broadcast-Peer anlegen, Empfang anmelden.
static bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);       // mit BLE nicht erlaubt (Fehler ignoriert), ohne AP-Verbindung wirkungslos
  esp_wifi_set_country_code("DE", true);             // Kanäle 1-13 (Standard "01" erlaubt nur 1-11)
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    logf("WARNUNG: ESP-NOW-Kanal %d nicht gesetzt\n", ESPNOW_CHANNEL);
  if (esp_now_init() != ESP_OK) return false;
  esp_wifi_get_mac(WIFI_IF_STA, ownMac);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  espNowUp = esp_now_register_recv_cb(onRecv) == ESP_OK;
  return espNowUp;
}
