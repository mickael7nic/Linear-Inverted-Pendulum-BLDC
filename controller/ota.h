// ota.h – OTA-Modus: WLAN (Router per DHCP oder Notfall-AP), ArduinoOTA
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// 'O' bzw. Button "OTA-Modus": Motor aus, Pendel-Sender pausieren, ESP-NOW beenden, WLAN mit dem Router
// verbinden (secrets.h) oder einen eigenen Access Point aufmachen, dann Web-Upload (ota_web.h) und
// ArduinoOTA (Netzwerk-Port in der Arduino IDE) bereitstellen. Zurück nur per Neustart.
#pragma once

// OTA nur, wenn der Motor nicht gerade etwas Gefährliches tut
static bool otaAllowed() {
  using pc::Mode;
  const Mode m = ctrl.mode;
  if (m == Mode::SWINGUP || m == Mode::BALANCE || m == Mode::PERIOD || m == Mode::STOPPING || m == Mode::ARMED) {
    logf(">> OTA nur im Ruhezustand: erst stoppen (x)");
    return false;
  }
  return true;
}

// WLAN für OTA: Heim-WLAN (WIFI_SSID) oder eigener Access Point
// Kein WiFi.disconnect(true): das würde den WLAN-Stack komplett deinitialisieren (bei aktivem BLE riskant).
// Stattdessen nur den Modus umschalten, der Stack bleibt initialisiert.
static String startOtaWifi() {
  // WLAN komplett neu starten: der feste ESP-NOW-Kanal (13) darf die Router-Suche nicht beeinflussen
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(100));
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_country_code("DE", false);          // Kanäle 1-13, Router-Kanal per vollständigem Scan finden
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 150 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(100));
    if (WiFi.status() == WL_CONNECTED) {
      logf(">> WLAN verbunden, Router-Kanal %d", (int)WiFi.channel());
      return WiFi.localIP().toString() + " (Router '" WIFI_SSID "', DHCP)";
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    vTaskDelay(pdMS_TO_TICKS(100));
    logf(">> WLAN '%s' nicht erreichbar -> Notfall-AP '%s'", WIFI_SSID, OTA_HOSTNAME);
  } else {
    logf(">> WIFI_SSID in secrets.h leer -> Notfall-AP '%s'", OTA_HOSTNAME);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OTA_HOSTNAME, AP_PASS, OTA_AP_CHANNEL);
  return WiFi.softAPIP().toString() + " (AP '" OTA_HOSTNAME "', Passwort '" AP_PASS "')";
}

static void sendTelemetry();

// Aufgerufen vom UI-Task (Befehl 'O'). Danach übernimmt loop() (otaReady).
static void startCartOta() {
  if (otaActive || !otaAllowed()) return;
  homeAbort = true;
  otaActive = true;                          // ctrl -> IDLE, Motor aus
  logf(">> OTA Wagen: Motor aus, Pendel pausiert, ESP-NOW wird beendet ...");
  sendPendelCmd(CMD_PAUSE, 5);
  vTaskDelay(pdMS_TO_TICKS(50));
  if (espNowUp) {                            // ESP-NOW-Aufrufe ohne Init können abstürzen (Fehlerzustand)
    espNowUp = false;
    esp_now_unregister_recv_cb();
    esp_now_deinit();
  }
  otaStartMs = millis();
  const String ip = startOtaWifi();
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() { logf(">> OTA: Upload startet"); bleReleaseForOta(); });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int last = -1;
    const int pct = total ? (int)(100ULL * done / total) : 0;
    if (pct / 10 != last) { last = pct / 10; logf(">> OTA: %d %%", pct); }
  });
  ArduinoOTA.onEnd([]() { logf(">> OTA fertig - Neustart"); });
  ArduinoOTA.onError([](ota_error_t e) { logf(">> OTA Fehler %u", (unsigned)e); otaFailRestart(); });
  ArduinoOTA.begin();
  setupWebOta();
  otaStartMs = millis();
  logf(">> OTA bereit: %s - .bin in der Web-App hochladen oder Netzwerk-Port '%s' in der Arduino IDE", ip.c_str(),
       OTA_HOSTNAME);
  const IPAddress addr = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();
  logf("OTA-IP: %s", addr.toString().c_str());   // wird von index.html ausgewertet
  logf("   freier RAM %u kB, groesster Block %u kB", (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
  otaReady = true;
}
