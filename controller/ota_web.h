// ota_web.h – Web-OTA: HTTP-Upload /update, CORS
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
//
// Ablauf eines Updates aus der Web-App: App fragt im OTA-Modus /ping ab, schickt die .bin per POST an /update.
// Der Upload-Handler schreibt die Blöcke direkt in die zweite App-Partition (Update.*); am Ende startet der
// Wagen neu und bootet die neue Firmware. Schlägt etwas fehl, bleibt die alte Firmware aktiv.
// CORS-Header: index.html wird von einer anderen Adresse (Datei oder https-Seite) geladen und darf den Wagen
// nur ansprechen, wenn der Wagen das ausdrücklich erlaubt.
// Läuft komplett in loop() auf Kern 1 (otaService), siehe Pendel_bldc.ino.
#pragma once

static uint32_t otaStartMs = 0;   // letzte Aktivität; nach OTA_TIMEOUT_MS ohne Upload -> Neustart

// ---------------- Web-OTA: POST /update (multipart, Feld beliebig), CORS für index.html ----------------
static WebServer web(80);   // HTTP-Server auf Port 80
static bool webOtaUp = false;
static volatile bool otaReady = false;   // erst nach WLAN + Server-Start darf loop() (Kern 1) otaService() aufrufen
static bool otaWritten = false;          // Update.end() erfolgreich
// Bluetooth beim Upload-Start abschalten: WLAN + BLE + Webserver lassen sonst nur ~13 kB RAM frei (zu knapp).
// Das Abschalten macht der UI-Task (dort laufen alle BLE-Zugriffe); hier nur anfordern und warten.
static volatile bool bleRelReq = false, bleReleased = false;
static void bleReleaseForOta() {
  if (bleReleased) return;
  const unsigned h0 = ESP.getFreeHeap() / 1024;
  bleRelReq = true;
  for (int i = 0; i < 150 && !bleReleased; i++) delay(10);
  logf(">> OTA: Bluetooth beendet (freier RAM %u -> %u kB, groesster Block %u kB). Nach dem Update startet der Wagen neu",
       h0, (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
}
static void otaFailRestart() {          // ohne Bluetooth geht nichts mehr -> Neustart
  if (!bleReleased) return;
  logf(">> OTA fehlgeschlagen - Neustart, damit Bluetooth wieder laeuft");
  delay(500); ESP.restart();
}

// Antwort-Header, die dem Browser den Zugriff von index.html aus erlauben
static void cors() {
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  web.sendHeader("Access-Control-Allow-Headers", "*");
  web.sendHeader("Access-Control-Allow-Private-Network", "true");   // Chrome: file:// -> private IP
}

// URLs: /ping (App prüft Erreichbarkeit), /update (Firmware), /reboot, / (einfache Upload-Seite ohne App)
static void setupWebOta() {
  web.on("/update", HTTP_OPTIONS, []() { cors(); web.send(204); });
  web.on("/reboot", HTTP_OPTIONS, []() { cors(); web.send(204); });
  web.on("/ping", HTTP_GET, []() { cors(); web.send(200, "text/plain", "pendel-wagen"); });
  web.onNotFound([]() { logf(">> HTTP %s unbekannt", web.uri().c_str()); cors(); web.send(404, "text/plain", "?"); });
  web.on("/reboot", HTTP_GET, []() {
    cors(); web.send(200, "text/plain", "OK"); logf(">> Neustart (Web)"); delay(200); ESP.restart();
  });
  web.on("/", HTTP_GET, []() {
    logf(">> OTA-Seite abgerufen von %s", web.client().remoteIP().toString().c_str());
    web.send(200, "text/html",
             "<!doctype html><meta charset=utf-8><title>Pendel-Wagen OTA</title><h3>Pendel-Wagen Firmware-Update</h3>"
             "<form method=POST action=/update enctype=multipart/form-data><input type=file name=firmware accept=.bin>"
             " <input type=submit value=Hochladen></form>");
  });
  web.on("/update", HTTP_POST,
    []() {   // nach dem Upload
      cors();
      const bool ok = otaWritten && !Update.hasError();
      web.send(200, "text/plain", ok ? "OK" : String("FEHLER ") + Update.errorString());
      if (ok) { logf(">> OTA fertig - Neustart"); delay(300); ESP.restart(); }
      otaFailRestart();
    },
    []() {   // Datenblöcke
      HTTPUpload& u = web.upload();
      static int lastStep = -1;
      if (u.status == UPLOAD_FILE_START) {
        logf(">> OTA: Upload '%s' startet (Platz fuer Firmware %u kB, freier RAM %u kB, groesster Block %u kB)",
             u.filename.c_str(), (unsigned)(ESP.getFreeSketchSpace() / 1024), (unsigned)(ESP.getFreeHeap() / 1024),
             (unsigned)(ESP.getMaxAllocHeap() / 1024));
        lastStep = -1; otaWritten = false;
        if (ctrlTaskH) vTaskSuspend(ctrlTaskH);            // Regeltask anhalten, solange der Flash beschrieben wird
        bleReleaseForOta();
        if (Update.isRunning()) { logf(">> OTA: alter Upload haengt noch - verworfen"); Update.abort(); }
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          logf(">> OTA Fehler beim Start: %s", Update.hasError() ? Update.errorString() : "kein RAM fuer 4-kB-Puffer");
      } else if (u.status == UPLOAD_FILE_WRITE) {
        otaStartMs = millis();
        if (!Update.isRunning()) return;                    // Start fehlgeschlagen -> Rest ignorieren (Meldung oben)
        if (Update.write(u.buf, u.currentSize) != u.currentSize) logf(">> OTA Fehler: %s", Update.errorString());
        const int step = (int)(u.totalSize / 102400);   // alle 100 kB ins Log
        if (step != lastStep) { lastStep = step; logf(">> OTA: %u kB", (unsigned)(u.totalSize / 1024)); }
      } else if (u.status == UPLOAD_FILE_END) {
        if (!Update.isRunning()) logf(">> OTA: nichts geschrieben");
        else if (Update.end(true)) { otaWritten = true; logf(">> OTA: %u Bytes geschrieben", (unsigned)u.totalSize); }
        else logf(">> OTA Fehler: %s", Update.errorString());
      } else if (u.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        if (ctrlTaskH) vTaskResume(ctrlTaskH);
        logf(">> OTA abgebrochen");
        otaFailRestart();
      }
    });
  web.begin();
  webOtaUp = true;
}

// in loop() aufgerufen, solange OTA aktiv ist
static void otaService() {
  if (!otaReady) return;
  ArduinoOTA.handle();
  if (webOtaUp) web.handleClient();
  if (millis() - otaStartMs > OTA_TIMEOUT_MS) { logf(">> OTA Timeout - Neustart"); delay(100); ESP.restart(); }
}
