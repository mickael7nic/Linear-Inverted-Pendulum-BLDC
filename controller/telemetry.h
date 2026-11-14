// telemetry.h – Telemetrie an die Web-App (BLE)
// Teil von Pendel_bldc.ino (wird nur dort eingebunden, in fester Reihenfolge).
#pragma once

static void sendTelemetry() {
  if (!bleConnected || !chTel) return;
  if (telQuiet && ctrl.motorEnabled()) return;
  {   // bei laufendem Motor nur 10 Hz: weniger BLE-Funkzeit -> weniger ESP-NOW-Lücken (Funktest: 6 -> 2 Lücken/3 min)
    static uint32_t lastT = 0;
    if (ctrl.motorEnabled() && millis() - lastT < 100) return;
    lastT = millis();
  }
  Telemetry t;
  t.t_ms = millis();
  t.x = (int16_t)pc::clampf(tx_x * 1e4f, -32767, 32767);
  t.v = (int16_t)pc::clampf(tx_v * 1e3f, -32767, 32767);
  t.th = (int16_t)(pc::wrapPi(ctrl.th_c) * 5729.58f);
  t.om = (int16_t)pc::clampf(ctrl.om_c * 1e3f, -32767, 32767);
  t.a = (int16_t)pc::clampf(ctrl.a_cmd * 100.0f, -32767, 32767);
  t.iq = (int16_t)pc::clampf(X_SIGN * motor.current.q * 100.0f, -32767, 32767);
  t.mode = (uint8_t)ctrl.mode | (ctrl.resting && ctrl.mode == pc::Mode::BALANCE ? 0x80 : 0);   // Bit 7: Ruhezone (senkrecht, Motor stromlos)
  uint8_t f = 0;
  if (homed) f |= TF_HOMED;
  if (ctrl.isCalibrated()) f |= TF_EST_OK;
  if (calStored) f |= TF_CAL_STORED;
  if (senderLocked) f |= TF_PENDEL_LINK;
  if (homing.active()) f |= TF_HOMING;
  if (calib.active) f |= TF_CALIBRATING;
  if (otaActive) f |= TF_OTA;
  if (tx_flags & PKT_FLAG_VBAT_LOW) f |= TF_VBAT_LOW;
  t.flags = f;
  t.vbat = tx_vbat;
  t.xs = (int16_t)pc::clampf(ctrl.xs * 1e4f, -32767, 32767);
  t.trim = (int16_t)pc::clampf(angOfs * 5729.58f, -32767, 32767);
  chTel->notify((const uint8_t*)&t, sizeof(t));
}
