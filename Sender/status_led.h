// status_led.h – WS2812-Statusanzeige des Pendel-Senders
// Benötigt PIN_RGB und RGB_BRIGHT aus PendelSender.ino (vor dem #include definiert).
#pragma once
#include <Arduino.h>

enum class St : uint8_t { BOOT, ERR_IMU_INIT, ERR_NOW_INIT, ERR_IMU, NO_CART, PAUSED, VBAT_LOW, OK };
struct Rgb { uint8_t r, g, b; };

static void rgbRaw(Rgb c) {   // nur bei Änderung schreiben (RMT-Zugriff)
  static Rgb last = {1, 2, 3};
  if (c.r == last.r && c.g == last.g && c.b == last.b) return;
  last = c;
  rgbLedWrite(PIN_RGB, (uint16_t)c.r * RGB_BRIGHT / 255, (uint16_t)c.g * RGB_BRIGHT / 255,
              (uint16_t)c.b * RGB_BRIGHT / 255);
}

static void showStatus(St st) {
  const uint32_t t = millis();
  const bool fast = (t % 200) < 100, slow = (t % 1000) < 500, flash = (t % 2000) < 60;
  static constexpr Rgb OFF{0, 0, 0};
  switch (st) {
    case St::BOOT:         rgbRaw({255, 255, 255}); break;
    case St::ERR_IMU_INIT: rgbRaw(fast ? Rgb{255, 0, 0} : OFF); break;
    case St::ERR_NOW_INIT: rgbRaw(fast ? Rgb{255, 0, 255} : OFF); break;
    case St::ERR_IMU:      rgbRaw(slow ? Rgb{255, 0, 0} : OFF); break;
    case St::NO_CART:      rgbRaw(slow ? Rgb{0, 0, 255} : OFF); break;
    case St::PAUSED:       rgbRaw({0, 200, 200}); break;
    case St::VBAT_LOW:     rgbRaw(slow ? Rgb{255, 100, 0} : OFF); break;
    case St::OK:           rgbRaw(flash ? Rgb{0, 255, 0} : OFF); break;
  }
}

