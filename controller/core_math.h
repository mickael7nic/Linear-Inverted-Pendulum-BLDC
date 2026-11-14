// core_math.h – Konstanten und kleine Mathe-Helfer
// Teil des Regelungskerns (pendel_core.h), plattformunabhängig, auch in der Host-Simulation genutzt.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace pc {

constexpr float G = 9.80665f;
constexpr float PI_F = 3.14159265358979f;

// Winkel auf -pi..pi falten
inline float wrapPi(float a) {
  a = fmodf(a + PI_F, 2.0f * PI_F);
  if (a < 0) a += 2.0f * PI_F;
  return a - PI_F;
}
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace pc
