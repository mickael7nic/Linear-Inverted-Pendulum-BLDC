// pendel_core.h – plattformunabhängiger Regelungskern für das inverse Pendel
// (kein Arduino-Code: wird identisch in der Host-Simulation getestet)
//
// Konventionen
//   x   Wagenposition [m], +x = Richtung, in die der Wagen bei positivem x fährt
//   th  Pendelwinkel [rad], 0 = aufrecht, +th = Spitze kippt Richtung +x, hängend = ±pi
//   om  dth/dt [rad/s]
//   a   Wagenbeschleunigung [m/s^2] = Stellgröße der äußeren Regelung
//   Modell: th_dd = (g*sin(th) - a*cos(th)) / ell - c*om
//           ell = J_pivot / (m * l_com) = Länge des äquivalenten mathematischen Pendels
//   IMU im Abstand rs vom Drehpunkt; e_r zeigt Drehpunkt->Spitze, e_t in Richtung +th
//     f_r = a*sin(th) + g*cos(th) - rs*om^2
//     f_t = a*cos(th) + rs*th_dd - g*sin(th)
#pragma once

// Aufteilung in Module:
#include "core_math.h"
#include "lqr.h"
#include "ekf.h"
#include "period_meter.h"
#include "controller.h"
