#pragma once

namespace foxibot::math {

// Interpolation linéaire, t ∈ [0,1] (t est clampé)
template <typename T>
T lerp(const T & a, const T & b, double t);

// 3t² - 2t³ : départ et arrivée à vitesse nulle
double smoothstep(double t);

}  // namespace foxibot::math