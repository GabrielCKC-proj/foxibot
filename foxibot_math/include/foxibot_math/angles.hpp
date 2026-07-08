#pragma once

namespace foxibot::math {

constexpr double kPi = 3.14159265358979323846;

constexpr double degToRad(double deg);
constexpr double radToDeg(double rad);

// Ramène un angle dans (-pi, pi]
double wrapToPi(double angle);

// Vrai si angle ∈ [min, max]
bool isWithinLimits(double angle, double min, double max);

// Borne angle à [min, max]
double clampToLimits(double angle, double min, double max);

}  // namespace foxibot::math