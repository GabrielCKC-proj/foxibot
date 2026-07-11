// Copyright 2026 Gabriel Cekici
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once
#include <cmath>
namespace foxibot::math {

constexpr double kPi = 3.14159265358979323846;

constexpr double degToRad(double deg) { return deg * kPi / 180; }
constexpr double radToDeg(double rad) { return rad * 180 / kPi; }

// Ramène un angle dans (-pi, pi]
double wrapToPi(double angle) {
  double r = fmod(angle, 2 * kPi);
  if (r > kPi) {
    r = r - 2.0 * kPi;
  } else if (r <= -kPi) {
    r = r + 2.0 * kPi;
  }

  return r;
}

// Vrai si angle ∈ [min, max]
bool isWithinLimits(double angle, double min, double max) { return angle >= min && angle <= max; }

// Borne angle à [min, max]
double clampToLimits(double angle, double min, double max) {
  if (angle < min) {
    return min;
  } else if (angle > max) {
    return max;
  } else {
    return angle;
  }
}

}  // namespace foxibot::math
