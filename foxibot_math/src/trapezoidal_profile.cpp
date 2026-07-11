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

#include "foxibot_math/trapezoidal_profile.hpp"

#include <cmath>

#include "foxibot_math/angles.hpp"

namespace foxibot::math {
TrapezoidalProfile::TrapezoidalProfile(double start, double goal, ProfileConstraints constraints)
    : start_(start), goal_(goal), constraints_(constraints) {
  v_max_ = constraints.max_velocity;
  a_max_ = constraints.max_acceleration;
  double delta = goal_ - start_;
  d_tot_ = std::abs(delta);
  sign_ = (delta >= 0) ? 1.0 : -1.0;
  double seuil = (v_max_ * v_max_) / a_max_;
  if (d_tot_ <= 2 * seuil) {
    v_peak_ = std::sqrt(d_tot_ * a_max_);
    t_plat_ = 0.0;

  } else {
    v_peak_ = v_max_;
    t_plat_ = (d_tot_ - (v_peak_ * v_peak_) / a_max_) / v_max_;
  }
  t1_ = v_peak_ / a_max_;
  d_m_ = (v_peak_ * v_peak_) / (2.0 * a_max_);
  t2_ = t1_ + t_plat_;
  duration_ = t1_ + t2_;  // Par symétrie le t_montée = t_descente, or t_montée = t1_
  d_plat_ = t_plat_ * v_peak_;
  // TODO: d_total, signe, if trapèze/triangle, v_peak_, t1_, t2_, duration_
}

double TrapezoidalProfile::duration() const { return duration_; }
double TrapezoidalProfile::positionAt(double t) const {
  t = clampToLimits(t, 0.0, duration_);  // ton propre outil d'angles.hpp !
  double d;                              // distance parcourue, toujours positive
  if (t < t1_) {                         // montée
    d = (a_max_ * t * t) / 2;
  } else if (t < t2_) {  // plateau (vide en cas triangle, et c'est OK)
    d = d_m_ + v_peak_ * (t - t1_);
  } else {  // descente : temps de phase τ = t − t2_
    d = v_peak_ * (t - t2_) - (a_max_ * (t - t2_) * (t - t2_)) / 2 + d_m_ + d_plat_;
  }
  return start_ + sign_ * d;
}

double TrapezoidalProfile::velocityAt(double t) const { return 0.0; }
}  // namespace foxibot::math
