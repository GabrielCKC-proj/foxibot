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

#include "foxibot_kinematics/leg_kinematics.hpp"

#include <cmath>

#include "foxibot_kinematics/leg_model.hpp"
#include "foxibot_math/angles.hpp"
namespace foxibot::kinematics {
LegKinematics::LegKinematics(LegModel model) : model_(std::move(model)) {}

foxibot::math::Vec3 LegKinematics::forward(const JointAngles & angles) const {
  const double L0 = model_.hip_offset;
  const double L1 = model_.femur_length;
  const double L2 = model_.tibia_length;

  const double alpha = angles.hip_flexion + angles.knee;
  const double x = L1 * std::sin(angles.hip_flexion) + L2 * std::sin(alpha);
  const double y =
      L0 * std::cos(angles.hip_abduction) +
      (L1 * std::cos(angles.hip_flexion) + L2 * std::cos(alpha)) * std::sin(angles.hip_abduction);
  const double z =
      L0 * std::sin(angles.hip_abduction) -
      (L1 * std::cos(angles.hip_flexion) + L2 * std::cos(alpha)) * std::cos(angles.hip_abduction);
  return foxibot::math::Vec3(x, y, z);
}

std::optional<JointAngles> LegKinematics::inverse(const foxibot::math::Vec3 & target) const {
  const double x = target.x();
  const double y = target.y();
  const double z = target.z();
  const double L0 = model_.hip_offset;
  const double L1 = model_.femur_length;
  const double L2 = model_.tibia_length;
  JointAngles angles;

  const double radicand = y * y + z * z - L0 * L0;
  if (radicand < 0.0) {
    return std::nullopt;
  }
  const double A = std::sqrt(radicand);
  const double r = std::sqrt(x * x + A * A);
  angles.hip_abduction = std::atan2(L0 * z + A * y, y * L0 - A * z);

  double D = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (D < -1.0 || D > 1.0) {
    return std::nullopt;
  }
  D = foxibot::math::clampToLimits(D, -1.0, 1.0);

  angles.knee = -std::acos(D);
  const double U = L1 + L2 * std::cos(angles.knee);
  const double V = L2 * std::sin(angles.knee);
  angles.hip_flexion = std::atan2(x * U - V * A, A * U + V * x);

  return angles;
}

bool isWithinLimits(const JointAngles &) { return true; }
}  // namespace foxibot::kinematics
