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

#include <optional>
#include <utility>

#include "foxibot_kinematics/leg_model.hpp"
#include "foxibot_math/types.hpp"

namespace foxibot::kinematics {

struct JointAngles {
  double hip_abduction;  // θ1 [rad]
  double hip_flexion;    // θ2 [rad]
  double knee;           // θ3 [rad]
};

class LegKinematics {
public:
  explicit LegKinematics(LegModel model);

  // Position du pied dans le repère hanche [m]
  foxibot::math::Vec3 forward(const JointAngles & angles) const;

  // Angles pour atteindre target ; std::nullopt si hors d'atteinte
  std::optional<JointAngles> inverse(const foxibot::math::Vec3 & target) const;

  // true si tous les angles sont dans les limites du modèle
  bool isWithinLimits(const JointAngles & angles) const;

private:
  LegModel model_;
};

}  // namespace foxibot::kinematics
