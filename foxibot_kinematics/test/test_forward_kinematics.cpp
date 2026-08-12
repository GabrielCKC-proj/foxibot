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

#include <gtest/gtest.h>

#include "foxibot_kinematics/leg_kinematics.hpp"
#include "foxibot_kinematics/leg_model.hpp"
#include "foxibot_math/angles.hpp"

namespace fk = foxibot::kinematics;
namespace fm = foxibot::math;

constexpr double kTolerance = 1e-7;

fk::LegModel makeForwardTestModel() {
  fk::LegModel model{};
  model.femur_length = 0.1;
  model.tibia_length = 0.12;
  model.hip_offset = 0.025;
  // Les valeurs limites seront déterminés lors de la CAO : ici, arbitraires
  model.limits[0] = fk::JointLimits{-fm::kPi / 2.0, fm::kPi / 2.0};
  model.limits[1] = fk::JointLimits{-fm::kPi, fm::kPi};
  model.limits[2] = fk::JointLimits{-fm::kPi, fm::kPi};
  return model;
}

void expectPositionNear(const foxibot::math::Vec3 & actual, double expected_x, double expected_y,
                        double expected_z) {
  EXPECT_NEAR(actual.x(), expected_x, kTolerance);
  EXPECT_NEAR(actual.y(), expected_y, kTolerance);
  EXPECT_NEAR(actual.z(), expected_z, kTolerance);
}

// Pose A (zero) : th1 = th2 = th3 = 0
// Jambe tendue vers le bas, plan de jambe confondu avec XZ.
// x = L1*sin(0) + L2*sin(0)            = 0
// y = L0*cos(0) + (L1*cos(0)+L2*cos(0))*sin(0) = L0
// z = L0*sin(0) - (L1*cos(0)+L2*cos(0))*cos(0) = -(L1+L2)
TEST(ForwardKinematics, ZeroPoseFootBelowHip) {
  const fk::LegModel model = makeForwardTestModel();
  const fk::LegKinematics kinematics(model);

  const fk::JointAngles angles{0.0, 0.0, 0.0};
  const fm::Vec3 foot = kinematics.forward(angles);

  expectPositionNear(foot, 0.0, model.hip_offset, -(model.femur_length + model.tibia_length));
}

TEST(ForwardKinematics, HalfAngleFootPose) {
  const fk::LegModel model = makeForwardTestModel();
  const fk::LegKinematics kinematics(model);

  const fk::JointAngles angles(0.0, fm::kPi / 4, -fm::kPi / 2);
  const fm::Vec3 foot = kinematics.forward(angles);

  expectPositionNear(foot, (model.femur_length - model.tibia_length) / sqrt(2.0), model.hip_offset,
                     -(model.tibia_length + model.femur_length) / sqrt(2.0));
}

TEST(ForwardKinematics, AnyFootPose) {
  const fk::LegModel model = makeForwardTestModel();
  const fk::LegKinematics kinematics(model);

  const fk::JointAngles angles(fm::kPi / 4, 0.0, -fm::kPi / 2);
  const fm::Vec3 foot = kinematics.forward(angles);

  expectPositionNear(foot, -model.tibia_length, (model.hip_offset + model.femur_length) / sqrt(2.0),
                     (model.hip_offset - model.femur_length) / sqrt(2.0));
}
