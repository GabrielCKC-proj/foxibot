#include <gtest/gtest.h>

#include "foxibot_kinematics/leg_kinematics.hpp"
#include "foxibot_kinematics/leg_model.hpp"
#include "foxibot_math/angles.hpp"

namespace fk = foxibot::kinematics;
namespace fm = foxibot::math;

constexpr double kTolerance = 1e-9;

fk::LegModel makeInverseTestModel() {
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

void expectAnglesNear(const fk::JointAngles & actual, double expected_th1, double expected_th2,
                      double expected_th3) {
  EXPECT_NEAR(actual.hip_abduction, expected_th1, kTolerance);
  EXPECT_NEAR(actual.hip_flexion, expected_th2, kTolerance);
  EXPECT_NEAR(actual.knee, expected_th3, kTolerance);
}

TEST(InverseKinematics, ZeroPoseFootBelowHip) {
  const fk::LegModel model = makeInverseTestModel();
  const fk::LegKinematics kinematics(model);

  const fm::Vec3 position_target{0.0, model.hip_offset, -(model.femur_length + model.tibia_length)};
  const auto foot = kinematics.inverse(position_target);

  expectAnglesNear(*foot, 0.0, 0.0, 0.0);
}

TEST(InverseKinematics, HalfAngleFootPose) {
  const fk::LegModel model = makeInverseTestModel();
  const fk::LegKinematics kinematics(model);

  const fm::Vec3 position_target{(model.femur_length - model.tibia_length) / sqrt(2.0),
                                 model.hip_offset,
                                 -(model.tibia_length + model.femur_length) / sqrt(2.0)};
  const auto foot = kinematics.inverse(position_target);

  expectAnglesNear(*foot, 0.0, fm::kPi / 4, -fm::kPi / 2);
}

TEST(InverseKinematics, AnyFootPose) {
  const fk::LegModel model = makeInverseTestModel();
  const fk::LegKinematics kinematics(model);

  const fm::Vec3 position_target{-model.tibia_length,
                                 (model.hip_offset + model.femur_length) / sqrt(2),
                                 (model.hip_offset - model.femur_length) / sqrt(2.0)};
  const auto foot = kinematics.inverse(position_target);

  expectAnglesNear(*foot, fm::kPi / 4, 0.0, -fm::kPi / 2);
}
