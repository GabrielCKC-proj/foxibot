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

TEST(Roundtrip, ForwardInverseForwardReturnSamePosition) {
  const fk::LegModel model = makeInverseTestModel();
  const fk::LegKinematics kinematics(model);

  const double step = fm::degToRad(0.5);

  for (double th1 = -fm::kPi / 3; th1 <= fm::kPi / 3; th1 += step) {
    for (double th2 = -fm::kPi / 3; th2 <= fm::kPi / 3; th2 += step) {
      for (double th3 = -2.5; th3 <= -0.2; th3 += step) {
        const fk::JointAngles angles{th1, th2, th3};
        const fm::Vec3 p = kinematics.forward(angles);

        const auto recovered = kinematics.inverse(p);
        ASSERT_TRUE(recovered.has_value())
            << "IK a echoue pour th1=" << th1 << " th2=" << th2 << " th3=" << th3;

        const fm::Vec3 p2 = kinematics.forward(*recovered);
        EXPECT_NEAR(p2.x(), p.x(), 1e-6);
        EXPECT_NEAR(p2.y(), p.y(), 1e-6);
        EXPECT_NEAR(p2.z(), p.z(), 1e-6);
      }
    }
  }
}