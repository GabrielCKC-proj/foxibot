#include "foxibot_kinematics/leg_kinematics.hpp"

#include <cmath>

#include "foxibot_kinematics/leg_model.hpp"
#include "foxibot_math/angles.hpp"

namespace foxibot::kinematics {
LegKinematics::LegKinematics(LegModel model) : model_(std::move(model)) {}

foxibot::math::Vec3 forward(const JointAngles & angles) {
  return foxibot::math::Vec3(0.0, 0.0, 0.0);
}

std::optional<JointAngles> inverse(const foxibot::math::Vec3 & target) { return std::nullopt; }

bool isWithinLimits(const JointAngles & angles) { return true; }
}  // namespace foxibot::kinematics