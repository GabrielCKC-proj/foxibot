#pragma once

#include <optional>

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