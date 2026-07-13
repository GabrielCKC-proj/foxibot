#pragma once
#include "foxibot_common/types.hpp"

namespace foxibot::kinematics {

struct JointLimits {
  double min;  // [rad]
  double max;  // [rad]
};

struct LegModel {
  double femur_length;                         // L1 [m]
  double tibia_length;                         // L2 [m]
  double hip_offset;                           // L0, offset latéral J1→J2 [m] (0.0 si aligné)
  JointLimits limits[foxibot::kJointsPerLeg];  // ta constante de foxibot_common !
};

}  // namespace foxibot::kinematics