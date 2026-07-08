#pragma once
#include <cstdint>

namespace foxibot {

enum class LegId : std::uint8_t { kFrontLeft, kFrontRight, kRearLeft, kRearRight };

// Articulations d'une jambe 3 DOF
enum class JointId : std::uint8_t { kHipAbduction, kHipFlexion, kKnee };

inline constexpr std::size_t kJointsPerLeg = 3;

}  // namespace foxibot
