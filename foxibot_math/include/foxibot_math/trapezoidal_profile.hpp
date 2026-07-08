#pragma once

namespace foxibot::math {

struct ProfileConstraints {
  double max_velocity;      // > 0
  double max_acceleration;  // > 0
};

class TrapezoidalProfile {
public:
  // Calcule le profil pour aller de start à goal (positions scalaires).
  TrapezoidalProfile(double start, double goal, ProfileConstraints constraints);

  double duration() const;            // durée totale [s]
  double positionAt(double t) const;  // t clampé à [0, duration]
  double velocityAt(double t) const;

private:
  // phases : accélération / plateau / décélération
};

}  // namespace foxibot::math