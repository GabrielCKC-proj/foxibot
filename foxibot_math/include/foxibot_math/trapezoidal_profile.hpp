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
  // constructeur
  double start_;
  double goal_;
  ProfileConstraints constraints_;
  // sens du mouvement
  double sign_;
  // profile vitesse
  double t1_;
  double t2_;
  double t_plat_;
  double duration_;
  double d_m_;
  double d_plat_;
  double d_tot_;

  // vmax et amax
  double v_max_;
  double a_max_;

  // cas triangulaire
  double v_peak_;
};

}  // namespace foxibot::math
