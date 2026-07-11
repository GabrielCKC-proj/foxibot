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

#include "foxibot_math/trapezoidal_profile.hpp"

namespace fm = foxibot::math;

TEST(TrapezoidalProfile, StartsAtStartPosition) {
  const double start = 2.0;
  const double goal = 12.0;
  fm::TrapezoidalProfile profile(start, goal, {2.0, 1.0});

  EXPECT_DOUBLE_EQ(profile.positionAt(0.0), start);
}

TEST(TrapezoidalProfile, StartsAtGoalPosition) {
  const double start = 2.0;
  const double goal = 12.0;
  fm::TrapezoidalProfile profile(start, goal, {2.0, 1.0});

  EXPECT_DOUBLE_EQ(profile.positionAt(profile.duration()), goal);
}

TEST(TrapezoidalProfile, VelocityAtStartPosition) {
  fm::TrapezoidalProfile profile(0.0, 10.0, {2.0, 1.0});
  EXPECT_DOUBLE_EQ(profile.velocityAt(0.0), 0.0);
}
TEST(TrapezoidalProfile, VelocityAtGoalPosition) {
  fm::TrapezoidalProfile profile(0.0, 10.0, {2.0, 1.0});
  EXPECT_DOUBLE_EQ(profile.velocityAt(profile.duration()), 0.0);
}
TEST(TrapezoidalProfile, VelocityVelocityNeverExceedsMax) {
  fm::TrapezoidalProfile profile(0.0, 10.0, {2.0, 1.0});
  for (double t = 0.0; t <= profile.duration(); t += 0.01) {
    EXPECT_LE(std::abs(profile.velocityAt(t)), 2.0 + 1e-9);
  }
}
TEST(TrapezoidalProfile, TriangularCaseWhenDistanceTooShort) {
  // d_total = 1 < v_max^2/a_max = 4  => profil triangulaire
  // v_peak = sqrt(a*d) = 1 m/s, t1 = 1 s, duree = 2 s, sommet a 0.5 m
  fm::TrapezoidalProfile profile(0.0, 1.0, {2.0, 1.0});

  EXPECT_NEAR(profile.duration(), 2.0, 1e-9);
  EXPECT_NEAR(profile.positionAt(1.0), 0.5, 1e-9);
  EXPECT_NEAR(profile.velocityAt(1.0), 1.0, 1e-9);  // v_peak < v_max !
  EXPECT_NEAR(profile.positionAt(2.0), 1.0, 1e-9);
  EXPECT_NEAR(profile.velocityAt(2.0), 0.0, 1e-9);
}
TEST(TrapezoidalProfile, TriangularCaseWhenGoalLowerThanStart) {
  // d_total = 1 < v_max^2/a_max = 4  => profil triangulaire
  // v_peak = sqrt(a*d) = 1 m/s, t1 = 1 s, duree = 2 s, sommet a 0.5 m
  fm::TrapezoidalProfile profile(1.0, 0.0, {2.0, 1.0});

  EXPECT_NEAR(profile.duration(), 2.0, 1e-9);
  EXPECT_NEAR(profile.positionAt(1.0), 0.5, 1e-9);
  EXPECT_NEAR(profile.velocityAt(1.0), -1.0, 1e-9);  // v_peak < v_max !
  EXPECT_NEAR(profile.positionAt(2.0), 0.0, 1e-9);
  EXPECT_NEAR(profile.velocityAt(2.0), 0.0, 1e-9);
}
TEST(TrapezoidalProfile, TriangularCaseWhenStartEqualGoal) {
  fm::TrapezoidalProfile profile(0.0, 0.0, {2.0, 1.0});
  EXPECT_DOUBLE_EQ(profile.duration(), 0.0);
}
TEST(TrapezoidalProfile, PositionIsMonotonic) {
  fm::TrapezoidalProfile profile(0.0, 10.0, {2.0, 1.0});
  double previous = profile.positionAt(0.0);
  for (double t = 0.01; t <= profile.duration(); t += 0.01) {
    double current = profile.positionAt(t);
    EXPECT_GE(current, previous - 1e-9);
    previous = current;
  }
}
