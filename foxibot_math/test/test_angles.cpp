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

#include "foxibot_math/angles.hpp"

namespace fm = foxibot::math;

// 1 - test degToRad function
TEST(Angles, DegToRadConvertsHalfTurn) { EXPECT_DOUBLE_EQ(fm::degToRad(180.0), fm::kPi); }
TEST(Angles, RadToDegConvertHalfTurn) { EXPECT_DOUBLE_EQ(fm::radToDeg(fm::kPi), 180.0); }
// 2 - test wrapToPi function
TEST(Angles, WrapToPiHandlesThreePi) { EXPECT_NEAR(fm::wrapToPi(3.0 * fm::kPi), fm::kPi, 1e-12); }
TEST(Angles, WrapToPiHandlesNegativePi) { EXPECT_NEAR(fm::wrapToPi(-fm::kPi), fm::kPi, 1e-12); }
TEST(Angles, WrapToPiHandlesZeros) { EXPECT_DOUBLE_EQ(fm::wrapToPi(0.0), 0.0); }
TEST(Angles, WrapToPiHandlesNegativeThreePi) {
  EXPECT_NEAR(fm::wrapToPi(-3.0 * fm::kPi), fm::kPi, 1e-12);
}
TEST(Angles, WrapToPiHandlesBigAngles) {
  EXPECT_NEAR(fm::wrapToPi(100.0 * fm::kPi + 0.5), 0.5, 1e-12);
}
// 3 - Test isWithinLimits function
TEST(Angles, IsWithInLimitsHalfPi) { EXPECT_TRUE(fm::isWithinLimits(fm::kPi / 2, 0.0, fm::kPi)); }
TEST(Angles, IsWithInLimitsRejectBelowLimits) {
  EXPECT_FALSE(fm::isWithinLimits(0.0, fm::kPi, 3.0 * fm::kPi));
}
TEST(Angles, IsWithInLimitsRejectAboveLimits) {
  EXPECT_FALSE(fm::isWithinLimits(5.0 * fm::kPi, fm::kPi, 3.0 * fm::kPi));
}
TEST(Angles, IsWithInLimitsUpperBorn) { EXPECT_TRUE(fm::isWithinLimits(fm::kPi, 0.0, fm::kPi)); }
TEST(Angles, IsWithInLimitsLowerBorn) { EXPECT_TRUE(fm::isWithinLimits(0.0, 0.0, fm::kPi)); }
// 4 - Test clampToLimits function
TEST(Angles, ClampToLimitsLower) {
  EXPECT_DOUBLE_EQ(fm::clampToLimits(-fm::kPi, 0.0, fm::kPi), 0.0);
}
TEST(Angles, ClampToLimitsUpper) {
  EXPECT_DOUBLE_EQ(fm::clampToLimits(2.0 * fm::kPi, 0.0, fm::kPi), fm::kPi);
}
TEST(Angles, ClampToLimitsMiddle) {
  EXPECT_DOUBLE_EQ(fm::clampToLimits(fm::kPi / 2, 0.0, fm::kPi), fm::kPi / 2);
}
