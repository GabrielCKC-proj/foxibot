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

#include "foxibot_math/interpolation.hpp"
#include "foxibot_math/types.hpp"

namespace fm = foxibot::math;

// 1 - Test lerp function
// 1.1 - Test lerp as double
TEST(Lerp, DoubleReturnsStartAtZero) { EXPECT_DOUBLE_EQ(fm::lerp(2.0, 5.0, 0.0), 2.0); }
TEST(Lerp, DoubleReturnsEndAtOne) { EXPECT_DOUBLE_EQ(fm::lerp(2.0, 5.0, 1.0), 5.0); }
TEST(Lerp, DoubleReturnsClampingAboveOne) { EXPECT_DOUBLE_EQ(fm::lerp(2.0, 5.0, 2.0), 5.0); }
TEST(Lerp, DoubleReturnsClampingBelowZero) { EXPECT_DOUBLE_EQ(fm::lerp(2.0, 5.0, -1.0), 2.0); }
// 1.2 - Test lerp as vec3
TEST(Lerp, Vec3InterpolatesEachComponent) {
  fm::Vec3 a(0.0, 10.0, -2.0);
  fm::Vec3 b(4.0, 20.0, 2.0);
  fm::Vec3 mid = fm::lerp(a, b, 0.5);
  EXPECT_DOUBLE_EQ(mid.x(), 2.0);
  EXPECT_DOUBLE_EQ(mid.y(), 15.0);
  EXPECT_DOUBLE_EQ(mid.z(), 0.0);
}
TEST(Lerp, Vec3ClampsAboveOne) {
  fm::Vec3 a(0.0, 10.0, -2.0);
  fm::Vec3 b(4.0, 20.0, 2.0);
  EXPECT_TRUE(fm::lerp(a, b, 3.5).isApprox(b, 1e-12));
}
TEST(Lerp, Vec3ClampsBelowZero) {
  fm::Vec3 a(0.0, 10.0, -2.0);
  fm::Vec3 b(4.0, 20.0, 2.0);
  EXPECT_TRUE(fm::lerp(a, b, -0.5).isApprox(a, 1e-12));
}

// 2 - Test smoothstep function
TEST(Lerp, ReturnHalfAtHalf) { EXPECT_DOUBLE_EQ(fm::smoothstep(0.5), 0.5); }
TEST(Lerp, IsSymetricAroundHalf) {
  EXPECT_DOUBLE_EQ(fm::smoothstep(0.75) + fm::smoothstep(0.25), 1);
}
TEST(Lerp, ReturnLowerLimits) { EXPECT_DOUBLE_EQ(fm::smoothstep(0.0), 0.0); }
TEST(Lerp, ReturnUpperLimits) { EXPECT_DOUBLE_EQ(fm::smoothstep(1.0), 1.0); }
