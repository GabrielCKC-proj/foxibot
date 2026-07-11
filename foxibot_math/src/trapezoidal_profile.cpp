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

#include "foxibot_math/trapezoidal_profile.hpp"

namespace foxibot::math {
TrapezoidalProfile::TrapezoidalProfile(double start, double goal, ProfileConstraints constraints)
    : start_(start), goal_(goal), constraints_(constraints) {
  // TODO: d_total, signe, if trapèze/triangle, v_peak_, t1_, t2_, duration_
}

double TrapezoidalProfile::duration() const { return 0.0; }
double TrapezoidalProfile::positionAt(double t) const { return 0.0; }
double TrapezoidalProfile::velocityAt(double t) const { return 0.0; }
}  // namespace foxibot::math
