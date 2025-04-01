// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/EqualsUtil.h"

namespace util {

bool EqualsUtil::EpsilonEquals(double a, double b, double epsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}

bool EqualsUtil::EpsilonEquals(double a, double b) {
  return EpsilonEquals(a, b, 1e-9);
}

bool EqualsUtil::GeomExtensions::EpsilonEquals(
    const frc::geometry::Twist2d &twist, const frc::geometry::Twist2d &other) {
  return EqualsUtil::EpsilonEquals(twist.dx, other.dx) &&
         EqualsUtil::EpsilonEquals(twist.dy, other.dy) &&
         EqualsUtil::EpsilonEquals(twist.dtheta, other.dtheta);
}

} // namespace util