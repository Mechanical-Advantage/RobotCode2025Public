// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/geometry/Twist2d.h>

namespace util {

class EqualsUtil {
public:
  static bool EpsilonEquals(double a, double b, double epsilon);
  static bool EpsilonEquals(double a, double b);

  /** Extension methods for wpi geometry objects */
  class GeomExtensions {
  public:
    static bool EpsilonEquals(const frc::geometry::Twist2d &twist,
                              const frc::geometry::Twist2d &other);
  };
};

} // namespace util