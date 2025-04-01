// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <optional>
#include <vector>

#include "Constants.h"
#include "FieldConstants.h"
#include "thirdparty/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"

namespace util {

class AllianceFlipUtil {
public:
  static double ApplyX(double x);
  static double ApplyY(double y);
  static frc::geometry::Translation2d
  Apply(frc::geometry::Translation2d translation);
  static frc::geometry::Rotation2d Apply(frc::geometry::Rotation2d rotation);
  static frc::geometry::Pose2d Apply(frc::geometry::Pose2d pose);
  static frc::geometry::Translation3d
  Apply(frc::geometry::Translation3d translation);
  static frc::geometry::Rotation3d Apply(frc::geometry::Rotation3d rotation);
  static frc::geometry::Pose3d Apply(frc::geometry::Pose3d pose);
  static VehicleState Apply(const VehicleState &state);
  static bool ShouldFlip();
};

} // namespace util