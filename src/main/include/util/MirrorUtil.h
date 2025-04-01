// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include "FieldConstants.h"
#include "thirdparty/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

namespace util {

class MirrorUtil {
public:
  static void SetMirror(std::function<bool()> mirrorSupplier);
  static FieldConstants::CoralObjective
  Apply(FieldConstants::CoralObjective coralObjective);
  static frc::geometry::Pose2d Apply(frc::geometry::Pose2d pose);
  static VehicleState Apply(const VehicleState &state);

private:
  static std::function<bool()> mirror;
};

} // namespace util