// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <set>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/commands/AutoScoreCommands.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/TrajectoryGenerationHelpers.h"
#include "org/littletonrobotics/frc2025/util/GeomUtil.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"
#include "units/length.h"

class DriveTrajectories {
public:
  static std::map<std::string, std::vector<PathSegment>> paths;
  static std::vector<std::function<std::map<
      std::string, std::vector<PathSegment>>(const std::set<std::string> &)>>
      suppliedPaths;

  static constexpr units::meter_t upInTheWaterLineupDistance = 0.3_m;
  static std::array<frc::Pose2d, 4> upInTheWaterScoringPoses;

  static frc::Waypoint GetLastWaypoint(const std::string &trajectoryName);

private:
  static frc::Pose2d GetNearestIntakingPose(const frc::Pose2d &pose);

  static void InitializePaths();
};