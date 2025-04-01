// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/TrajectoryGenerationHelpers.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"
#include <string>
#include <vector>

class HolonomicTrajectory {
private:
  Trajectory trajectory;

public:
  HolonomicTrajectory(const std::string &name);

  double GetDuration() const;
  frc::Pose2d GetStartPose() const;
  std::vector<frc::Pose2d> GetTrajectoryPoses() const;
  std::vector<VehicleState> GetStates() const;
  VehicleState GetStartState() const;
  VehicleState GetEndState() const;
  VehicleState Sample(double timeSeconds) const;
};