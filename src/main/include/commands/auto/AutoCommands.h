// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Command.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveTrajectory.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "subsystems/superstructure/Superstructure.h"

namespace org::littletonrobotics::frc2025::commands {

class AutoCommands {
public:
  static frc2::command::Command *
  ResetPoseCommand(HolonomicTrajectory trajectory, bool mirror);
  static void ResetPose(HolonomicTrajectory trajectory, bool mirror);

  static DriveTrajectory *
  CoralScoringTrajectory(Drive &drive, HolonomicTrajectory trajectory,
                         FieldConstants::CoralObjective coralObjective,
                         bool mirror);

  static frc2::command::Command *DriveAimAtBranch(
      DriveTrajectory *trajectoryCommand,
      std::function<FieldConstants::CoralObjective()> coralObjective);

  static frc2::command::Command *SuperstructureAimAndEjectCommand(
      Superstructure &superstructure,
      FieldConstants::CoralObjective coralObjective, bool mirror,
      std::function<bool()> eject);

  static bool IsXCrossed(double x, bool towardsDriverStation);

private:
  AutoCommands() = delete;
};

} // namespace org::littletonrobotics::frc2025::commands