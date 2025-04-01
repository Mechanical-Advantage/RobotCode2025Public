// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <frc/Timer.h>
#include <frc/filter/Debouncer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/util/Units.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/AutoCommands.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveToPose.h"
#include "commands/DriveToStation.h"
#include "commands/DriveTrajectory.h"
#include "commands/IntakeCommands.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "subsystems/rollers/RollerSystem.h"
#include "subsystems/superstructure/Superstructure.h"

namespace org::littletonrobotics::frc2025::commands::auto_ {

class AutoBuilder {
public:
  AutoBuilder(Drive &drive, Superstructure &superstructure,
              RollerSystem &funnel);

  frc2::command::Command *SuperUpInTheWaterAuto();
  frc2::command::Command *UpInTheWaterAuto();
  frc2::command::Command *UpInTheWeedsAuto(bool isElims);
  frc2::command::Command *UpInTheSimplicityAuto();
  frc2::command::Command *UpInTheInspirationalAuto();

private:
  frc2::command::SequentialCommandGroup *
  GetUpInTheWaterSequence(int coralScoreIndex,
                          const FieldConstants::CoralObjective *coralObjectives,
                          frc::Timer &autoTimer);

  Drive &drive;
  Superstructure &superstructure;
  RollerSystem &funnel;

  const double intakeTimeSeconds = 0.35;
  const double coralEjectTimeSeconds = 0.3;
};

} // namespace org::littletonrobotics::frc2025::commands::auto_