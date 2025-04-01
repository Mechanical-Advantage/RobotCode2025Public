// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Command.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveCommands.h"
#include "commands/DriveToPose.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/superstructure/Superstructure.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureState.h"
#include "util/AllianceFlipUtil.h"
#include "util/Container.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"

namespace org::littletonrobotics::frc2025::commands {

class AlgaeScoreCommands {
public:
  static frc2::command::Command *
  Process(Drive &drive, Superstructure &superstructure,
          std::function<double()> driverX, std::function<double()> driverY,
          std::function<double()> driverOmega,
          frc2::command::Command *joystickDrive,
          std::function<bool()> onOpposingSide, bool eject,
          std::function<bool()> disableAlgaeScoreAutoAlign);

  static frc2::command::Command *
  NetThrowLineup(Drive &drive, Superstructure &superstructure,
                 std::function<double()> driverY,
                 frc2::command::Command *joystickDrive,
                 std::function<bool()> disableAlgaeScoreAutoAlign);

  static frc2::command::Command *NetThrowScore(Drive &drive,
                                               Superstructure &superstructure);

  static util::LoggedTunableNumber throwGripperEjectTime;

private:
  static util::LoggedTunableNumber processLineupXOffset;
  static util::LoggedTunableNumber processLineupYOffset;
  static util::LoggedTunableNumber processEjectDegOffset;
  static util::LoggedTunableNumber throwLineupDistance;
  static util::LoggedTunableNumber throwDriveDistance;
  static util::LoggedTunableNumber throwDriveVelocity;
  static util::LoggedTunableNumber throwReadyLinearTolerance;
  static util::LoggedTunableNumber throwReadyThetaToleranceDeg;

  AlgaeScoreCommands() = delete;
};

} // namespace org::littletonrobotics::frc2025::commands