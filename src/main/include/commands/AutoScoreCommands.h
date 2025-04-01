// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <list>
#include <optional>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/Trigger.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/DriveToPose.h"
#include "commands/IntakeCommands.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/rollers/RollerSystem.h"
#include "subsystems/superstructure/Superstructure.h"
#include "subsystems/superstructure/SuperstructurePose.h"
#include "subsystems/superstructure/SuperstructureState.h"
#include "util/AllianceFlipUtil.h"
#include "util/Container.h"
#include "util/LoggedTunableNumber.h"

namespace org::littletonrobotics::frc2025::commands {

class AutoScoreCommands {
public:
  static frc2::command::Command *AutoScore(
      Drive &drive, Superstructure &superstructure, RollerSystem &funnel,
      std::function<FieldConstants::ReefLevel()> reefLevel,
      std::function<std::optional<FieldConstants::CoralObjective>()>
          coralObjective,
      std::function<double()> driverX = []() { return 0; },
      std::function<double()> driverY = []() { return 0; },
      std::function<double()> driverOmega = []() { return 0; },
      frc2::command::Command *joystickDrive = nullptr,
      frc2::command::Trigger *disableReefAutoAlign = nullptr,
      std::function<bool()> manualEject = []() { return false; });

  static frc2::command::Command *
  AutoScore(Drive &drive, Superstructure &superstructure, RollerSystem &funnel,
            std::function<FieldConstants::ReefLevel()> reefLevel,
            std::function<std::optional<FieldConstants::CoralObjective>()>
                coralObjective);

  static frc2::command::Command *ReefIntake(
      Drive &drive, Superstructure &superstructure,
      std::function<std::optional<FieldConstants::AlgaeObjective>()>
          algaeObjective,
      std::function<double()> driverX = []() { return 0; },
      std::function<double()> driverY = []() { return 0; },
      std::function<double()> driverOmega = []() { return 0; },
      frc2::command::Command *joystickDrive = nullptr,
      frc2::command::Trigger *disableReefAutoAlign = nullptr);

  static frc2::command::Command *SuperstructureAimAndEject(
      Superstructure &superstructure,
      std::function<FieldConstants::ReefLevel()> reefLevel,
      std::function<std::optional<FieldConstants::CoralObjective>()>
          coralObjective,
      std::function<bool()> eject,
      std::function<bool()> disableReefAutoAlign = []() { return false; });

  static frc2::command::Command *SuperstructureAimAndEject(
      Superstructure &superstructure,
      std::function<FieldConstants::ReefLevel()> reefLevel,
      std::function<std::optional<FieldConstants::CoralObjective>()>
          coralObjective,
      std::function<bool()> eject);

  static frc::Pose2d GetDriveTarget(frc::Pose2d robot, frc::Pose2d goal);

  static frc::Pose2d
  GetCoralScorePose(FieldConstants::CoralObjective coralObjective);

  static frc::Pose2d
  GetReefIntakePose(FieldConstants::AlgaeObjective objective);

  static bool WithinDistanceToReef(frc::Pose2d robot, double distance);

  static bool OutOfDistanceToReef(frc::Pose2d robot, double distance);

  static frc::Pose2d
  GetRobotPose(FieldConstants::CoralObjective coralObjective);

  static frc::Pose2d GetBranchPose(FieldConstants::CoralObjective objective);

private:
  static util::LoggedTunableNumber maxDistanceReefLineup;
  static util::LoggedTunableNumber minDistanceReefClearAlgae;
  static util::LoggedTunableNumber algaeBackupTime;
  static util::LoggedTunableNumber minDistanceReefClear;
  static util::LoggedTunableNumber distanceSuperstructureReady;
  static std::array<util::LoggedTunableNumber, 4> linearXToleranceEject;
  static std::array<util::LoggedTunableNumber, 4> linearYToleranceEject;
  static std::array<util::LoggedTunableNumber, 4> maxLinearVel;
  static std::array<util::LoggedTunableNumber, 4> maxAngularVel;
  static util::LoggedTunableNumber thetaToleranceEject;
  static util::LoggedTunableNumber l2ReefIntakeDistance;
  static util::LoggedTunableNumber l3ReefIntakeDistance;
  static util::LoggedTunableNumber l1AlignOffsetX;
  static util::LoggedTunableNumber l1AlignOffsetY;
  static util::LoggedTunableNumber minDistanceAim;
  static util::LoggedTunableNumber ejectTimeSeconds;

  AutoScoreCommands() = delete;

  static frc2::command::Command *
  SuperstructureGetBack(Superstructure &superstructure,
                        std::function<SuperstructureState()> holdState,
                        std::function<bool()> disableReefAutoAlign);

  static frc::Pose2d
  GetRobotPose(FieldConstants::AlgaeObjective algaeObjective);

  static frc::Pose2d GetL1Pose(FieldConstants::CoralObjective coralObjective);
};

} // namespace org::littletonrobotics::frc2025::commands