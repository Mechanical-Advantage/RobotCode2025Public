// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/auto/AutoCommands.h"

#include <functional>
#include <optional>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveTrajectory.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "subsystems/superstructure/Superstructure.h"
#include "util/AllianceFlipUtil.h"
#include "util/MirrorUtil.h"

using namespace org::littletonrobotics::frc2025::commands;

frc2::command::Command *
AutoCommands::ResetPoseCommand(HolonomicTrajectory trajectory, bool mirror) {
  return frc2::command::Commands::RunOnce([&, trajectory, mirror]() {
    RobotState::GetInstance().ResetPose(AllianceFlipUtil::Apply(
        mirror ? MirrorUtil::Apply(trajectory.GetStartPose())
               : trajectory.GetStartPose()));
  });
}

void AutoCommands::ResetPose(HolonomicTrajectory trajectory, bool mirror) {
  RobotState::GetInstance().ResetPose(AllianceFlipUtil::Apply(
      mirror ? MirrorUtil::Apply(trajectory.GetStartPose())
             : trajectory.GetStartPose()));
}

DriveTrajectory *AutoCommands::CoralScoringTrajectory(
    Drive &drive, HolonomicTrajectory trajectory,
    FieldConstants::CoralObjective coralObjective, bool mirror) {
  return new DriveTrajectory(
      drive, trajectory,
      [&, coralObjective, mirror]() {
        return AutoScoreCommands::GetRobotPose(
            mirror ? MirrorUtil::Apply(coralObjective) : coralObjective);
      },
      mirror);
}

frc2::command::Command *AutoCommands::DriveAimAtBranch(
    DriveTrajectory *trajectoryCommand,
    std::function<FieldConstants::CoralObjective()> coralObjective) {
  return frc2::command::Commands::Run([&, trajectoryCommand, coralObjective]() {
           trajectoryCommand->SetOverrideRotation(
               std::optional<frc::Rotation2d>(
                   AllianceFlipUtil::Apply(
                       AutoScoreCommands::GetBranchPose(coralObjective()))
                       .Translation()
                       .Minus(AutoScoreCommands::GetRobotPose(coralObjective())
                                  .Translation())
                       .Angle()));
         })
      ->FinallyDo([&, trajectoryCommand]() {
        trajectoryCommand->SetOverrideRotation(std::nullopt);
      });
}

frc2::command::Command *AutoCommands::SuperstructureAimAndEjectCommand(
    Superstructure &superstructure,
    FieldConstants::CoralObjective coralObjective, bool mirror,
    std::function<bool()> eject) {
  return AutoScoreCommands::SuperstructureAimAndEject(
      superstructure, [&]() { return coralObjective.reefLevel; },
      [&, coralObjective, mirror]() {
        return std::optional<frc::Pose2d>(
            mirror ? MirrorUtil::Apply(coralObjective) : coralObjective);
      },
      eject);
}

bool AutoCommands::IsXCrossed(double x, bool towardsDriverStation) {
  double flippedX =
      AllianceFlipUtil::Apply(RobotState::GetInstance().GetEstimatedPose()).X();
  return towardsDriverStation ? flippedX <= x : flippedX >= x;
}