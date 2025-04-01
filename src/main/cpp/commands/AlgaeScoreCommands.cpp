// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/AlgaeScoreCommands.h"

#include <algorithm>
#include <cmath>
#include <functional>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Commands.h>

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

using namespace org::littletonrobotics::frc2025::commands;

util::LoggedTunableNumber AlgaeScoreCommands::processLineupXOffset(
    "AlgaeScoreCommands/ProcessLineupXOffset", 0.12);
util::LoggedTunableNumber AlgaeScoreCommands::processLineupYOffset(
    "AlgaeScoreCommands/ProcessLineupYOffset", 0.1);
util::LoggedTunableNumber AlgaeScoreCommands::processEjectDegOffset(
    "AlgaeScoreCommands/ProcessEjectDegreeOffset", 15.0);
util::LoggedTunableNumber AlgaeScoreCommands::throwLineupDistance(
    "AlgaeScoreCommands/ThrowLineupDistance", 0.6);
util::LoggedTunableNumber AlgaeScoreCommands::throwDriveDistance(
    "AlgaeScoreCommands/ThrowDriveDistance", 0.4);
util::LoggedTunableNumber AlgaeScoreCommands::throwDriveVelocity(
    "AlgaeScoreCommands/ThrowDriveVelocity", 1.5);
util::LoggedTunableNumber AlgaeScoreCommands::throwGripperEjectTime(
    "AlgaeScoreCommands/ThrowGripperEjectTime", 0.5);
util::LoggedTunableNumber AlgaeScoreCommands::throwReadyLinearTolerance(
    "AlgaeScoreCommands/ThrowReadyLinearTolerance", 0.4);
util::LoggedTunableNumber AlgaeScoreCommands::throwReadyThetaToleranceDeg(
    "AlgaeScoreCommands/ThrowReadyThetaToleranceDegrees", 10.0);

frc2::command::Command *AlgaeScoreCommands::Process(
    Drive &drive, Superstructure &superstructure,
    std::function<double()> driverX, std::function<double()> driverY,
    std::function<double()> driverOmega, frc2::command::Command *joystickDrive,
    std::function<bool()> onOpposingSide, bool eject,
    std::function<bool()> disableAlgaeScoreAutoAlign) {
  return frc2::command::Commands::Either(
             joystickDrive,
             new DriveToPose(
                 drive,
                 [&]() {
                   return AutoScoreCommands::GetDriveTarget(
                       RobotState::GetInstance().GetEstimatedPose(),
                       AllianceFlipUtil::Apply(
                           onOpposingSide()
                               ? FieldConstants::Processor::opposingCenterFace
                               : FieldConstants::Processor::centerFace)
                           .TransformBy(util::GeomUtil::ToTransform2d(
                               DriveConstants::robotWidth / 2.0 +
                                   processLineupXOffset.Get(),
                               processLineupYOffset.Get()))
                           .TransformBy(util::GeomUtil::ToTransform2d(
                               frc::Rotation2d::Degrees(180).Plus(
                                   frc::Rotation2d::Degrees(
                                       eject ? processEjectDegOffset.Get()
                                             : 0.0)))));
                 },
                 [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
                 [&]() {
                   return DriveCommands::GetLinearVelocityFromJoysticks(
                              driverX(), driverY())
                       .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
                 },
                 [&]() {
                   return DriveCommands::GetOmegaFromJoysticks(driverOmega());
                 })
                 ->DeadlineWith(frc2::command::Commands::StartEnd(
                     [&]() { Leds::GetInstance().autoScoring = true; },
                     [&]() { Leds::GetInstance().autoScoring = false; })),
             disableAlgaeScoreAutoAlign)
      ->AlongWith(
          eject ? superstructure.RunGoal(SuperstructureState::PROCESSED)
                : superstructure.RunGoal(SuperstructureState::ALGAE_STOW));
}

frc2::command::Command *AlgaeScoreCommands::NetThrowLineup(
    Drive &drive, Superstructure &superstructure,
    std::function<double()> driverY, frc2::command::Command *joystickDrive,
    std::function<bool()> disableAlgaeScoreAutoAlign) {
  auto *autoAlignCommand = new DriveToPose(
      drive,
      [&]() {
        return frc::Pose2d(
            AllianceFlipUtil::ApplyX(
                FieldConstants::fieldLength / 2.0 -
                FieldConstants::Barge::netWidth / 2.0 -
                FieldConstants::algaeDiameter -
                SuperstructureConstants::pivotToTunnelFront *
                    std::cos(20.5 / 180.0 * M_PI) -
                SuperstructureConstants::elevatorMaxTravel *
                    SuperstructureConstants::elevatorAngle.Cos() -
                SuperstructureConstants::dispenserOrigin2d.X() -
                throwLineupDistance.Get()),
            RobotState::GetInstance().GetEstimatedPose().Y(),
            AllianceFlipUtil::Apply(frc::Rotation2d::Degrees(0)));
      },
      [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
      [&]() {
        return DriveCommands::GetLinearVelocityFromJoysticks(0, driverY())
            .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
      },
      [&]() { return 0; });

  return frc2::command::Commands::Either(joystickDrive, autoAlignCommand,
                                         disableAlgaeScoreAutoAlign)
      ->AlongWith(
          frc2::command::Commands::WaitUntil([&]() {
            return disableAlgaeScoreAutoAlign() ||
                   (autoAlignCommand->IsRunning() &&
                    autoAlignCommand->WithinTolerance(
                        throwReadyLinearTolerance.Get(),
                        frc::Rotation2d::Degrees(
                            throwReadyThetaToleranceDeg.Get())));
          }).AndThen(superstructure.RunGoal(SuperstructureState::PRE_THROWN)));
}

frc2::command::Command *
AlgaeScoreCommands::NetThrowScore(Drive &drive,
                                  Superstructure &superstructure) {
  util::Container<frc::Pose2d> startPose;
  frc::Timer driveTimer;
  return frc2::command::Commands::RunOnce([&]() {
           startPose.value = RobotState::GetInstance().GetEstimatedPose();
           driveTimer.Restart();
         })
      .AndThen(
          new DriveToPose(drive, [&]() {
            return startPose.value.TransformBy(util::GeomUtil::ToTransform2d(
                std::min(driveTimer.Get() * throwDriveVelocity.Get(),
                         throwDriveDistance.Get()),
                0));
          })->AlongWith(superstructure.RunGoal(SuperstructureState::THROWN)));
}