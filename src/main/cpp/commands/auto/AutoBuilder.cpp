// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/auto/AutoBuilder.h"

#include <iostream>
#include <vector>

#include <frc/Timer.h>
#include <frc/filter/Debouncer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/util/Units.h>
#include <frc2/command/Commands.h>
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
#include "util/AllianceFlipUtil.h"
#include "util/MirrorUtil.h"

using namespace org::littletonrobotics::frc2025::commands::auto_;

AutoBuilder::AutoBuilder(Drive &drive, Superstructure &superstructure,
                         RollerSystem &funnel)
    : drive(drive), superstructure(superstructure), funnel(funnel) {}

frc2::command::Command *AutoBuilder::SuperUpInTheWaterAuto() {
  HolonomicTrajectory upInTheWater1Score("SuperUpInTheWater1Score");
  HolonomicTrajectory upInTheWater1Intake("SuperUpInTheWater1Intake");

  FieldConstants::ReefLevel level =
      Constants::GetRobot() == Constants::Robot::kDevbot
          ? FieldConstants::ReefLevel::kL3
          : FieldConstants::ReefLevel::kL4;
  std::array<FieldConstants::CoralObjective, 4> coralObjectives = {
      FieldConstants::CoralObjective{9, level},
      FieldConstants::CoralObjective{10, level},
      FieldConstants::CoralObjective{11, level},
      FieldConstants::CoralObjective{0, level}};

  frc::Timer *autoTimer = new frc::Timer();
  return frc2::command::Commands::RunOnce([&]() {
           AutoCommands::ResetPose(upInTheWater1Score, true);
           autoTimer->Restart();
           superstructure.SetAutoStart();
         })
      .AndThen(
          // First score and intake
          frc2::command::Commands::Sequence(
              AutoCommands::CoralScoringTrajectory(drive, upInTheWater1Score,
                                                   coralObjectives[0], true),
              new DriveTrajectory(drive, upInTheWater1Intake, true))
              ->DeadlineWith(frc2::command::Commands::Sequence(
                  AutoCommands::SuperstructureAimAndEjectCommand(
                      superstructure, coralObjectives[0], true,
                      [&]() {
                        return autoTimer->HasElapsed(
                            upInTheWater1Score.GetDuration() -
                            coralEjectTimeSeconds);
                      }),
                  frc2::command::Commands::RunOnce([&]() {
                    std::printf("Scored Coral #1 at %.2f\n", autoTimer->Get());
                  }),
                  IntakeCommands::Intake(superstructure, funnel))),

          // Rest of them
          frc2::command::Commands::Sequence(
              GetUpInTheWaterSequence(2, coralObjectives.data(), *autoTimer),
              GetUpInTheWaterSequence(3, coralObjectives.data(), *autoTimer),
              GetUpInTheWaterSequence(4, coralObjectives.data(), *autoTimer)));
}

frc2::command::Command *AutoBuilder::UpInTheWaterAuto() {
  FieldConstants::ReefLevel level =
      Constants::GetRobot() == Constants::Robot::kDevbot
          ? FieldConstants::ReefLevel::kL3
          : FieldConstants::ReefLevel::kL4;
  std::array<FieldConstants::CoralObjective, 4> coralObjectives = {
      FieldConstants::CoralObjective{9, level},
      FieldConstants::CoralObjective{10, level},
      FieldConstants::CoralObjective{11, level},
      FieldConstants::CoralObjective{0, level}};

  frc::Timer *autoTimer = new frc::Timer();
  return frc2::command::Commands::RunOnce([&]() {
           AutoCommands::ResetPose(HolonomicTrajectory("UpInTheWater1Score"),
                                   true);
           autoTimer->Restart();
           superstructure.SetAutoStart();
         })
      .AndThen(frc2::command::Commands::Sequence(
          GetUpInTheWaterSequence(1, coralObjectives.data(), *autoTimer),
          GetUpInTheWaterSequence(2, coralObjectives.data(), *autoTimer),
          GetUpInTheWaterSequence(3, coralObjectives.data(), *autoTimer),
          GetUpInTheWaterSequence(4, coralObjectives.data(), *autoTimer)));
}

frc2::command::SequentialCommandGroup *AutoBuilder::GetUpInTheWaterSequence(
    int coralScoreIndex, const FieldConstants::CoralObjective *coralObjectives,
    frc::Timer &autoTimer) {
  FieldConstants::CoralObjective coralObjective =
      coralObjectives[coralScoreIndex - 1];
  HolonomicTrajectory coralScoringTrajectory(
      "UpInTheWater" + std::to_string(coralScoreIndex) + "Score");
  HolonomicTrajectory *coralIntakingTrajectory =
      coralScoreIndex != 4
          ? new HolonomicTrajectory("UpInTheWater" +
                                    std::to_string(coralScoreIndex) + "Intake")
          : nullptr;

  DriveTrajectory *driveScoringCommand = AutoCommands::CoralScoringTrajectory(
      drive, coralScoringTrajectory, coralObjective, true);
  DriveTrajectory *driveIntakingCommand =
      coralScoreIndex != 4
          ? new DriveTrajectory(drive, *coralIntakingTrajectory, true)
          : nullptr;

  frc::Timer *timer = new frc::Timer();
  return new frc2::command::SequentialCommandGroup(
      frc2::command::Commands::RunOnce([&]() { timer->Restart(); }),
      frc2::command::Commands::Sequence(
          IntakeCommands::Intake(superstructure, funnel)->Until([&]() {
            return timer->HasElapsed(coralScoringTrajectory.GetDuration() -
                                     1.6);
          }),
          AutoCommands::SuperstructureAimAndEjectCommand(
              superstructure, coralObjective, true,
              [&]() {
                return timer->HasElapsed(coralScoringTrajectory.GetDuration() -
                                         coralEjectTimeSeconds / 2.0);
              }),
          frc2::command::Commands::RunOnce([&]() {
            std::printf("Scored Coral #%d at %.2f\n", coralScoreIndex,
                        autoTimer.Get());
          }),
          coralScoreIndex != 4
              ? frc2::command::Commands::WaitUntil([&]() {
                  return AutoScoreCommands::OutOfDistanceToReef(
                      RobotState::GetInstance().GetEstimatedPose(), 0.10);
                })
                    .AndThen(IntakeCommands::Intake(superstructure, funnel)
                                 ->Until([&]() {
                                   return timer->HasElapsed(
                                       coralScoringTrajectory.GetDuration() +
                                       coralIntakingTrajectory->GetDuration() +
                                       intakeTimeSeconds);
                                 }))
              : superstructure
                    .RunGoal(Superstructure::GetScoringState(
                        coralObjective.reefLevel, false))
                    ->Until([&]() { return autoTimer.HasElapsed(15.3); }))
          ->DeadlineWith(frc2::command::Commands::Sequence(
              driveScoringCommand, coralScoreIndex != 4
                                       ? driveIntakingCommand
                                       : frc2::command::Commands::None())));
}

frc2::command::Command *AutoBuilder::UpInTheWeedsAuto(bool isElims) {
  const double intakeTimeSeconds = 0.15;
  const double driveToStationBiasSeconds = 0.2;
  std::array<FieldConstants::CoralObjective, 3> coralObjectives = {
      FieldConstants::CoralObjective{9, isElims
                                            ? FieldConstants::ReefLevel::kL4
                                            : FieldConstants::ReefLevel::kL2},
      FieldConstants::CoralObjective{10, FieldConstants::ReefLevel::kL4},
      FieldConstants::CoralObjective{0, FieldConstants::ReefLevel::kL2}};

  frc::Timer *autoTimer = new frc::Timer();
  return frc2::command::Commands::RunOnce([&]() {
           RobotState::GetInstance().ResetPose(
               AllianceFlipUtil::Apply(MirrorUtil::Apply(
                   frc::Pose2d(startingLineX - DriveConstants::robotWidth / 2.0,
                               FieldConstants::fieldWidth -
                                   FieldConstants::Barge::closeCage.Y(),
                               frc::Rotation2d::Degrees(90)))));
           superstructure.SetAutoStart();
           autoTimer->Restart();
         })
      .AndThen(
          new DriveToPose(
              drive,
              [&]() {
                return AutoScoreCommands::GetCoralScorePose(
                    MirrorUtil::Apply(coralObjectives[0]));
              },
              [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
              [&]() {
                return AllianceFlipUtil::Apply(frc::Translation2d(-3.0, 0.0));
              },
              [&]() { return 0.0; })
              ->Until([&]() {
                return AutoCommands::IsXCrossed(startingLineX - 0.5, true);
              }),
          frc2::command::Commands::Sequence([&]() {
            std::vector<frc2::command::Command *> commands;
            for (int index = 0; index < 3; ++index) {
              auto *driveToStation = new DriveToStation(drive, true);
              frc::Debouncer *intakingDebouncer =
                  new frc::Debouncer(intakeTimeSeconds);
              commands.push_back(
                  AutoScoreCommands::AutoScore(
                      drive, superstructure, funnel,
                      [&, index]() { return coralObjectives[index].reefLevel; },
                      [&, index]() {
                        return std::optional<frc::Pose2d>(
                            MirrorUtil::Apply(coralObjectives[index]));
                      })
                      ->WithTimeout(4.0)
                      .AndThen(
                          frc2::command::Commands::RunOnce([&, index]() {
                            std::printf("Scored Coral #%d at %.2f\n", index + 1,
                                        autoTimer->Get());
                            intakingDebouncer->Calculate(false);
                          }),
                          new DriveToStation(
                              drive,
                              [&, index]() {
                                return AutoScoreCommands::GetCoralScorePose(
                                           MirrorUtil::Apply(
                                               coralObjectives[index]))
                                    .Translation()
                                    .Minus(FieldConstants::Reef::center)
                                    .Times(AllianceFlipUtil::ShouldFlip()
                                               ? -1.0
                                               : 1.0);
                              },
                              [&]() { return 0.0; }, true)
                              ->AlongWith(superstructure.RunGoal(
                                  Superstructure::GetScoringState(
                                      coralObjectives[index].reefLevel, false)))
                              ->WithTimeout(driveToStationBiasSeconds),
                          driveToStation
                              ->AlongWith(IntakeCommands::Intake(superstructure,
                                                                 funnel))
                              ->Until([&, driveToStation, intakingDebouncer]() {
                                return intakingDebouncer->Calculate(
                                    driveToStation->IsRunning() &&
                                    driveToStation->WithinTolerance(
                                        frc::Units::InchesToMeters(5.0),
                                        frc::Rotation2d::Degrees(10.0)));
                              })));
            }
            return frc2::command::Commands::Sequence(commands);
          }()
                                                ->Unwrap()));
}

frc2::command::Command *AutoBuilder::UpInTheSimplicityAuto() {
  const auto objective =
      FieldConstants::CoralObjective{7, FieldConstants::ReefLevel::kL4};
  return frc2::command::Commands::RunOnce([&]() {
           RobotState::GetInstance().ResetPose(
               AllianceFlipUtil::Apply(MirrorUtil::Apply(
                   frc::Pose2d(startingLineX - DriveConstants::robotWidth / 2.0,
                               FieldConstants::fieldWidth / 2.0,
                               frc::Rotation2d::Degrees(180)))));
           superstructure.SetAutoStart();
         })
      .AndThen(
          AutoScoreCommands::AutoScore(
              drive, superstructure, funnel,
              [&]() { return objective.reefLevel; },
              [&]() {
                return std::optional<frc::Pose2d>(MirrorUtil::Apply(objective));
              }),
          new DriveToPose(
              drive,
              [&]() {
                return AutoScoreCommands::GetCoralScorePose(objective).Plus(
                    frc::Transform2d(-0.5, 0.0, frc::Rotation2d::Degrees(0)));
              })
              ->WithTimeout(3.0)
              ->DeadlineWith(
                  superstructure.RunGoal(Superstructure::GetScoringState(
                      objective.reefLevel, false))));
}

frc2::command::Command *AutoBuilder::UpInTheInspirationalAuto() {
  return frc2::command::Commands::RunOnce([&]() {
           RobotState::GetInstance().ResetPose(
               AllianceFlipUtil::Apply(frc::Pose2d(
                   RobotState::GetInstance().GetEstimatedPose().Translation(),
                   frc::Rotation2d::Degrees(180))));
         })
      .AndThen(
          new DriveToPose(
              drive,
              [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
              [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
              [&]() {
                return frc::Translation2d(
                    (AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0) * -1.0, 0.0);
              },
              [&]() { return 0.0; })
              ->WithTimeout(0.6));
}
