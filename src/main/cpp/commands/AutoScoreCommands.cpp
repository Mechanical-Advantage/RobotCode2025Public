// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/AutoScoreCommands.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <list>
#include <optional>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/units/units.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/DriveCommands.h"
#include "commands/DriveToPose.h"
#include "commands/IntakeCommands.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/rollers/RollerSystem.h"
#include "subsystems/superstructure/Superstructure.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructurePose.h"
#include "subsystems/superstructure/SuperstructureState.h"
#include "util/AllianceFlipUtil.h"
#include "util/Container.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"
#include "util/Util.h"
#include "wpi/math/MathUtil.h"
#include "wpi/math/numbers.h"

using namespace org::littletonrobotics::frc2025::commands;

util::LoggedTunableNumber
    AutoScoreCommands::maxDistanceReefLineup("AutoScore/MaxDistanceReefLineup",
                                             1.5);
util::LoggedTunableNumber AutoScoreCommands::minDistanceReefClearAlgae(
    "AutoScore/MinDistanceReefClearAlgae", frc::units::inch_to_meter(18.0));
util::LoggedTunableNumber
    AutoScoreCommands::algaeBackupTime("AutoScore/AlgaeBackupTime", 1.2);
util::LoggedTunableNumber
    AutoScoreCommands::minDistanceReefClear("AutoScore/MinDistanceReefClear",
                                            frc::units::inch_to_meter(12.0));
util::LoggedTunableNumber AutoScoreCommands::distanceSuperstructureReady(
    "AutoScore/DistanceSuperstructureReady", frc::units::inch_to_meter(72.0));
std::array<util::LoggedTunableNumber, 4>
    AutoScoreCommands::linearXToleranceEject = {
        util::LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.03),
        util::LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.15),
        util::LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.15),
        util::LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.025)};
std::array<util::LoggedTunableNumber, 4>
    AutoScoreCommands::linearYToleranceEject = {
        util::LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.03),
        util::LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.015),
        util::LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.015),
        util::LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.01)};
std::array<util::LoggedTunableNumber, 4> AutoScoreCommands::maxLinearVel = {
    util::LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 0.1),
    util::LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 0.1),
    util::LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 0.1),
    util::LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 0.1)};
std::array<util::LoggedTunableNumber, 4> AutoScoreCommands::maxAngularVel = {
    util::LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 1),
    util::LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 1),
    util::LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 1),
    util::LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 1)};
util::LoggedTunableNumber
    AutoScoreCommands::thetaToleranceEject("AutoScore/ThetaToleranceEject",
                                           2.0);
util::LoggedTunableNumber
    AutoScoreCommands::l2ReefIntakeDistance("AutoScore/L2ReefIntakeDistance",
                                            0.05);
util::LoggedTunableNumber
    AutoScoreCommands::l3ReefIntakeDistance("AutoScore/L3ReefIntakeDistance",
                                            0.08);
util::LoggedTunableNumber
    AutoScoreCommands::l1AlignOffsetX("AutoScore/L1AlignOffsetX", 0.45);
util::LoggedTunableNumber AutoScoreCommands::l1AlignOffsetY(
    "AutoScore/L1AlignOffsetY",
    FieldConstants::Reef::faceLength / 2.0 - frc::units::inch_to_meter(2.5));
util::LoggedTunableNumber
    AutoScoreCommands::minDistanceAim("AutoScore/MinDistanceAim", 0.2);
util::LoggedTunableNumber
    AutoScoreCommands::ejectTimeSeconds("AutoScore/EjectTimeSeconds", 0.5);

frc2::command::Command *AutoScoreCommands::AutoScore(
    Drive &drive, Superstructure &superstructure, RollerSystem &funnel,
    std::function<FieldConstants::ReefLevel()> reefLevel,
    std::function<std::optional<FieldConstants::CoralObjective>()>
        coralObjective,
    std::function<double()> driverX, std::function<double()> driverY,
    std::function<double()> driverOmega, frc2::command::Command *joystickDrive,
    frc2::command::Trigger *disableReefAutoAlign,
    std::function<bool()> manualEject) {
  std::function<frc::Pose2d()> robot = [&]() {
    return coralObjective().has_value()
               ? GetRobotPose(coralObjective().value())
               : RobotState::GetInstance().GetEstimatedPose();
  };

  std::function<frc::Pose2d(FieldConstants::CoralObjective)> goal =
      [&](FieldConstants::CoralObjective objective) {
        return objective.reefLevel == FieldConstants::ReefLevel::kL1
                   ? GetL1Pose(objective)
                   : GetCoralScorePose(objective);
      };

  util::Container<FieldConstants::CoralObjective> coralObjectiveScored;
  util::Container<bool> needsToGetBack(false);
  util::Container<bool> hasEnded(false);

  auto *driveCommand = new DriveToPose(
      drive,
      [&]() {
        if (coralObjective().has_value()) {
          auto objective = coralObjective().value();
          if (reefLevel() == FieldConstants::ReefLevel::kL1) {
            return GetDriveTarget(robot(), GetL1Pose(objective));
          }
          frc::Pose2d goalPose = GetCoralScorePose(objective);
          if (superstructure.GetState() !=
                  Superstructure::GetScoringState(reefLevel(), false) &&
              superstructure.GetState() !=
                  Superstructure::GetScoringState(reefLevel(), true)) {
            goalPose = goalPose.TransformBy(util::GeomUtil::ToTransform2d(
                goalPose.Translation().Distance(FieldConstants::Reef::center) -
                    FieldConstants::Reef::faceLength -
                    (DriveConstants::robotWidth / 2.0) -
                    minDistanceReefClear.Get(),
                0.0));
          }
          goalPose = frc::Pose2d(goalPose.Translation(),
                                 GetBranchPose(objective)
                                     .Translation()
                                     .Minus(robot().Translation())
                                     .Rotation());
          return GetDriveTarget(robot(), AllianceFlipUtil::Apply(goalPose));
        } else {
          return RobotState::GetInstance().GetEstimatedPose();
        }
      },
      robot,
      [&]() {
        return DriveCommands::GetLinearVelocityFromJoysticks(driverX(),
                                                             driverY())
            .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
      },
      [&]() { return DriveCommands::GetOmegaFromJoysticks(driverOmega()); });

  // Schedule get back command
  frc2::command::button::Trigger(
      [&]() { return hasEnded.value && needsToGetBack.value; })
      .And(frc2::command::button::RobotModeTriggers::Teleop())
      .And(disableReefAutoAlign->Negate())
      .OnTrue(SuperstructureGetBack(
                  superstructure, [&]() { return superstructure.GetState(); },
                  [&]() { return disableReefAutoAlign->Get(); })
                  .AndThen([&]() { needsToGetBack.value = false; })
                  .WithInterruptBehavior(
                      frc2::command::Command::InterruptionBehavior::
                          kCancelIncoming));

  return frc2::command::Commands::RunOnce([&]() {
           // Start LEDs
           Leds::GetInstance().autoScoringReef = true;
           Leds::GetInstance().autoScoringLevel = reefLevel();

           // Reset state
           needsToGetBack.value = false;
           hasEnded.value = false;

           // Log reef level
           wpi::log::DataLogEntry(wpi::log::StringLog("AutoScore/ReefLevel"))
               .Append(util::ToString(reefLevel()));

           // Clear logs
           wpi::log::DataLogEntry(
               wpi::log::BooleanLog("AutoScore/AllowPreReady"))
               .Append(false);
           wpi::log::DataLogEntry(wpi::log::BooleanLog("AutoScore/AllowEject"))
               .Append(false);
         })
      .AndThen(
          // Run superstructure
          IntakeCommands::Intake(superstructure, funnel).Until([&]() {
            return superstructure.HasCoral();
          }),
          // Check if need wait until pre ready or already ready
          frc2::command::Commands::WaitUntil([&]() {
            bool ready = WithinDistanceToReef(
                             robot(), distanceSuperstructureReady.Get()) ||
                         disableReefAutoAlign->Get();
            wpi::log::DataLogEntry(
                wpi::log::BooleanLog("AutoScore/AllowPreReady"))
                .Append(ready);

            // Get back!
            if (ready) {
              needsToGetBack.value = true;
            }
            return ready;
          }),
          SuperstructureAimAndEject(
              superstructure, reefLevel, coralObjective,
              [&]() {
                if (!coralObjective().has_value())
                  return false;
                frc::Pose2d poseError =
                    AllianceFlipUtil::Apply(robot()).RelativeTo(
                        goal(coralObjective().value()));

                int intReefLevel =
                    static_cast<int>(coralObjective().value().reefLevel);
                bool ready =
                    (std::abs(poseError.Translation().X()) <=
                         linearXToleranceEject[intReefLevel].Get() &&
                     std::abs(poseError.Translation().Y()) <=
                         linearYToleranceEject[intReefLevel].Get() &&
                     std::hypot(
                         RobotState::GetInstance().GetRobotVelocity().Vx(),
                         RobotState::GetInstance().GetRobotVelocity().Vy()) <=
                         maxLinearVel[intReefLevel].Get() &&
                     std::abs(RobotState::GetInstance()
                                  .GetRobotVelocity()
                                  .Omega()) <=
                         maxAngularVel[intReefLevel].Get() &&
                     std::abs(poseError.Rotation().Degrees()) <=
                         thetaToleranceEject.Get() &&
                     superstructure.AtGoal() && !disableReefAutoAlign->Get()) ||
                    manualEject();
                wpi::log::DataLogEntry(
                    wpi::log::BooleanLog("AutoScore/AllowEject"))
                    .Append(ready);
                if (ready) {
                  coralObjectiveScored.value = coralObjective().value();
                }
                return ready;
              },
              [&]() { return disableReefAutoAlign->Get(); }))
      .DeadlineWith(frc2::command::Commands::Either(
          joystickDrive, driveCommand,
          disableReefAutoAlign)) // Deadline with driving command
      .FinallyDo([&](bool interrupted) {
        RobotState::GetInstance().SetDistanceToBranch(std::nullopt);

        // Clear logs
        wpi::log::DataLogEntry(wpi::log::StringLog("AutoScore/ReefLevel"))
            .Append("");
        wpi::log::DataLogEntry(wpi::log::BooleanLog("AutoScore/AllowPreReady"))
            .Append(false);
        wpi::log::DataLogEntry(wpi::log::BooleanLog("AutoScore/AllowEject"))
            .Append(false);

        // Stop LEDs
        Leds::GetInstance().autoScoringReef = false;

        // Indicate has ended command
        hasEnded.value = true;
      });
}

frc2::command::Command *AutoScoreCommands::AutoScore(
    Drive &drive, Superstructure &superstructure, RollerSystem &funnel,
    std::function<FieldConstants::ReefLevel()> reefLevel,
    std::function<std::optional<FieldConstants::CoralObjective>()>
        coralObjective) {
  return AutoScore(
      drive, superstructure, funnel, reefLevel, coralObjective,
      []() { return 0; }, []() { return 0; }, []() { return 0; }, nullptr,
      nullptr, []() { return false; });
}

frc2::command::Command *AutoScoreCommands::ReefIntake(
    Drive &drive, Superstructure &superstructure,
    std::function<std::optional<FieldConstants::AlgaeObjective>()>
        algaeObjective,
    std::function<double()> driverX, std::function<double()> driverY,
    std::function<double()> driverOmega, frc2::command::Command *joystickDrive,
    frc2::command::Trigger *disableReefAutoAlign) {
  std::function<frc::Pose2d()> robot = [&]() {
    return algaeObjective().has_value()
               ? GetRobotPose(algaeObjective().value())
               : RobotState::GetInstance().GetEstimatedPose();
  };

  std::function<SuperstructureState()> algaeIntakeState = [&]() {
    if (algaeObjective().has_value()) {
      return algaeObjective().value().id % 2 == 0
                 ? SuperstructureState::ALGAE_L3_INTAKE
                 : SuperstructureState::ALGAE_L2_INTAKE;
    } else {
      return superstructure.GetState();
    }
  };

  util::Container<FieldConstants::AlgaeObjective> algaeIntaked;
  util::Container<bool> needsToGetBack(false);
  util::Container<bool> hasEnded(false);

  frc::Timer hasAlgaeTimer;
  hasAlgaeTimer.Start();
  std::function<frc::Pose2d()> goal = [&]() {
    frc::Pose2d goalPose =
        algaeObjective().has_value()
            ? AllianceFlipUtil::Apply(
                  GetReefIntakePose(algaeObjective().value()))
            : (algaeIntaked.value.has_value()
                   ? AllianceFlipUtil::Apply(
                         GetReefIntakePose(algaeIntaked.value.value()))
                   : robot());
    if (!algaeObjective().has_value() && !algaeIntaked.value.has_value()) {
      return goalPose;
    }
    if (superstructure.HasAlgae()) {
      return goalPose.TransformBy(util::GeomUtil::ToTransform2d(
          -minDistanceReefClearAlgae.Get() *
              std::min(1.0, hasAlgaeTimer.Get() / algaeBackupTime.Get()),
          0.0));
    } else {
      hasAlgaeTimer.Restart();
    }
    return goalPose;
  };

  // Schedule get back command
  frc2::command::button::Trigger(
      [&]() { return hasEnded.value && needsToGetBack.value; })
      .And(frc2::command::button::RobotModeTriggers::Teleop())
      .And(disableReefAutoAlign->Negate())
      .OnTrue(
          SuperstructureGetBack(superstructure, algaeIntakeState,
                                [&]() { return disableReefAutoAlign->Get(); })
              .AndThen([&]() { needsToGetBack.value = false; })
              .WithInterruptBehavior(
                  frc2::command::Command::InterruptionBehavior::
                      kCancelIncoming));

  return frc2::command::Commands::RunOnce([&]() {
           // Reset State
           algaeIntaked.value = std::nullopt;
           needsToGetBack.value = false;
           hasEnded.value = false;
         })
      .AndThen(frc2::command::Commands::Either(
          joystickDrive,
          new DriveToPose(
              drive, goal, robot,
              [&]() {
                return DriveCommands::GetLinearVelocityFromJoysticks(driverX(),
                                                                     driverY())
                    .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
              },
              [&]() {
                return DriveCommands::GetOmegaFromJoysticks(driverOmega());
              }),
          disableReefAutoAlign))
      .AlongWith(
          superstructure.RunGoal(algaeIntakeState)
              .AlongWith(frc2::command::Commands::WaitUntil([&]() {
                           return superstructure.GetState() ==
                                  algaeIntakeState();
                         }),
                         frc2::command::Commands::RunOnce(
                             [&]() { needsToGetBack.value = true; })),
          frc2::command::Commands::WaitUntil([&]() {
            return superstructure.HasAlgae();
          }).AndThen(frc2::command::Commands::RunOnce([&]() {
                       algaeIntaked.value = algaeObjective();
                     }).OnlyIf([&]() { return algaeObjective().has_value(); })))
      .FinallyDo([&](bool interrupted) { hasEnded.value = true; });
}

frc2::command::Command *AutoScoreCommands::SuperstructureAimAndEject(
    Superstructure &superstructure,
    std::function<FieldConstants::ReefLevel()> reefLevel,
    std::function<std::optional<FieldConstants::CoralObjective>()>
        coralObjective,
    std::function<bool()> eject, std::function<bool()> disableReefAutoAlign) {
  frc::Timer ejectTimer;
  return superstructure
      .RunGoal(
          [&]() { return Superstructure::GetScoringState(reefLevel(), false); })
      .Until(eject)
      .AndThen(
          frc2::command::Commands::RunOnce([&]() { ejectTimer.Restart(); }),
          superstructure
              .RunGoal([&]() {
                return Superstructure::GetScoringState(reefLevel(), true);
              })
              .Until([&]() {
                return ejectTimer.HasElapsed(ejectTimeSeconds.Get());
              }))
      .DeadlineWith(
          // Measure distance to branch
          frc2::command::Commands::Run([&]() {
            if (coralObjective().has_value()) {
              auto objective = coralObjective().value();
              if (disableReefAutoAlign()) {
                RobotState::GetInstance().SetDistanceToBranch(std::nullopt);
                return;
              }

              if (objective.reefLevel == FieldConstants::ReefLevel::kL1) {
                RobotState::GetInstance().SetDistanceToBranch(std::nullopt);
                return;
              }

              auto dispenserPose =
                  AllianceFlipUtil::Apply(GetRobotPose(objective).TransformBy(
                      util::GeomUtil::ToTransform2d(
                          SuperstructurePose::DispenserPose::forCoralScore(
                              objective.reefLevel)
                                      .GetElevatorHeight() *
                                  SuperstructureConstants::elevatorAngle.Cos() +
                              SuperstructureConstants::dispenserOrigin2d.X(),
                          0.0)));
              auto offsetTranslation =
                  dispenserPose
                      .RelativeTo(GetBranchPose(objective).TransformBy(
                          util::GeomUtil::ToTransform2d(
                              frc::Rotation2d::Degrees(180))))
                      .Translation();
              double distanceToBranch = offsetTranslation.Norm();
              wpi::log::DataLogEntry(
                  wpi::log::DoubleLog("AutoScore/DistanceToBranch"))
                  .Append(distanceToBranch);
              RobotState::GetInstance().SetDistanceToBranch(
                  distanceToBranch <= minDistanceAim.Get()
                      ? std::nullopt
                      : std::make_optional(distanceToBranch));
            } else {
              RobotState::GetInstance().SetDistanceToBranch(std::nullopt);
            }
          }));
}

frc2::command::Command *AutoScoreCommands::SuperstructureAimAndEject(
    Superstructure &superstructure,
    std::function<FieldConstants::ReefLevel()> reefLevel,
    std::function<std::optional<FieldConstants::CoralObjective>()>
        coralObjective,
    std::function<bool()> eject) {
  return SuperstructureAimAndEject(superstructure, reefLevel, coralObjective,
                                   eject, []() { return false; });
}

frc2::command::Command *AutoScoreCommands::SuperstructureGetBack(
    Superstructure &superstructure,
    std::function<SuperstructureState()> holdState,
    std::function<bool()> disableReefAutoAlign) {
  return superstructure.RunGoal(holdState)
      .Until([&]() {
        return superstructure.HasAlgae()
                   ? OutOfDistanceToReef(
                         RobotState::GetInstance().GetEstimatedPose(),
                         minDistanceReefClearAlgae.Get())
                   : OutOfDistanceToReef(
                         RobotState::GetInstance().GetEstimatedPose(),
                         minDistanceReefClear.Get());
      })
      .WithName("Superstructure Get Back!");
}

/** Get drive target. */
frc::Pose2d AutoScoreCommands::GetDriveTarget(frc::Pose2d robot,
                                              frc::Pose2d goal) {
  auto offset = robot.RelativeTo(goal);
  double yDistance = std::abs(offset.Y());
  double xDistance = std::abs(offset.X());
  double shiftXT = wpi::math::numbers::clamp(
      (yDistance / (FieldConstants::Reef::faceLength * 2)) +
          ((xDistance - 0.3) / (FieldConstants::Reef::faceLength * 3)),
      0.0, 1.0);
  double shiftYT = wpi::math::numbers::clamp(
      yDistance <= 0.2 ? 0.0 : offset.X() / FieldConstants::Reef::faceLength,
      0.0, 1.0);
  return goal.TransformBy(util::GeomUtil::ToTransform2d(
      -shiftXT * maxDistanceReefLineup.Get(),
      util::Sign(offset.Y()) * shiftYT * maxDistanceReefLineup.Get() * 0.8));
}

/** Get position of robot aligned with branch for selected objective. */
frc::Pose2d AutoScoreCommands::GetCoralScorePose(
    FieldConstants::CoralObjective coralObjective) {
  return GetBranchPose(coralObjective)
      .TransformBy(SuperstructurePose::DispenserPose::forCoralScore(
                       coralObjective.reefLevel)
                       .ToRobotPose());
}

frc::Pose2d
AutoScoreCommands::GetReefIntakePose(FieldConstants::AlgaeObjective objective) {
  int branchId = objective.id * 2;
  return GetBranchPose(FieldConstants::CoralObjective(
                           branchId, FieldConstants::ReefLevel::kL3))
      .Interpolate(GetBranchPose(FieldConstants::CoralObjective(
                       branchId + 1, FieldConstants::ReefLevel::kL3)),
                   0.5)
      .TransformBy(util::GeomUtil::ToTransform2d(
          (objective.low ? l2ReefIntakeDistance.Get()
                         : l3ReefIntakeDistance.Get()) +
              DriveConstants::robotWidth / 2.0,
          0.0));
}

frc::Pose2d
AutoScoreCommands::GetL1Pose(FieldConstants::CoralObjective coralObjective) {
  frc::Pose2d centerFace =
      FieldConstants::Reef::centerFaces[coralObjective.branchId / 2]
          .TransformBy(frc::Transform2d(l1AlignOffsetX.Get(), 0,
                                        frc::Rotation2d::Degrees(180)));
  return RobotState::GetInstance().GetEstimatedPose().Nearest(
      {centerFace.TransformBy(frc::Transform2d(0, -l1AlignOffsetY.Get(),
                                               frc::Rotation2d::Degrees(0))),
       centerFace,
       centerFace.TransformBy(frc::Transform2d(0, l1AlignOffsetY.Get(),
                                               frc::Rotation2d::Degrees(0)))});
}

bool AutoScoreCommands::WithinDistanceToReef(frc::Pose2d robot,
                                             double distance) {
  const double distanceToReefCenter =
      AllianceFlipUtil::Apply(robot).Translation().Distance(
          FieldConstants::Reef::center);
  wpi::log::DataLogEntry(wpi::log::DoubleLog("AutoScore/DistanceToReefCenter"))
      .Append(distanceToReefCenter);
  return distanceToReefCenter <= FieldConstants::Reef::faceLength +
                                     DriveConstants::robotWidth / 2.0 +
                                     distance;
}

bool AutoScoreCommands::OutOfDistanceToReef(frc::Pose2d robot,
                                            double distance) {
  const double distanceToReefCenter =
      AllianceFlipUtil::Apply(robot).Translation().Distance(
          FieldConstants::Reef::center);
  wpi::log::DataLogEntry(wpi::log::DoubleLog("AutoScore/DistanceToReefCenter"))
      .Append(distanceToReefCenter);
  return distanceToReefCenter >= FieldConstants::Reef::faceLength +
                                     DriveConstants::robotWidth / 2.0 +
                                     distance;
}

frc::Pose2d
AutoScoreCommands::GetRobotPose(FieldConstants::CoralObjective coralObjective) {
  return RobotState::GetInstance().GetReefPose(
      coralObjective.branchId / 2, GetCoralScorePose(coralObjective));
}

frc::Pose2d
AutoScoreCommands::GetRobotPose(FieldConstants::AlgaeObjective algaeObjective) {
  return RobotState::GetInstance().GetReefPose(
      algaeObjective.id, GetReefIntakePose(algaeObjective));
}

frc::Pose2d
AutoScoreCommands::GetBranchPose(FieldConstants::CoralObjective objective) {
  return FieldConstants::Reef::branchPositions2d[objective.branchId]
                                                [objective.reefLevel];
}
