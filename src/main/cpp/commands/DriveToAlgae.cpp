// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "drivetoalgae.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "fieldconstants.h"
#include "junction/logger.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "subsystems/drive/driveconstants.h"
#include "util/alliancefliputil.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

static LoggedTunableNumber lookAheadSecs{"DriveToAlgae/LookAheadSecs", 0.3};
static LoggedTunableNumber angleDifferenceWeight{
    "DriveToAlgae/AngleDifferenceWeight", 0.3};
static LoggedTunableNumber algaeMaxDistance{"DriveToAlgae/AlgaeMaxDistance",
                                            1.4};
static LoggedTunableNumber algaeMaxAngleDeg{"DriveToAlgae/AlgaeMaxAngleDegrees",
                                            70.0};

DriveToAlgae::DriveToAlgae(Drive &drive, std::function<double()> driverX,
                           std::function<double()> driverY,
                           std::function<double()> driverOmega)
    : DriveToPose(
          drive,
          [&]() -> frc::Pose2d {
            RobotState &instance = RobotState::GetInstance();
            frc::Pose2d robot = instance.GetEstimatedPose();

            frc::ChassisSpeeds robotVelocity = instance.GetRobotVelocity();

            frc::Pose2d predictedRobot =
                robot.Exp(robotVelocity.ToTwist2d(lookAheadSecs.Get()));
            Logger::RecordOutput("DriveToAlgae/LookAheadPose", predictedRobot);

            const std::vector<frc::Translation2d> &algaeTranslations =
                instance.GetAlgaeTranslations();

            if (algaeTranslations.empty()) {
              Logger::RecordOutput("DriveToAlgae/TargetedAlgae",
                                   std::vector<frc::Translation2d>{});
              return RobotState::GetInstance().GetEstimatedPose();
            }

            auto minAlgae = std::min_element(
                algaeTranslations.begin(), algaeTranslations.end(),
                [&](const frc::Translation2d &a, const frc::Translation2d &b) {
                  double distanceA = a.Distance(predictedRobot.Translation()) +
                                     std::abs(a.Minus(robot.Translation())
                                                  .Angle()
                                                  .Minus(robot.Rotation())
                                                  .Radians() *
                                              angleDifferenceWeight.Get());
                  double distanceB = b.Distance(predictedRobot.Translation()) +
                                     std::abs(b.Minus(robot.Translation())
                                                  .Angle()
                                                  .Minus(robot.Rotation())
                                                  .Radians() *
                                              angleDifferenceWeight.Get());
                  return distanceA < distanceB;
                });

            if (minAlgae == algaeTranslations.end()) {
              Logger::RecordOutput("DriveToAlgae/TargetedAlgae",
                                   std::vector<frc::Translation2d>{});
              return RobotState::GetInstance().GetEstimatedPose();
            }

            if (minAlgae->Distance(predictedRobot.Translation()) <=
                    algaeMaxDistance.Get() &&
                std::abs(minAlgae->Minus(predictedRobot.Translation())
                             .Angle()
                             .Degrees()) <= algaeMaxAngleDeg.Get()) {
              Logger::RecordOutput("DriveToAlgae/TargetedAlgae",
                                   std::vector<frc::Translation2d>{*minAlgae});
              return frc::Pose2d(*minAlgae,
                                 robot.Translation().Minus(*minAlgae).Angle())
                  .TransformBy(
                      frc::Transform2d{FieldConstants::algaeDiameter / 2.0 +
                                           DriveConstants::robotWidth / 2.0,
                                       0.0, frc::Rotation2d{M_PI}});
            } else {
              Logger::RecordOutput("DriveToAlgae/TargetedAlgae",
                                   std::vector<frc::Translation2d>{});
              return RobotState::GetInstance().GetEstimatedPose();
            }
          },
          [&]() -> frc::Pose2d {
            return RobotState::GetInstance().GetEstimatedPose();
          },
          [&]() -> frc::ChassisSpeeds {
            return DriveCommands::GetLinearVelocityFromJoysticks(driverX(),
                                                                 driverY())
                .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
          },
          [&]() -> double {
            return DriveCommands::GetOmegaFromJoysticks(driverOmega());
          }) {}

} // namespace org::littletonrobotics::frc2025::commands