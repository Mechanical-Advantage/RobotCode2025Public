// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "drivetostation.h"

#include <cmath>
#include <functional>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/util/Units.h>

#include "drivecommands.h"
#include "fieldconstants.h"
#include "junction/logger.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "subsystems/drive/driveconstants.h"
#include "util/alliancefliputil.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

LoggedTunableNumber DriveToStation::stationAlignDistance{
    "DriveToStation/StationAlignDistance",
    DriveConstants::robotWidth / 2.0 + frc::Units::InchesToMeters(5.0)};
LoggedTunableNumber DriveToStation::horizontalMaxOffset{
    "DriveToStation/HorizontalMaxOffset",
    FieldConstants::CoralStation::stationLength / 2 -
        frc::Units::InchesToMeters(32)};
LoggedTunableNumber DriveToStation::autoOffset{
    "DriveToStation/AutoOffset",
    FieldConstants::CoralStation::stationLength / 2 -
        frc::Units::InchesToMeters(24)};

DriveToStation::DriveToStation(Drive &drive, bool isAuto)
    : DriveToStation{drive, []() { return 0.0; }, []() { return 0.0; },
                     []() { return 0.0; }, isAuto} {}

DriveToStation::DriveToStation(Drive &drive, std::function<double()> driverX,
                               std::function<double()> driverY,
                               std::function<double()> driverOmega, bool isAuto)
    : DriveToStation{drive,
                     [&]() {
                       return DriveCommands::GetLinearVelocityFromJoysticks(
                                  driverX(), driverY())
                           .Times(AllianceFlipUtil::ShouldFlip() ? -1.0 : 1.0);
                     },
                     [&]() {
                       return std::copysign(
                           std::pow(MathUtil::ApplyDeadband(
                                        driverOmega(), DriveCommands::DEADBAND),
                                    2.0),
                           driverOmega());
                     },
                     isAuto} {}

DriveToStation::DriveToStation(Drive &drive,
                               std::function<frc::Translation2d()> linearFF,
                               std::function<double()> theta, bool isAuto)
    : DriveToPose{
          drive,
          [&, isAuto]() {
            frc::Pose2d curPose = AllianceFlipUtil::Apply(
                RobotState::GetInstance().GetEstimatedPose());

            std::vector<frc::Pose2d> finalPoses;
            for (const frc::Pose2d &stationCenter :
                 {FieldConstants::CoralStation::leftCenterFace,
                  FieldConstants::CoralStation::rightCenterFace}) {
              frc::Transform2d offset{stationCenter, curPose};
              offset = frc::Transform2d{
                  stationAlignDistance.Get(),
                  isAuto
                      ? (curPose.Y() < FieldConstants::fieldWidth / 2.0
                             ? -autoOffset.Get()
                             : autoOffset.Get())
                      : MathUtil::Clamp(offset.Y(), -horizontalMaxOffset.Get(),
                                        horizontalMaxOffset.Get()),
                  frc::Rotation2d{}};

              frc::Rotation2d rotationOffset =
                  curPose.Rotation().Minus(stationCenter.Rotation());
              if (std::abs(rotationOffset.Degrees()) > 45 && !isAuto) {
                finalPoses.emplace_back(
                    stationCenter.TransformBy(offset).Translation(),
                    stationCenter.Rotation().RotateBy(frc::Rotation2d::Degrees(
                        std::copysign(90.0, rotationOffset.Degrees()))));
              } else {
                finalPoses.emplace_back(stationCenter.TransformBy(offset));
              }
            }
            Logger::RecordOutput("DriveToStation/LeftClosestPose",
                                 AllianceFlipUtil::Apply(finalPoses[0]));
            Logger::RecordOutput("DriveToStation/RightClosestPose",
                                 AllianceFlipUtil::Apply(finalPoses[1]));
            return AllianceFlipUtil::Apply(curPose.Nearest(finalPoses));
          },
          [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
          linearFF, theta} {}

} // namespace org::littletonrobotics::frc2025::commands