// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <cmath>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/units/units.h"

#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/util/swerve/ModuleLimits.h"

struct DriveConstants {
  static constexpr double odometryFrequency = 250;
  static constexpr double trackWidthX =
      org::littletonrobotics::frc2025::Constants::GetRobot() ==
              org::littletonrobotics::frc2025::Constants::RobotType::DEVBOT
          ? frc::units::inch_to_meter(20.75)
          : frc::units::inch_to_meter(22.75);
  static constexpr double trackWidthY =
      org::littletonrobotics::frc2025::Constants::GetRobot() ==
              org::littletonrobotics::frc2025::Constants::RobotType::DEVBOT
          ? frc::units::inch_to_meter(20.75)
          : frc::units::inch_to_meter(22.75);
  static constexpr double driveBaseRadius =
      hypot(trackWidthX / 2, trackWidthY / 2);
  static constexpr double maxLinearSpeed = 4.69;
  static constexpr double maxAngularSpeed = 4.69 / driveBaseRadius;

  /** Includes bumpers! */
  static constexpr double robotWidth =
      frc::units::inch_to_meter(28.0) + 2 * frc::units::inch_to_meter(2.0);

  static constexpr std::array<frc::Translation2d, 4> moduleTranslations = {
      frc::Translation2d(trackWidthX / 2, trackWidthY / 2),
      frc::Translation2d(trackWidthX / 2, -trackWidthY / 2),
      frc::Translation2d(-trackWidthX / 2, trackWidthY / 2),
      frc::Translation2d(-trackWidthX / 2, -trackWidthY / 2)};

  static constexpr double wheelRadius = frc::units::inch_to_meter(2.000);

  static constexpr ModuleLimits moduleLimitsFree = ModuleLimits(
      maxLinearSpeed, maxAngularSpeed, frc::units::degrees_to_radians(1080.0));

  struct ModuleConfig {
    int driveMotorId;
    int turnMotorId;
    int encoderChannel;
    frc::Rotation2d encoderOffset;
    bool turnInverted;
    bool encoderInverted;
  };

  static constexpr std::array<ModuleConfig, 4> moduleConfigsComp = {
      {// FL
       {16, 15, 41, frc::Rotation2d(2.5356702423749646), true, false},
       // FR
       {10, 11, 42, frc::Rotation2d(-1.872990542008368), true, false},
       // BL
       {18, 19, 43, frc::Rotation2d(0.6458059116998549), true, false},
       // BR
       {13, 14, 44, frc::Rotation2d(-2.5187964537082226), true, false}}};

  static constexpr std::array<ModuleConfig, 4> moduleConfigsDev = {
      {// FL
       {12, 9, 1, frc::Rotation2d(-0.009115335014721037), true, false},
       // FR
       {2, 10, 3, frc::Rotation2d(0.8427416931125384), true, false},
       // BL
       {15, 11, 0, frc::Rotation2d(-1.0620197413817225), true, false},
       // BR
       {3, 8, 2, frc::Rotation2d(-2.600063124240756), true, false}}};

  struct PigeonConstants {
    static constexpr int id =
        org::littletonrobotics::frc2025::Constants::GetRobot() ==
                org::littletonrobotics::frc2025::Constants::RobotType::DEVBOT
            ? 3
            : 30;
  };
};