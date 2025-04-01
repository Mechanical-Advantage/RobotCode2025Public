// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/math/units/units.h>

#include "util/LoggedTunableNumber.h"

class SuperstructureConstants {
public:
  static constexpr double pivotToTunnelFront = units::inch_to_meter(4.55);
  static constexpr double G = 9.807;
  static constexpr double firstStageHeight = units::inch_to_meter(33.0);
  static constexpr double stageHeight = units::inch_to_meter(29.125);
  static constexpr double stageThickness = units::inch_to_meter(1.0);
  static constexpr double dispenserToTop = units::inch_to_meter(5.50);
  static constexpr double dispenserToBottom = units::inch_to_meter(6.50);
  static constexpr double stageToStage = units::inch_to_meter(4.0);

  static constexpr frc::Rotation2d elevatorAngle =
      frc::Rotation2d::Degrees(82.0);
  static constexpr frc::Translation2d superstructureOrigin2d =
      frc::Translation2d(0.0825, 0.029);
  static constexpr frc::Translation3d superstructureOrigin3d =
      frc::Translation3d(superstructureOrigin2d.X(), 0.0,
                         superstructureOrigin2d.Y());
  static constexpr frc::Translation2d dispenserOrigin2d =
      superstructureOrigin2d +
      frc::Translation2d(dispenserToBottom + stageThickness * 2, elevatorAngle);
  static constexpr frc::Translation3d dispenserOrigin3d =
      frc::Translation3d(dispenserOrigin2d.X(), 0.0, dispenserOrigin2d.Y());

  static constexpr double elevatorMaxTravel = 1.8;

  static constexpr double stage1ToStage2Height = 0.80;
  static constexpr double stage2ToStage3Height = 1.37;

  static constexpr frc::Rotation2d pivotSafeAngle =
      frc::Rotation2d::Degrees(-25.0);
  static LoggedTunableNumber throwHeight;
  static LoggedTunableNumber throwVelocity;

  static constexpr double chariotMaxExtension = 0.442630;
};