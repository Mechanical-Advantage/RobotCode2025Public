// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/math/units/units.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"

struct SuperstructurePose {
  std::function<double()> elevatorHeight;
  std::function<frc::Rotation2d()> pivotAngle;

  SuperstructurePose();
  SuperstructurePose(std::function<double()> elevatorHeight,
                     std::function<frc::Rotation2d()> pivotAngle);

  struct DispenserPose {
    enum class Preset { L1, L2, L3, L4 };

    DispenserPose(Preset preset);

    double GetElevatorHeight();
    double GetDispenserAngleDeg();
    frc::Transform2d ToRobotPose();

    static DispenserPose ForCoralScore(FieldConstants::ReefLevel reefLevel);

  private:
    static frc::Pose2d GetCoralScorePose(FieldConstants::ReefLevel reefLevel);

    frc::Pose2d pose;
  };

  enum class Preset {
    STOW,
    INTAKE,
    GOODBYE_CORAL,
    L1,
    L1_EJECT,
    L2,
    L3,
    L4,
    ALGAE_L2_INTAKE,
    ALGAE_L3_INTAKE,
    PRE_THROW,
    THROW,
    ALGAE_STOW
  };

  SuperstructurePose(Preset preset);

private:
  static LoggedTunableNumber intakeHeightBaseline;
  static LoggedTunableNumber intakeHeightRange;
  static LoggedTunableNumber intakeHeightTimeFactor;

  static std::map<FieldConstants::ReefLevel, LoggedTunableNumber> ejectDistance;
  static std::map<FieldConstants::ReefLevel, LoggedTunableNumber> ejectAngles;
  static std::map<FieldConstants::ReefLevel, LoggedTunableNumber> heightFudges;

  static void
  AddInitialValue(std::map<FieldConstants::ReefLevel, LoggedTunableNumber> &map,
                  FieldConstants::ReefLevel reefLevel, double initialValue,
                  const std::string &key);

  static SuperstructurePose CreateFromPreset(Preset preset);
};