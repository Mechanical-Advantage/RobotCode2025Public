// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cmath>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/math/units/units.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "thirdparty/junction/Logger.h"
#include "thirdparty/junction/mechanism/LoggedMechanism2d.h"
#include "thirdparty/junction/mechanism/LoggedMechanismLigament2d.h"
#include "thirdparty/junction/mechanism/LoggedMechanismRoot2d.h"
#include "util/EqualsUtil.h"

class SuperstructureVisualizer {
public:
  SuperstructureVisualizer(const std::string &name);

  void Update(double elevatorHeightMeters, frc::Rotation2d pivotFinalAngle,
              double algaeIntakePosition, bool hasAlgae);

  static void UpdateSimIntake(double angleRad);

private:
  static constexpr double modelToRealDispenserRotation = 5.0;
  static constexpr frc::Translation3d intakeOrigin3d = {0.341630, 0.0,
                                                        0.287234};
  static constexpr double intakeAngleDeg = 14.010320;

  std::string name;
  LoggedMechanism2d mechanism;
  LoggedMechanismLigament2d elevatorMechanism;
  LoggedMechanismLigament2d pivotMechanism;
};