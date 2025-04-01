// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "commands/DriveToPose.h"
#include "subsystems/drive/Drive.h"
#include "util/LoggedTunableNumber.h"

namespace org::littletonrobotics::frc2025::commands {

class DriveToAlgae : public DriveToPose {
public:
  DriveToAlgae(Drive &drive, std::function<double()> driverX,
               std::function<double()> driverY,
               std::function<double()> driverOmega);

private:
  static util::LoggedTunableNumber lookAheadSecs;
  static util::LoggedTunableNumber angleDifferenceWeight;
  static util::LoggedTunableNumber algaeMaxDistance;
  static util::LoggedTunableNumber algaeMaxAngleDeg;
};

} // namespace org::littletonrobotics::frc2025::commands