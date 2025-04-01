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

#include "drivetopose.h"
#include "subsystems/drive/drive.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

class DriveToStation : public DriveToPose {
public:
  DriveToStation(Drive &drive, bool isAuto);
  DriveToStation(Drive &drive, std::function<double()> driverX,
                 std::function<double()> driverY,
                 std::function<double()> driverOmega, bool isAuto);
  DriveToStation(Drive &drive, std::function<frc::Translation2d()> linearFF,
                 std::function<double()> theta, bool isAuto);

private:
  static LoggedTunableNumber stationAlignDistance;
  static LoggedTunableNumber horizontalMaxOffset;
  static LoggedTunableNumber autoOffset;
};

} // namespace org::littletonrobotics::frc2025::commands