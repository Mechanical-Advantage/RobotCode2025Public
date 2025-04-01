// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <queue>

#include "frc/filter/Debouncer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/units/units.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

#include "reduxrobotics/sensors/canandgyro/Canandgyro.hpp"
#include "reduxrobotics/sensors/canandgyro/CanandgyroSettings.hpp"

class GyroIORedux : public GyroIO {
public:
  GyroIORedux();

  void UpdateInputs(GyroIOInputs &inputs) override;

private:
  reduxrobotics::sensors::canandgyro::Canandgyro gyro{30};
  std::queue<double> *yawTimestampQueue;
  std::queue<double> *yawPositionQueue;
  frc::Debouncer connectedDebouncer{0.5};
};