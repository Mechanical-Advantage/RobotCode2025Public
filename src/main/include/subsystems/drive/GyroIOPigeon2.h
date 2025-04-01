// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <queue>

#include "frc/geometry/Rotation2d.h"
#include "frc/units/units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/Pigeon2Configuration.hpp"
#include "ctre/phoenix6/hardware/Pigeon2.hpp"

#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

/** IO implementation for Pigeon 2. */
class GyroIOPigeon2 : public GyroIO {
public:
  GyroIOPigeon2();

  void UpdateInputs(GyroIOInputs &inputs) override;

private:
  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveConstants::PigeonConstants::id,
                                           "*"};
  ctre::phoenix6::signals::StatusSignal<units::degree_t> yaw{pigeon.GetYaw()};
  std::queue<double> *yawPositionQueue;
  std::queue<double> *yawTimestampQueue;
  ctre::phoenix6::signals::StatusSignal<units::degrees_per_second_t>
      yawVelocity{pigeon.GetAngularVelocityZWorld()};
};