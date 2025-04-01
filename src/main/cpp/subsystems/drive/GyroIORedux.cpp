// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIORedux.h"

#include <numeric>
#include <vector>

#include "frc/geometry/Rotation2d.h"
#include "frc/units/units.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

GyroIORedux::GyroIORedux()
    : yawTimestampQueue(
          PhoenixOdometryThread::GetInstance().MakeTimestampQueue()),
      yawPositionQueue(PhoenixOdometryThread::GetInstance().RegisterSignal(
          std::bind(&reduxrobotics::sensors::canandgyro::Canandgyro::GetYaw,
                    &gyro))) {
  // Configure the gyro
  reduxrobotics::sensors::canandgyro::CanandgyroSettings settings;
  settings.AngularPositionFramePeriod = 1.0 / DriveConstants::odometryFrequency;
  settings.AngularVelocityFramePeriod = 0.01;
  gyro.SetSettings(settings, 0.25, 5);
  gyro.SetYaw(0.0, 0.1);
  gyro.ClearStickyFaults();
}

void GyroIORedux::UpdateInputs(GyroIOInputs &inputs) {
  inputs.data.connected = connectedDebouncer.Calculate(gyro.IsConnected());
  inputs.data.yawPosition = frc::Rotation2d(gyro.GetYaw());
  inputs.data.yawVelocityRadPerSec =
      frc::units::rotations_to_radians(gyro.GetAngularVelocityYaw());

  inputs.odometryYawTimestamps.resize(yawTimestampQueue->size());
  std::copy(yawTimestampQueue->begin(), yawTimestampQueue->end(),
            inputs.odometryYawTimestamps.begin());

  inputs.odometryYawPositions.resize(yawPositionQueue->size());
  std::transform(yawPositionQueue->begin(), yawPositionQueue->end(),
                 inputs.odometryYawPositions.begin(),
                 [](double value) { return frc::Rotation2d(value); });

  yawTimestampQueue->clear();
  yawPositionQueue->clear();
}