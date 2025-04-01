// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIOPigeon2.h"

#include <numeric>
#include <vector>

#include "frc/geometry/Rotation2d.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

GyroIOPigeon2::GyroIOPigeon2()
    : yawPositionQueue(
          PhoenixOdometryThread::GetInstance().RegisterSignal(pigeon.GetYaw())),
      yawTimestampQueue(
          PhoenixOdometryThread::GetInstance().MakeTimestampQueue()) {
  pigeon.GetConfigurator().Apply(
      ctre::phoenix6::configs::Pigeon2Configuration{});
  pigeon.GetConfigurator().SetYaw(0.0_deg);
  yaw.SetUpdateFrequency(DriveConstants::odometryFrequency);
  yawVelocity.SetUpdateFrequency(50.0);
  pigeon.OptimizeBusUtilization();
  PhoenixUtil::RegisterSignals(true, yaw, yawVelocity);
  PhoenixUtil::TryUntilOk(5, [&]() { return pigeon.SetYaw(0.0_deg, 0.25); });
}

void GyroIOPigeon2::UpdateInputs(GyroIOInputs &inputs) {
  inputs.data.connected =
      ctre::phoenix6::BaseStatusSignal::IsAllGood(yaw, yawVelocity);
  inputs.data.yawPosition = frc::Rotation2d(yaw.GetValue().value());
  inputs.data.yawVelocityRadPerSec = yawVelocity.GetValue().value();

  inputs.odometryYawTimestamps.resize(yawTimestampQueue->size());
  std::copy(yawTimestampQueue->begin(), yawTimestampQueue->end(),
            inputs.odometryYawTimestamps.begin());

  inputs.odometryYawPositions.resize(yawPositionQueue->size());
  std::transform(yawPositionQueue->begin(), yawPositionQueue->end(),
                 inputs.odometryYawPositions.begin(), [](double value) {
                   return frc::Rotation2d(units::degree_t(value));
                 });

  yawTimestampQueue->clear();
  yawPositionQueue->clear();
}