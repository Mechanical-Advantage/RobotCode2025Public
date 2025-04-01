// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <future>
#include <queue>
#include <vector>

#include "frc/filter/Debouncer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/units/units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/CANcoderConfiguration.hpp"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/hardware/CANcoder.hpp"
#include "ctre/phoenix6/hardware/ParentDevice.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/FeedbackSensorSourceValue.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"
#include "ctre/phoenix6/signals/SensorDirectionValue.hpp"

#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

class ModuleIOComp : public ModuleIO {
public:
  ModuleIOComp(DriveConstants::ModuleConfig config);

  void UpdateInputs(ModuleIOInputs &inputs) override;
  void RunDriveOpenLoop(double output) override;
  void RunTurnOpenLoop(double output) override;
  void RunDriveVelocity(double velocityRadPerSec, double feedforward) override;
  void RunTurnPosition(frc::Rotation2d rotation) override;
  void SetDrivePID(double kP, double kI, double kD) override;
  void SetTurnPID(double kP, double kI, double kD) override;
  void SetBrakeMode(bool enabled) override;

  static constexpr double driveReduction =
      (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  static constexpr double turnReduction = (150.0 / 7.0);

private:
  static constexpr double driveCurrentLimitAmps = 80;
  static constexpr double turnCurrentLimitAmps = 40;

  ctre::phoenix6::hardware::TalonFX driveTalon;
  ctre::phoenix6::hardware::TalonFX turnTalon;
  ctre::phoenix6::hardware::CANcoder encoder;

  ctre::phoenix6::configs::TalonFXConfiguration driveConfig;
  ctre::phoenix6::configs::TalonFXConfiguration turnConfig;
  frc::Rotation2d encoderOffset;

  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentRequest{0};
  ctre::phoenix6::controls::PositionTorqueCurrentFOC
      positionTorqueCurrentRequest{0.0};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC
      velocityTorqueCurrentRequest{0.0};

  ctre::phoenix6::signals::StatusSignal<units::radian_t> drivePosition;
  std::queue<double> *drivePositionQueue;
  ctre::phoenix6::signals::StatusSignal<units::radians_per_second_t>
      driveVelocity;
  ctre::phoenix6::signals::StatusSignal<units::volt_t> driveAppliedVolts;
  ctre::phoenix6::signals::StatusSignal<units::ampere_t> driveSupplyCurrentAmps;
  ctre::phoenix6::signals::StatusSignal<units::ampere_t> driveTorqueCurrentAmps;

  ctre::phoenix6::signals::StatusSignal<units::radian_t> turnAbsolutePosition;
  ctre::phoenix6::signals::StatusSignal<units::radian_t> turnPosition;
  std::queue<double> *turnPositionQueue;
  ctre::phoenix6::signals::StatusSignal<units::radians_per_second_t>
      turnVelocity;
  ctre::phoenix6::signals::StatusSignal<units::volt_t> turnAppliedVolts;
  ctre::phoenix6::signals::StatusSignal<units::ampere_t> turnSupplyCurrentAmps;
  ctre::phoenix6::signals::StatusSignal<units::ampere_t> turnTorqueCurrentAmps;

  frc::Debouncer driveConnectedDebounce{0.5};
  frc::Debouncer turnConnectedDebounce{0.5};
  frc::Debouncer turnEncoderConnectedDebounce{0.5};

  static std::future<void> brakeModeExecutor(std::function<void()> &&func);
};