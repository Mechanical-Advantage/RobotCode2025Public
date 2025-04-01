// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/PivotIOTalonFX.h"

#include "frc/filter/Debouncer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/current.h"
#include "frc/units/temperature.h"
#include "frc/units/voltage.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"

namespace org::littletonrobotics::frc2025::subsystems::superstructure::
    dispenser {

PivotIOTalonFX::PivotIOTalonFX() : talon(5) {
  Config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  Config.Slot0 =
      ctre::phoenix6::configs::Slot0Configs().WithKP(0).WithKI(0).WithKD(0);
  Config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  Config.Feedback.SensorToMechanismRatio = reduction;
  Config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
  Config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
  Config.CurrentLimits.StatorCurrentLimit = 40.0;
  Config.CurrentLimits.StatorCurrentLimitEnable = true;
  Config.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(Config, 0.25); });

  internalPosition = talon.GetPosition();
  internalVelocity = talon.GetVelocity();
  appliedVolts = talon.GetMotorVoltage();
  supplyCurrentAmps = talon.GetSupplyCurrent();
  torqueCurrentAmps = talon.GetTorqueCurrent();
  temp = talon.GetDeviceTemp();

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0, internalPosition, internalVelocity, appliedVolts, supplyCurrentAmps,
      torqueCurrentAmps, temp);
  talon.OptimizeBusUtilization();

  PhoenixUtil::RegisterSignals(false, internalPosition, internalVelocity,
                               appliedVolts, supplyCurrentAmps,
                               torqueCurrentAmps, temp);
}

void PivotIOTalonFX::UpdateInputs(PivotIOInputs &inputs) {
  inputs.data.motorConnected = motorConnectedDebouncer.Calculate(
      ctre::phoenix6::BaseStatusSignal::IsAllGood(
          internalPosition, internalVelocity, appliedVolts, supplyCurrentAmps,
          torqueCurrentAmps, temp));
  inputs.data.position = frc::Rotation2d(internalPosition.GetValue().value());
  inputs.data.velocityRadPerSec = internalVelocity.GetValue().value();
  inputs.data.appliedVolts = appliedVolts.GetValue().value();
  inputs.data.supplyCurrentAmps = supplyCurrentAmps.GetValue().value();
  inputs.data.torqueCurrentAmps = torqueCurrentAmps.GetValue().value();
  inputs.data.tempCelsius = temp.GetValue().value();
}

void PivotIOTalonFX::RunOpenLoop(double output) {
  talon.SetControl(torqueCurrentFOC.WithOutput(output));
}

void PivotIOTalonFX::RunVolts(double volts) {
  talon.SetControl(voltageRequest.WithOutput(volts));
}

void PivotIOTalonFX::Stop() { talon.StopMotor(); }

void PivotIOTalonFX::RunPosition(const frc::Rotation2d &position,
                                 double feedforward) {
  talon.SetControl(positionTorqueCurrentFOC.WithPosition(position.Radians())
                       .WithFeedForward(feedforward));
}

void PivotIOTalonFX::SetPID(double kP, double kI, double kD) {
  Config.Slot0.kP = kP;
  Config.Slot0.kI = kI;
  Config.Slot0.kD = kD;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(Config); });
}

void PivotIOTalonFX::SetBrakeMode(bool enabled) {
  std::thread([this, enabled]() {
    Config.MotorOutput.NeutralMode =
        enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                : ctre::phoenix6::signals::NeutralModeValue::Coast;
    PhoenixUtil::TryUntilOk(
        5, [&]() { return talon.GetConfigurator().Apply(Config); });
  }).detach();
}

} // namespace
  // org::littletonrobotics::frc2025::subsystems::superstructure::dispenser