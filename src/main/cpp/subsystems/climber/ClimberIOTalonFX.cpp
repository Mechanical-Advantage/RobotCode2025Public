// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "ClimberIOTalonFX.h"
#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"
#include "frc/MathUtil.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/temperature.h"
#include "units/voltage.h"
#include <thread>

ClimberIOTalonFX::ClimberIOTalonFX() : talon(1) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.Feedback.SensorToMechanismRatio = reduction;
  config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
  config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
  config.CurrentLimits.StatorCurrentLimit = 120.0;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      units::degree_t(frc::DegreesToRadians(245.0));
  config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

  org::littletonrobotics::frc2025::util::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(config, 0.25); });
  org::littletonrobotics::frc2025::util::TryUntilOk(
      5, [&]() { return talon.SetPosition(0.0); });

  position = talon.GetPosition();
  velocity = talon.GetVelocity();
  appliedVolts = talon.GetMotorVoltage();
  supplyCurrentAmps = talon.GetSupplyCurrent();
  torqueCurrentAmps = talon.GetTorqueCurrent();
  temp = talon.GetDeviceTemp();

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0, position, velocity, appliedVolts, supplyCurrentAmps,
      torqueCurrentAmps, temp);
  talon.OptimizeBusUtilization();

  org::littletonrobotics::frc2025::util::RegisterSignals(
      false, position, velocity, appliedVolts, supplyCurrentAmps,
      torqueCurrentAmps, temp);
}

void ClimberIOTalonFX::UpdateInputs(ClimberIOInputs &inputs) {
  inputs.data.motorConnected = motorConnectedDebouncer.Calculate(
      ctre::phoenix6::BaseStatusSignal::IsAllGood(
          position, velocity, appliedVolts, supplyCurrentAmps, temp));
  inputs.data.positionRads = position.GetValue();
  inputs.data.velocityRadsPerSec = velocity.GetValue();
  inputs.data.appliedVoltage = appliedVolts.GetValue();
  inputs.data.torqueCurrentAmps = torqueCurrentAmps.GetValue();
  inputs.data.supplyCurrentAmps = supplyCurrentAmps.GetValue();
  inputs.data.tempCelsius = temp.GetValue();
}

void ClimberIOTalonFX::RunTorqueCurrent(double current) {
  talon.SetControl(torqueCurrentRequest.WithOutput(current));
}

void ClimberIOTalonFX::Stop() { talon.StopMotor(); }

void ClimberIOTalonFX::SetBrakeMode(bool enabled) {
  std::thread([this, enabled]() {
    org::littletonrobotics::frc2025::util::TryUntilOk(5, [&]() {
      return talon.SetNeutralMode(
          enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                  : ctre::phoenix6::signals::NeutralModeValue::Coast,
          0.25);
    });
  }).detach();
}