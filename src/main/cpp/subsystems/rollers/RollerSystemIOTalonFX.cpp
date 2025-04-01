// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIOTalonFX.h"

#include <thread>

#include "frc/math/Util.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/current.h"
#include "frc/units/temperature.h"
#include "frc/units/voltage.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

RollerSystemIOTalonFX::RollerSystemIOTalonFX(int id, const std::string &bus,
                                             int currentLimitAmps, bool invert,
                                             bool brake, double reduction)
    : talon(id, bus), position(talon.GetPosition()),
      velocity(talon.GetVelocity()), appliedVoltage(talon.GetMotorVoltage()),
      supplyCurrent(talon.GetSupplyCurrent()),
      torqueCurrent(talon.GetTorqueCurrent()),
      tempCelsius(talon.GetDeviceTemp()),
      tempFault(talon.GetFault_DeviceTemp()), config(), reduction(reduction) {
  config.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.MotorOutput.NeutralMode =
      brake ? ctre::phoenix6::signals::NeutralModeValue::Brake
            : ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.Feedback.VelocityFilterTimeConstant = 0.1;

  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(config); });

  PhoenixUtil::TryUntilOk(5, [&]() {
    return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent,
        tempCelsius, tempFault);
  });
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.OptimizeBusUtilization(0, 1.0); });

  PhoenixUtil::RegisterSignals(
      ctre::phoenix6::CANBus(bus).IsNetworkFD(), position, velocity,
      appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius, tempFault);
}

void RollerSystemIOTalonFX::UpdateInputs(RollerSystemIOInputs &inputs) {
  inputs.data.positionRads = frc::units::radian_t(
      frc::units::rotations_t(position.GetValueAsDouble()) / reduction);
  inputs.data.velocityRadsPerSec = frc::units::radians_per_second_t(
      frc::units::rotations_per_second_t(velocity.GetValueAsDouble()) /
      reduction);
  inputs.data.appliedVoltage =
      frc::units::volt_t(appliedVoltage.GetValueAsDouble());
  inputs.data.supplyCurrentAmps =
      frc::units::ampere_t(supplyCurrent.GetValueAsDouble());
  inputs.data.torqueCurrentAmps =
      frc::units::ampere_t(torqueCurrent.GetValueAsDouble());
  inputs.data.tempCelsius =
      frc::units::degree_celsius_t(tempCelsius.GetValueAsDouble());
  inputs.data.tempFault = tempFault.GetValue();
  inputs.data.connected =
      connectedDebouncer.Calculate(ctre::phoenix6::BaseStatusSignal::IsAllGood(
          position, velocity, appliedVoltage, supplyCurrent, torqueCurrent,
          tempCelsius, tempFault));
}

void RollerSystemIOTalonFX::RunVolts(double volts) {
  talon.SetControl(voltageOut.WithOutput(volts));
}

void RollerSystemIOTalonFX::Stop() { talon.SetControl(neutralOut); }

void RollerSystemIOTalonFX::SetCurrentLimit(double currentLimit) {
  std::thread([this, currentLimit]() {
    ctre::phoenix6::configs::TalonFXConfiguration tempConfig = config;
    tempConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    PhoenixUtil::TryUntilOk(
        5, [&]() { return talon.GetConfigurator().Apply(tempConfig); });
  }).detach();
}

void RollerSystemIOTalonFX::SetBrakeMode(bool enabled) {
  std::thread([this, enabled]() {
    PhoenixUtil::TryUntilOk(5, [&]() {
      return talon.SetNeutralMode(
          enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                  : ctre::phoenix6::signals::NeutralModeValue::Coast);
    });
  }).detach();
}