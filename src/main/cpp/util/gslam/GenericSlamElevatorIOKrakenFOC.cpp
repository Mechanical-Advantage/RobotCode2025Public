// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/gslam/GenericSlamElevatorIOKrakenFOC.h"

#include <frc/units/angle.h>
#include <frc/units/angular_velocity.h>
#include <frc/units/current.h>
#include <frc/units/temperature.h>
#include <frc/units/voltage.h>

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/configs/TalonFXConfiguration.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/hardware/TalonFX.hpp>
#include <ctre/phoenix6/signals/InvertedValue.hpp>
#include <ctre/phoenix6/signals/NeutralModeValue.hpp>

#include "util/PhoenixUtil.h"

GenericSlamElevatorIOKrakenFOC::GenericSlamElevatorIOKrakenFOC(
    int id, const std::string &bus, int currentLimitAmps, bool invert,
    double reduction)
    : talon(id, bus), position(talon.GetPosition()),
      velocity(talon.GetVelocity()), appliedVoltage(talon.GetMotorVoltage()),
      supplyCurrent(talon.GetSupplyCurrent()),
      torqueCurrent(talon.GetTorqueCurrent()), temp(talon.GetDeviceTemp()) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.Feedback.SensorToMechanismRatio = reduction;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(config); });

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent,
      temp);
  talon.OptimizeBusUtilization(0, 1.0);

  PhoenixUtil::RegisterSignals(ctre::phoenix6::CANBus(bus).IsNetworkFD(),
                               position, velocity, appliedVoltage,
                               supplyCurrent, torqueCurrent, temp);
}

void GenericSlamElevatorIOKrakenFOC::UpdateInputs(
    GenericSlamElevatorIOInputs &inputs) {
  inputs.data =
      GenericSlamElevatorIOData{ctre::phoenix6::BaseStatusSignal::IsAllGood(
                                    position, velocity, appliedVoltage,
                                    supplyCurrent, torqueCurrent, temp),
                                position.GetValue().value(),
                                velocity.GetValue().value(),
                                appliedVoltage.GetValue().value(),
                                supplyCurrent.GetValue().value(),
                                torqueCurrent.GetValue().value(),
                                temp.GetValue().value()};
}

void GenericSlamElevatorIOKrakenFOC::RunCurrent(double amps) {
  talon.SetControl(currentControl.WithOutput(amps));
}

void GenericSlamElevatorIOKrakenFOC::Stop() { talon.SetControl(neutralOut); }

void GenericSlamElevatorIOKrakenFOC::SetBrakeMode(bool enable) {
  talon.SetNeutralMode(enable
                           ? ctre::phoenix6::signals::NeutralModeValue::Brake
                           : ctre::phoenix6::signals::NeutralModeValue::Coast);
}