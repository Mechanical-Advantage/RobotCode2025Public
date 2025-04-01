// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "ElevatorIOTalonFX.h"

#include <thread>

#include <frc/math/units/units.h>

#include "Constants.h"
#include "util/PhoenixUtil.h"

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/configs/TalonFXConfiguration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/hardware/TalonFX.hpp>
#include <ctre/phoenix6/signals/InvertedValue.hpp>
#include <ctre/phoenix6/signals/NeutralModeValue.hpp>

ElevatorIOTalonFX::ElevatorIOTalonFX()
    : talon(Constants::GetRobot() == RobotType::DEVBOT ? 13 : 17, "*"),
      followerTalon(Constants::GetRobot() == RobotType::DEVBOT ? 14 : 9, "*") {
  followerTalon.SetControl(
      ctre::phoenix6::controls::Follower(talon.GetDeviceID(), true));

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Slot0 =
      ctre::phoenix6::configs::Slot0Configs().WithKP(0).WithKI(0).WithKD(0);
  config.Feedback.SensorToMechanismRatio = reduction;
  config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
  config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
  config.CurrentLimits.StatorCurrentLimit = 80.0;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(config, 0.25); });

  position = talon.GetPosition();
  velocity = talon.GetVelocity();
  appliedVolts = talon.GetMotorVoltage();
  torqueCurrent = talon.GetTorqueCurrent();
  supplyCurrent = talon.GetSupplyCurrent();
  temp = talon.GetDeviceTemp();
  followerAppliedVolts = followerTalon.GetMotorVoltage();
  followerTorqueCurrent = followerTalon.GetTorqueCurrent();
  followerSupplyCurrent = followerTalon.GetSupplyCurrent();
  followerTemp = followerTalon.GetDeviceTemp();

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0_Hz, position, velocity, appliedVolts, supplyCurrent, temp,
      followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent,
      followerTemp);
  torqueCurrent.SetUpdateFrequency(1000.0_Hz);
  ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
      talon, followerTalon);

  PhoenixUtil::RegisterSignals(true, position, velocity, appliedVolts,
                               torqueCurrent, supplyCurrent, temp,
                               followerAppliedVolts, followerTorqueCurrent,
                               followerSupplyCurrent, followerTemp);
}

void ElevatorIOTalonFX::UpdateInputs(ElevatorIOInputsAutoLogged &inputs) {
  inputs.data.motorConnected =
      connectedDebouncer.Calculate(ctre::phoenix6::BaseStatusSignal::IsAllGood(
          position, velocity, appliedVolts, torqueCurrent, supplyCurrent,
          temp));
  inputs.data.followerConnected =
      connectedDebouncer.Calculate(ctre::phoenix6::BaseStatusSignal::IsAllGood(
          followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent,
          followerTemp));
  inputs.data.positionRad = position.GetValue();
  inputs.data.velocityRadPerSec = velocity.GetValue();
  inputs.data.appliedVolts = appliedVolts.GetValue();
  inputs.data.torqueCurrentAmps = torqueCurrent.GetValue();
  inputs.data.supplyCurrentAmps = supplyCurrent.GetValue();
  inputs.data.tempCelsius = temp.GetValue();
  inputs.data.followerAppliedVolts = followerAppliedVolts.GetValue();
  inputs.data.followerTorqueCurrentAmps = followerTorqueCurrent.GetValue();
  inputs.data.followerSupplyCurrentAmps = followerSupplyCurrent.GetValue();
  inputs.data.followerTempCelsius = followerTemp.GetValue();
}

void ElevatorIOTalonFX::RunOpenLoop(double output) {
  talon.SetControl(torqueCurrentRequest.WithOutput(output));
}

void ElevatorIOTalonFX::RunVolts(double volts) {
  talon.SetControl(voltageRequest.WithOutput(volts));
}

void ElevatorIOTalonFX::Stop() { talon.StopMotor(); }

void ElevatorIOTalonFX::RunPosition(double positionRad, double feedforward) {
  talon.SetControl(positionTorqueCurrentRequest.WithPosition(positionRad)
                       .WithFeedForward(feedforward));
}

void ElevatorIOTalonFX::SetPID(double kP, double kI, double kD) {
  config.Slot0.kP = kP;
  config.Slot0.kI = kI;
  config.Slot0.kD = kD;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return talon.GetConfigurator().Apply(config); });
}

void ElevatorIOTalonFX::SetBrakeMode(bool enabled) {
  std::thread([this, enabled]() {
    talon.SetNeutralMode(
        enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                : ctre::phoenix6::signals::NeutralModeValue::Coast);
  }).detach();
}