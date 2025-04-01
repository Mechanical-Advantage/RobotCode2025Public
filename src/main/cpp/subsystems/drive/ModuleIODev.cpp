// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIODev.h"

#include <future>
#include <mutex>
#include <numeric>
#include <vector>

#include "frc/units/units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"

ModuleIODev::ModuleIODev(DriveConstants::ModuleConfig config)
    : driveTalon(config.driveMotorId, "*"), turnTalon(config.turnMotorId, "*"),
      encoder(config.encoderChannel), encoderOffset(config.encoderOffset),
      drivePositionQueue(PhoenixOdometryThread::GetInstance().RegisterSignal(
          driveTalon.GetPosition())),
      turnPositionQueue(PhoenixOdometryThread::GetInstance().RegisterSignal(
          turnTalon.GetPosition())) {
  // Configure drive motor
  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Slot0 =
      ctre::phoenix6::configs::Slot0Configs().withKP(0).withKI(0).withKD(0);
  driveConfig.Feedback.SensorToMechanismRatio = driveReduction;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveCurrentLimitAmps;
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveCurrentLimitAmps;
  driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimitAmps;
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
  PhoenixUtil::TryUntilOk(5, [&]() {
    return driveTalon.GetConfigurator().Apply(driveConfig, 0.25);
  });
  PhoenixUtil::TryUntilOk(5,
                          [&]() { return driveTalon.SetPosition(0.0, 0.25); });

  // Configure turn motor
  turnConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  turnConfig.Slot0 =
      ctre::phoenix6::configs::Slot0Configs().withKP(0).withKI(0).withKD(0);
  turnConfig.Feedback.SensorToMechanismRatio = turnReduction;
  turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
  turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnCurrentLimitAmps;
  turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnCurrentLimitAmps;
  turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimitAmps;
  turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  turnConfig.MotorOutput.Inverted =
      config.turnInverted
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return turnTalon.GetConfigurator().Apply(turnConfig, 0.25); });

  // Configure absolute encoder and set position on turn talon
  turnAbsolutePosition = [&]() {
    return frc::Rotation2d(encoder.GetVoltage() /
                           frc::RobotController::GetVoltage5V() * 2.0 * M_PI) +
           encoderOffset;
  };
  PhoenixUtil::TryUntilOk(5, [&]() {
    return turnTalon.SetPosition(turnAbsolutePosition().Radians().value(),
                                 0.25);
  });

  // Create drive status signals
  drivePosition = driveTalon.GetPosition();
  driveVelocity = driveTalon.GetVelocity();
  driveAppliedVolts = driveTalon.GetMotorVoltage();
  driveSupplyCurrentAmps = driveTalon.GetSupplyCurrent();
  driveTorqueCurrentAmps = driveTalon.GetTorqueCurrent();

  // Create turn status signals
  turnPosition = turnTalon.GetPosition();
  turnVelocity = turnTalon.GetVelocity();
  turnAppliedVolts = turnTalon.GetMotorVoltage();
  turnSupplyCurrentAmps = turnTalon.GetSupplyCurrent();
  turnTorqueCurrentAmps = turnTalon.GetTorqueCurrent();

  // Configure periodic frames
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      DriveConstants::odometryFrequency, drivePosition, turnPosition);
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0, driveVelocity, driveAppliedVolts, driveSupplyCurrentAmps,
      driveTorqueCurrentAmps, turnVelocity, turnAppliedVolts,
      turnSupplyCurrentAmps, turnTorqueCurrentAmps);
  ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
      driveTalon, turnTalon);

  // Register signals for refresh
  PhoenixUtil::RegisterSignals(true, drivePosition, driveVelocity,
                               driveAppliedVolts, driveSupplyCurrentAmps,
                               driveTorqueCurrentAmps, turnPosition,
                               turnVelocity, turnAppliedVolts,
                               turnSupplyCurrentAmps, turnTorqueCurrentAmps);
}

void ModuleIODev::UpdateInputs(ModuleIOInputs &inputs) {
  // Update drive inputs
  inputs.data.driveConnected = driveConnectedDebounce.Calculate(
      ctre::phoenix6::BaseStatusSignal::IsAllGood(
          drivePosition, driveVelocity, driveAppliedVolts,
          driveSupplyCurrentAmps, driveTorqueCurrentAmps));
  inputs.data.drivePositionRad = drivePosition.GetValue().value();
  inputs.data.driveVelocityRadPerSec = driveVelocity.GetValue().value();
  inputs.data.driveAppliedVolts = driveAppliedVolts.GetValue().value();
  inputs.data.driveSupplyCurrentAmps =
      driveSupplyCurrentAmps.GetValue().value();
  inputs.data.driveTorqueCurrentAmps =
      driveTorqueCurrentAmps.GetValue().value();
  inputs.data.turnConnected = turnConnectedDebounce.Calculate(
      ctre::phoenix6::BaseStatusSignal::IsAllGood(
          turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrentAmps,
          turnTorqueCurrentAmps));
  inputs.data.turnEncoderConnected = true;
  inputs.data.turnAbsolutePosition = turnAbsolutePosition() - encoderOffset;
  inputs.data.turnPosition = frc::Rotation2d(turnPosition.GetValue().value());
  inputs.data.turnVelocityRadPerSec = turnVelocity.GetValue().value();
  inputs.data.turnAppliedVolts = turnAppliedVolts.GetValue().value();
  inputs.data.turnSupplyCurrentAmps = turnSupplyCurrentAmps.GetValue().value();
  inputs.data.turnTorqueCurrentAmps = turnTorqueCurrentAmps.GetValue().value();

  // Update odometry inputs
  inputs.odometryDrivePositionsRad.resize(drivePositionQueue->size());
  std::transform(drivePositionQueue->begin(), drivePositionQueue->end(),
                 inputs.odometryDrivePositionsRad.begin(),
                 [](double value) { return value; });

  inputs.odometryTurnPositions.resize(turnPositionQueue->size());
  std::transform(turnPositionQueue->begin(), turnPositionQueue->end(),
                 inputs.odometryTurnPositions.begin(),
                 [](double value) { return frc::Rotation2d(value); });

  drivePositionQueue->clear();
  turnPositionQueue->clear();
}

void ModuleIODev::RunDriveOpenLoop(double output) {
  driveTalon.SetControl(torqueCurrentRequest.WithOutput(output));
}

void ModuleIODev::RunTurnOpenLoop(double output) {
  turnTalon.SetControl(torqueCurrentRequest.WithOutput(output));
}

void ModuleIODev::RunDriveVelocity(double velocityRadPerSec,
                                   double feedforward) {
  driveTalon.SetControl(
      velocityTorqueCurrentRequest.WithVelocity(velocityRadPerSec)
          .WithFeedForward(feedforward));
}

void ModuleIODev::RunTurnPosition(frc::Rotation2d rotation) {
  turnTalon.SetControl(
      positionTorqueCurrentRequest.WithPosition(rotation.Radians().value()));
}

void ModuleIODev::SetDrivePID(double kP, double kI, double kD) {
  driveConfig.Slot0.kP = kP;
  driveConfig.Slot0.kI = kI;
  driveConfig.Slot0.kD = kD;
  PhoenixUtil::TryUntilOk(5, [&]() {
    return driveTalon.GetConfigurator().Apply(driveConfig, 0.25);
  });
}

void ModuleIODev::SetTurnPID(double kP, double kI, double kD) {
  turnConfig.Slot0.kP = kP;
  turnConfig.Slot0.kI = kI;
  turnConfig.Slot0.kD = kD;
  PhoenixUtil::TryUntilOk(
      5, [&]() { return turnTalon.GetConfigurator().Apply(turnConfig, 0.25); });
}

std::future<void> ModuleIODev::brakeModeExecutor(std::function<void()> &&func) {
  return std::async(std::launch::async, std::move(func));
}

void ModuleIODev::SetBrakeMode(bool enabled) {
  brakeModeExecutor([&, enabled]() {
    std::lock_guard<std::mutex> lock(driveConfigMutex);
    driveConfig.MotorOutput.NeutralMode =
        enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                : ctre::phoenix6::signals::NeutralModeValue::Coast;
    PhoenixUtil::TryUntilOk(5, [&]() {
      return driveTalon.GetConfigurator().Apply(driveConfig, 0.25);
    });
  });

  brakeModeExecutor([&, enabled]() {
    std::lock_guard<std::mutex> lock(turnConfigMutex);
    turnConfig.MotorOutput.NeutralMode =
        enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                : ctre::phoenix6::signals::NeutralModeValue::Coast;
    PhoenixUtil::TryUntilOk(5, [&]() {
      return turnTalon.GetConfigurator().Apply(turnConfig, 0.25);
    });
  });
}
