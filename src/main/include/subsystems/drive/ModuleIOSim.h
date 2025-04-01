// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/controller/PIDController.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"

#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIOComp.h"

class ModuleIOSim : public ModuleIO {
public:
  ModuleIOSim();

  void UpdateInputs(ModuleIOInputs &inputs) override;
  void RunDriveOpenLoop(double output) override;
  void RunTurnOpenLoop(double output) override;
  void RunDriveVelocity(double velocityRadPerSec, double feedforward) override;
  void RunTurnPosition(frc::Rotation2d rotation) override;
  void SetDrivePID(double kP, double kI, double kD) override;
  void SetTurnPID(double kP, double kI, double kD) override;

private:
  static constexpr frc::DCMotor driveMotorModel = frc::DCMotor::KrakenX60Foc(1);
  static constexpr frc::DCMotor turnMotorModel = frc::DCMotor::KrakenX60Foc(1);

  frc::sim::DCMotorSim driveSim = frc::sim::DCMotorSim(
      frc::LinearSystemId::IdentifyDCMotorSystem(driveMotorModel, 0.025,
                                                 ModuleIOComp::driveReduction),
      driveMotorModel);
  frc::sim::DCMotorSim turnSim = frc::sim::DCMotorSim(
      frc::LinearSystemId::IdentifyDCMotorSystem(turnMotorModel, 0.004,
                                                 ModuleIOComp::turnReduction),
      turnMotorModel);

  bool driveClosedLoop = false;
  bool turnClosedLoop = false;
  frc::PIDController driveController = frc::PIDController(0, 0, 0);
  frc::PIDController turnController = frc::PIDController(0, 0, 0);
  double driveFFVolts = 0;
  double driveAppliedVolts = 0.0;
  double turnAppliedVolts = 0.0;
};