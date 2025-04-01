// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIOSim.h"

#include "frc/MathUtil.h"
#include "frc/controller/PIDController.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"

#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIOComp.h"

ModuleIOSim::ModuleIOSim() {
  // Enable wrapping for turn PID
  turnController.EnableContinuousInput(-M_PI, M_PI);
}

void ModuleIOSim::UpdateInputs(ModuleIOInputs &inputs) {
  // Run closed-loop control
  if (driveClosedLoop) {
    driveAppliedVolts =
        driveFFVolts + driveController.Calculate(driveSim.GetAngularVelocity());
  } else {
    driveController.Reset();
  }
  if (turnClosedLoop) {
    turnAppliedVolts = turnController.Calculate(turnSim.GetAngularPosition());
  } else {
    turnController.Reset();
  }

  // Update simulation state
  driveSim.SetInputVoltage(
      frc::MathUtil::Clamp(driveAppliedVolts, -12.0, 12.0));
  turnSim.SetInputVoltage(frc::MathUtil::Clamp(turnAppliedVolts, -12.0, 12.0));
  driveSim.Update(Constants::loopPeriodSecs);
  turnSim.Update(Constants::loopPeriodSecs);

  // Update drive inputs
  inputs.data.driveConnected = true;
  inputs.data.drivePositionRad = driveSim.GetAngularPosition();
  inputs.data.driveVelocityRadPerSec = driveSim.GetAngularVelocity();
  inputs.data.driveAppliedVolts = driveAppliedVolts;
  inputs.data.driveSupplyCurrentAmps = std::abs(driveSim.GetCurrentDraw());
  inputs.data.driveTorqueCurrentAmps = 0.0;
  inputs.data.turnConnected = true;
  inputs.data.turnEncoderConnected = true;
  inputs.data.turnAbsolutePosition =
      frc::Rotation2d(turnSim.GetAngularPosition());
  inputs.data.turnPosition = frc::Rotation2d(turnSim.GetAngularPosition());
  inputs.data.turnVelocityRadPerSec = turnSim.GetAngularVelocity();
  inputs.data.turnAppliedVolts = turnAppliedVolts;
  inputs.data.turnSupplyCurrentAmps = std::abs(turnSim.GetCurrentDraw());
  inputs.data.turnTorqueCurrentAmps = 0.0;

  // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
  // matter)
  inputs.odometryDrivePositionsRad = {inputs.data.drivePositionRad};
  inputs.odometryTurnPositions = {inputs.data.turnPosition};
}

void ModuleIOSim::RunDriveOpenLoop(double output) {
  driveClosedLoop = false;
  driveAppliedVolts = output;
}

void ModuleIOSim::RunTurnOpenLoop(double output) {
  turnClosedLoop = false;
  turnAppliedVolts = output;
}

void ModuleIOSim::RunDriveVelocity(double velocityRadPerSec,
                                   double feedforward) {
  driveClosedLoop = true;
  driveFFVolts = feedforward;
  driveController.SetSetpoint(velocityRadPerSec);
}

void ModuleIOSim::RunTurnPosition(frc::Rotation2d rotation) {
  turnClosedLoop = true;
  turnController.SetSetpoint(rotation.Radians().value());
}

void ModuleIOSim::SetDrivePID(double kP, double kI, double kD) {
  driveController.SetPID(kP, kI, kD);
}

void ModuleIOSim::SetTurnPID(double kP, double kI, double kD) {
  turnController.SetPID(kP, kI, kD);
}