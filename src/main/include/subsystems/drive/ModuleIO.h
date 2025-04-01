// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <vector>

#include "frc/geometry/Rotation2d.h"
#include "org/littletonrobotics/junction/AutoLog.h"

class ModuleIO {
public:
  struct ModuleIOInputs {
    struct ModuleIOData {
      bool driveConnected = false;
      double drivePositionRad = 0.0;
      double driveVelocityRadPerSec = 0.0;
      double driveAppliedVolts = 0.0;
      double driveSupplyCurrentAmps = 0.0;
      double driveTorqueCurrentAmps = 0.0;
      bool turnConnected = false;
      bool turnEncoderConnected = false;
      frc::Rotation2d turnAbsolutePosition = frc::Rotation2d();
      frc::Rotation2d turnPosition = frc::Rotation2d();
      double turnVelocityRadPerSec = 0.0;
      double turnAppliedVolts = 0.0;
      double turnSupplyCurrentAmps = 0.0;
      double turnTorqueCurrentAmps = 0.0;
    };

    ModuleIOData data;
    std::vector<double> odometryDrivePositionsRad;
    std::vector<frc::Rotation2d> odometryTurnPositions;

    AUTO_LOG_STRUCT(ModuleIOInputs);
  };

  virtual ~ModuleIO() = default;

  /** Updates the set of loggable inputs. */
  virtual void UpdateInputs(ModuleIOInputs &inputs) {}

  /** Run the drive motor at the specified open loop value. */
  virtual void RunDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  virtual void RunTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  virtual void RunDriveVelocity(double velocityRadPerSec, double feedforward) {}

  /** Run the turn motor to the specified rotation. */
  virtual void RunTurnPosition(frc::Rotation2d rotation) {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  virtual void SetDrivePID(double kP, double kI, double kD) {}

  /** Set P, I, and D gains for closed loop control on turn motor. */
  virtual void SetTurnPID(double kP, double kI, double kD) {}

  /** Set brake mode on drive motor */
  virtual void SetBrakeMode(bool enabled) {}
};