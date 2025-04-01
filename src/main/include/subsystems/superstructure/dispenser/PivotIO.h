// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/geometry/Rotation2d.h"
#include "org/littletonrobotics/junction/AutoLog.h"

class PivotIO {
public:
  struct PivotIOInputs {
    struct PivotIOData {
      bool motorConnected = false;
      frc::Rotation2d position = frc::Rotation2d::Zero();
      double velocityRadPerSec = 0.0;
      double appliedVolts = 0.0;
      double supplyCurrentAmps = 0.0;
      double torqueCurrentAmps = 0.0;
      double tempCelsius = 0.0;
    };
    PivotIOData data;
  };

  virtual ~PivotIO() = default;

  virtual void UpdateInputs(PivotIOInputs &inputs) {}

  virtual void RunOpenLoop(double output) {}

  virtual void RunVolts(double volts) {}

  virtual void Stop() {}

  virtual void RunPosition(const frc::Rotation2d &position,
                           double feedforward) {}

  virtual void SetPID(double kP, double kI, double kD) {}

  virtual void SetBrakeMode(bool enabled) {}
};