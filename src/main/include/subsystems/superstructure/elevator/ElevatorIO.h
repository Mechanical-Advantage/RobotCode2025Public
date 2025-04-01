// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "ElevatorIOInputsAutoLogged.h"

class ElevatorIO {
public:
  struct ElevatorIOData {
    bool motorConnected = false;
    bool followerConnected = false;
    double positionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double torqueCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double tempCelsius = 0.0;
    double followerAppliedVolts = 0.0;
    double followerTorqueCurrentAmps = 0.0;
    double followerSupplyCurrentAmps = 0.0;
    double followerTempCelsius = 0.0;
  };

  virtual ~ElevatorIO() = default;

  virtual void UpdateInputs(ElevatorIOInputsAutoLogged &inputs) {}

  virtual void RunOpenLoop(double output) {}

  virtual void RunVolts(double volts) {}

  virtual void Stop() {}

  virtual void RunPosition(double positionRad, double feedforward) {}

  virtual void SetPID(double kP, double kI, double kD) {}

  virtual void SetBrakeMode(bool enabled) {}
};