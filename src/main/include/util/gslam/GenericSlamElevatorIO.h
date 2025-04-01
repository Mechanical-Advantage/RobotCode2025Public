// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "thirdparty/junction/AutoLog.h"

class GenericSlamElevatorIO {
public:
  struct GenericSlamElevatorIOInputs {
    JUnction_AUTO_LOG_VARS() GenericSlamElevatorIOData data =
        GenericSlamElevatorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  };

  struct GenericSlamElevatorIOData {
    JUnction_AUTO_LOG_VARS() bool motorConnected;
    double positionRads;
    double velocityRadsPerSec;
    double appliedVoltage;
    double supplyCurrentAmps;
    double torqueCurrentAmps;
    double tempCelsius;
  };

  virtual void UpdateInputs(GenericSlamElevatorIOInputs &inputs) {}
  virtual void RunCurrent(double amps) {}
  virtual void Stop() {}
  virtual void SetBrakeMode(bool enable) {}

  virtual ~GenericSlamElevatorIO() = default;
};