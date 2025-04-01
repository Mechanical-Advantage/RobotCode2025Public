// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "org/littletonrobotics/junction/AutoLog.h"

class RollerSystemIO {
public:
  struct RollerSystemIOInputs {
    struct RollerSystemIOData {
      double positionRads;
      double velocityRadsPerSec;
      double appliedVoltage;
      double supplyCurrentAmps;
      double torqueCurrentAmps;
      double tempCelsius;
      bool tempFault;
      bool connected;

      AUTO_LOG_STRUCT()
    };

    RollerSystemIOData data;

    AUTO_LOG_STRUCT()
  };

  virtual ~RollerSystemIO() = default;

  virtual void UpdateInputs(RollerSystemIOInputs &inputs) {}

  /* Run rollers at volts */
  virtual void RunVolts(double volts) {}

  /* Stop rollers */
  virtual void Stop() {}

  virtual void SetCurrentLimit(double currentLimit) {}

  virtual void SetBrakeMode(bool enabled) {}
};