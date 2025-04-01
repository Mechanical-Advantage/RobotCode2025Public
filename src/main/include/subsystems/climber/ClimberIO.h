// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "junction/AutoLog.h"
#include "units/angle.h"
#include "units/current.h"
#include "units/temperature.h"
#include "units/time.h"
#include "units/voltage.h"

struct ClimberIOData {
  bool motorConnected = false;
  units::radian_t positionRads = 0_rad;
  units::radians_per_second_t velocityRadsPerSec = 0_rad_per_s;
  units::volt_t appliedVoltage = 0_V;
  units::ampere_t torqueCurrentAmps = 0_A;
  units::ampere_t supplyCurrentAmps = 0_A;
  units::degree_celsius_t tempCelsius = 0_deg_C;
};

struct ClimberIOInputs {
  @AutoLog() ClimberIOData data;
};

class ClimberIO {
public:
  virtual ~ClimberIO() = default;

  virtual void UpdateInputs(ClimberIOInputs &inputs) {}

  virtual void RunTorqueCurrent(double current) {}

  virtual void Stop() {}

  virtual void SetBrakeMode(bool enabled) {}
};