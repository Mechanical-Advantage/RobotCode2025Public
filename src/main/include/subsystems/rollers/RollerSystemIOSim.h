// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/math/MathUtil.h"
#include "frc/math/system/plant/DCMotor.h"
#include "frc/math/system/plant/LinearSystemId.h"
#include "frc/simulation/DCMotorSim.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"

class RollerSystemIOSim : public RollerSystemIO {
public:
  RollerSystemIOSim(frc::DCMotor motorModel, double reduction, double moi);
  ~RollerSystemIOSim() override = default;

  void UpdateInputs(RollerSystemIOInputs &inputs) override;
  void RunVolts(double volts) override;
  void Stop() override;

private:
  frc::sim::DCMotorSim sim;
  frc::DCMotor gearbox;
  double appliedVoltage = 0.0;
};