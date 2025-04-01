// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/math/controller/PIDController.h>
#include <frc/math/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>

#include "util/gslam/GenericSlamElevatorIO.h"

class GenericSlamElevatorIOSim : public GenericSlamElevatorIO {
public:
  GenericSlamElevatorIOSim(double maxLengthMeters, double reduction,
                           double drumRadius);

  void UpdateInputs(GenericSlamElevatorIOInputs &inputs) override;
  void RunCurrent(double amps) override;
  void Stop() override;

private:
  frc::sim::ElevatorSim sim;
  double appliedVoltage = 0.0;
  frc::PIDController currentController =
      frc::PIDController((12.0 / 483.0) * 40, 0.0, 0.0);
  double drumRadius;
};