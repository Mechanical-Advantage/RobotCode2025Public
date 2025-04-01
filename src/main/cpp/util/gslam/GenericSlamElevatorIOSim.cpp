// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/gslam/GenericSlamElevatorIOSim.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/math/MathUtil.h>
#include <frc/math/controller/PIDController.h>
#include <frc/math/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>

#include "Constants.h"

GenericSlamElevatorIOSim::GenericSlamElevatorIOSim(double maxLengthMeters,
                                                   double reduction,
                                                   double drumRadius)
    : sim(frc::DCMotor::GetKrakenX60Foc(1), reduction, 0.5, drumRadius, 0.0,
          maxLengthMeters, false, 0.0),
      drumRadius(drumRadius) {
  sim.SetState(maxLengthMeters / 2.0, 0);
}

void GenericSlamElevatorIOSim::UpdateInputs(
    GenericSlamElevatorIOInputs &inputs) {
  if (frc::DriverStation::IsDisabled()) {
    Stop();
  }

  sim.Update(Constants::loopPeriodSecs);
  inputs.data = GenericSlamElevatorIOData{true,
                                          sim.GetPositionMeters() / drumRadius,
                                          sim.GetVelocityMetersPerSecond(),
                                          appliedVoltage,
                                          std::abs(sim.GetCurrentDrawAmps()),
                                          std::abs(sim.GetCurrentDrawAmps()),
                                          0.0};
}

void GenericSlamElevatorIOSim::RunCurrent(double amps) {
  appliedVoltage = currentController.Calculate(sim.GetCurrentDrawAmps(), amps);
  appliedVoltage = frc::math::MathUtil::Clamp(appliedVoltage, -12.0, 12.0);
  sim.SetInputVoltage(appliedVoltage);
}

void GenericSlamElevatorIOSim::Stop() {
  appliedVoltage = 0.0;
  sim.SetInputVoltage(appliedVoltage);
}