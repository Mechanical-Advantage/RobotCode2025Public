// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIOSim.h"

#include "frc/DriverStation.h"
#include "frc/math/MathUtil.h"
#include "org/littletonrobotics/frc2025/Constants.h"

RollerSystemIOSim::RollerSystemIOSim(frc::DCMotor motorModel, double reduction,
                                     double moi)
    : gearbox(motorModel),
      sim(frc::sim::DCMotorSim(frc::LinearSystemId::IdentifyDCMotorSystem(
                                   motorModel, moi, reduction),
                               motorModel)) {}

void RollerSystemIOSim::UpdateInputs(RollerSystemIOInputs &inputs) {
  if (frc::DriverStation::IsDisabled()) {
    RunVolts(0.0);
  }

  sim.Update(Constants::loopPeriodSecs);
  inputs.data.positionRads = sim.GetAngularPosition();
  inputs.data.velocityRadsPerSec = sim.GetAngularVelocity();
  inputs.data.appliedVoltage = appliedVoltage;
  inputs.data.supplyCurrentAmps = sim.GetCurrentDraw();
  inputs.data.torqueCurrentAmps =
      gearbox.GetCurrent(sim.GetAngularVelocity(), appliedVoltage);
  inputs.data.tempCelsius = 0.0;
  inputs.data.tempFault = false;
  inputs.data.connected = true;
}

void RollerSystemIOSim::RunVolts(double volts) {
  appliedVoltage = frc::math::Clamp(volts, -12.0, 12.0);
  sim.SetInputVoltage(appliedVoltage);
}

void RollerSystemIOSim::Stop() { RunVolts(0.0); }