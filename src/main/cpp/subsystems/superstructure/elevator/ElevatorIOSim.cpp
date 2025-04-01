// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "ElevatorIOSim.h"

#include <cmath>

#include <frc/math/MathUtil.h>
#include <frc/math/Matrix.h>
#include <frc/math/Nat.h>
#include <frc/math/VecBuilder.h>
#include <frc/math/numbers/N1.h>
#include <frc/math/numbers/N2.h>
#include <frc/math/system/NumericalIntegration.h>

#include "Constants.h"
#include "subsystems/superstructure/SuperstructureConstants.h"

ElevatorIOSim::ElevatorIOSim()
    : simState(frc::VecBuilder<frc::N2>::fill(0.0, 0.0)) {}

void ElevatorIOSim::UpdateInputs(ElevatorIOInputsAutoLogged &inputs) {
  if (!closedLoop) {
    controller.Reset();
    Update(Constants::loopPeriodSecs);
  } else {
    for (int i = 0;
         i < static_cast<int>(Constants::loopPeriodSecs / (1.0 / 1000.0));
         i++) {
      SetInputTorqueCurrent(
          controller.Calculate(simState(0) / Elevator::drumRadius) +
          feedforward);
      Update(1.0 / 1000.0);
    }
  }

  inputs.data.motorConnected = true;
  inputs.data.followerConnected = true;
  inputs.data.positionRad = simState(0) / Elevator::drumRadius;
  inputs.data.velocityRadPerSec = simState(1) / Elevator::drumRadius;
  inputs.data.appliedVolts = appliedVolts;
  inputs.data.torqueCurrentAmps =
      std::copysign(inputTorqueCurrent, appliedVolts);
  inputs.data.supplyCurrentAmps =
      std::copysign(inputTorqueCurrent, appliedVolts);
  inputs.data.tempCelsius = 0.0;
  inputs.data.followerAppliedVolts = 0.0;
  inputs.data.followerTorqueCurrentAmps = 0.0;
  inputs.data.followerSupplyCurrentAmps = 0.0;
  inputs.data.followerTempCelsius = 0.0;
}

void ElevatorIOSim::RunOpenLoop(double output) {
  closedLoop = false;
  SetInputTorqueCurrent(output);
}

void ElevatorIOSim::RunVolts(double volts) {
  closedLoop = false;
  SetInputVoltage(volts);
}

void ElevatorIOSim::Stop() { RunOpenLoop(0.0); }

void ElevatorIOSim::RunPosition(double positionRad, double feedforward) {
  closedLoop = true;
  controller.SetSetpoint(positionRad);
  this->feedforward = feedforward;
}

void ElevatorIOSim::SetPID(double kP, double kI, double kD) {
  controller.SetPID(kP, kI, kD);
}

void ElevatorIOSim::SetInputTorqueCurrent(double torqueCurrent) {
  inputTorqueCurrent = torqueCurrent;
  appliedVolts = gearbox.GetVoltage(gearbox.GetTorque(inputTorqueCurrent),
                                    simState(1) / Elevator::drumRadius);
  appliedVolts = frc::math::MathUtil::Clamp(appliedVolts, -12.0, 12.0);
}

void ElevatorIOSim::SetInputVoltage(double voltage) {
  SetInputTorqueCurrent(
      gearbox.GetCurrent(simState(1) / Elevator::drumRadius, voltage));
}

void ElevatorIOSim::Update(double dt) {
  inputTorqueCurrent = frc::math::MathUtil::Clamp(inputTorqueCurrent,
                                                  -gearbox.StallCurrentAmps(),
                                                  gearbox.StallCurrentAmps());
  frc::Matrix<frc::N2, frc::N1> updatedState = frc::NumericalIntegration::RKDP(
      [](const frc::Matrix<frc::N2, frc::N1> &x,
         const frc::Matrix<frc::N1, frc::N1> &u) {
        return A * x + B * u +
               frc::VecBuilder<frc::N2>::fill(
                   0.0,
                   -SuperstructureConstants::G *
                       std::sin(SuperstructureConstants::elevatorAngle.Get()));
      },
      simState, frc::MatBuilder<frc::N1, frc::N1>::fill(inputTorqueCurrent),
      dt);

  simState = frc::VecBuilder<frc::N2>::fill(updatedState(0), updatedState(1));

  if (simState(0) <= 0.0) {
    simState(1) = 0.0;
    simState(0) = 0.0;
  }
  if (simState(0) >= SuperstructureConstants::elevatorMaxTravel) {
    simState(1) = 0.0;
    simState(0) = SuperstructureConstants::elevatorMaxTravel;
  }
}