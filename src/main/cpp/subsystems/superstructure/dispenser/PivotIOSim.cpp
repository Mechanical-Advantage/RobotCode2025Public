// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/PivotIOSim.h"

#include <cmath>

#include "frc/controller/PIDController.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/system/NumericalIntegration.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/units/angle.h"
#include "frc/units/mass.h"
#include "frc/units/moment_of_inertia.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/SuperstructureConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/Dispenser.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/elevator/ElevatorIOSim.h"

PivotIOSim::PivotIOSim()
    : simState(
          (Eigen::Vector<double, 2>() << Dispenser::maxAngle.Radians() - 0.1,
           0.0)
              .finished()) {}

void PivotIOSim::UpdateInputs(PivotIOInputs &inputs) {
  if (!closedLoop) {
    controller.Reset();
    Update(Constants::loopPeriodSecs);
  } else {
    for (int i = 0; i < Constants::loopPeriodSecs / (1.0 / 1000.0); i++) {
      SetInputTorqueCurrent(controller.Calculate(simState(0)) + feedforward);
      Update(1.0 / 1000.0);
    }
  }

  inputs.data.motorConnected = true;
  inputs.data.position =
      frc::Rotation2d(simState(0) - Dispenser::maxAngle.Radians());
  inputs.data.velocityRadPerSec = simState(1);
  inputs.data.appliedVolts = pivotAppliedVolts;
  inputs.data.supplyCurrentAmps = 0.0;
  inputs.data.torqueCurrentAmps = inputTorqueCurrent;
  inputs.data.tempCelsius = 0.0;
}

void PivotIOSim::RunOpenLoop(double output) {
  closedLoop = false;
  SetInputTorqueCurrent(output);
}

void PivotIOSim::RunVolts(double volts) {
  closedLoop = false;
  SetInputVoltage(volts);
}

void PivotIOSim::Stop() { RunOpenLoop(0.0); }

void PivotIOSim::RunPosition(const frc::Rotation2d &position,
                             double feedforward) {
  closedLoop = true;
  controller.SetSetpoint(position.Radians() + Dispenser::maxAngle.Radians());
  this->feedforward = feedforward;
}

void PivotIOSim::SetPID(double kP, double kI, double kD) {
  controller = frc::PIDController(kP, kI, kD);
}

void PivotIOSim::SetInputTorqueCurrent(double torqueCurrent) {
  inputTorqueCurrent = torqueCurrent;
  pivotAppliedVolts =
      gearbox.GetVoltage(gearbox.GetTorque(inputTorqueCurrent), simState(1));
  pivotAppliedVolts = frc::math::Clamp(pivotAppliedVolts, -12.0, 12.0);
}

void PivotIOSim::SetInputVoltage(double voltage) {
  SetInputTorqueCurrent(gearbox.GetCurrent(simState(1), voltage));
}

void PivotIOSim::Update(double dt) {
  inputTorqueCurrent = frc::math::Clamp(inputTorqueCurrent, -40.0, 40.0);
  auto updatedState = frc::NumericalIntegration::RKDP(
      [](const Eigen::Vector<double, 2> &x, const Eigen::Vector<double, 1> &u) {
        Eigen::Vector<double, 2> xdot = A * x + B * u;
        xdot += (Eigen::Vector<double, 2>() << 0,
                 -org::littletonrobotics::frc2025::subsystems::superstructure::
                         SuperstructureConstants::G.value() *
                     cgRadius.value() *
                     frc::Rotation2d(x(0))
                         .Minus(org::littletonrobotics::frc2025::subsystems::
                                    superstructure::SuperstructureConstants::
                                        elevatorAngle)
                         .Cos() /
                     moi.value())
                    .finished();
        return xdot;
      },
      simState, (Eigen::Vector<double, 1>() << inputTorqueCurrent).finished(),
      dt);

  simState = (Eigen::Vector<double, 2>() << updatedState(0), updatedState(1))
                 .finished();
  if (simState(0) <= Dispenser::minAngle.Radians()) {
    simState(1) = 0.0;
    simState(0) = Dispenser::minAngle.Radians();
  }
  if (simState(0) >= Dispenser::maxAngle.Radians()) {
    simState(1) = 0.0;
    simState(0) = Dispenser::maxAngle.Radians();
  }
}