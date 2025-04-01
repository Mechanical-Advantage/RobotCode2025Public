// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "ClimberIOSim.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/math/Matrix.h"
#include "frc/math/VecBuilder.h"
#include "frc/math/numbers/N1.h"
#include "frc/math/numbers/N2.h"
#include "frc/math/system/NumericalIntegration.h"
#include "frc/math/system/plant/DCMotor.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/SuperstructureConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/SuperstructureVisualizer.h"

ClimberIOSim::ClimberIOSim() {
  simState = frc::VecBuilder<frc::numbers::N2>::Fill(wpi::math::pi / 2.0, 0.0);
}

void ClimberIOSim::UpdateInputs(ClimberIOInputs &inputs) {
  Update(org::littletonrobotics::frc2025::Constants::loopPeriodSecs);
  org::littletonrobotics::frc2025::subsystems::superstructure::
      SuperstructureVisualizer::UpdateSimIntake(simState(0));

  inputs.data.motorConnected = true;
  inputs.data.positionRads = units::radian_t(simState(0));
  inputs.data.velocityRadsPerSec = units::radians_per_second_t(simState(1));
  inputs.data.appliedVoltage = units::volt_t(appliedVolts);
  inputs.data.torqueCurrentAmps =
      units::ampere_t(wpi::math::copySign(inputTorqueCurrent, appliedVolts));
  inputs.data.supplyCurrentAmps =
      units::ampere_t(wpi::math::copySign(inputTorqueCurrent, appliedVolts));
  inputs.data.tempCelsius = units::degree_celsius_t(0.0);
}

void ClimberIOSim::RunTorqueCurrent(double current) {
  inputTorqueCurrent = current;
  appliedVolts =
      gearbox.GetVoltage(gearbox.GetTorque(inputTorqueCurrent), simState(1));
  appliedVolts = frc::math::clamp(appliedVolts, -12.0, 12.0);
}

void ClimberIOSim::Stop() { RunTorqueCurrent(0.0); }

void ClimberIOSim::Update(double dt) {
  inputTorqueCurrent = frc::math::clamp(
      inputTorqueCurrent, -gearbox.StallCurrent(), gearbox.StallCurrent());
  frc::Matrix<frc::numbers::N2, frc::numbers::N1> updatedState =
      frc::NumericalIntegration::RKDP(
          [](const frc::Matrix<frc::numbers::N2, frc::numbers::N1> &x,
             const frc::Matrix<frc::numbers::N1, frc::numbers::N1> &u) {
            return A * x + B * u +
                   (-org::littletonrobotics::frc2025::subsystems::
                        superstructure::SuperstructureConstants::G *
                    cgRadius * frc::Rotation2d(units::radian_t(x(0))).Cos() /
                    moi);
          },
          simState,
          frc::VecBuilder<frc::numbers::N1>::Fill(
              inputTorqueCurrent * 15.0), // Magic constant of doom
          dt);
  simState =
      frc::VecBuilder<frc::numbers::N2>::Fill(updatedState(0), updatedState(1));
}