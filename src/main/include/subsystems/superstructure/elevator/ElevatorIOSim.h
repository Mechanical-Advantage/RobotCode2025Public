// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <Eigen/Dense>

#include <frc/math/Matrix.h>
#include <frc/math/Nat.h>
#include <frc/math/VecBuilder.h>
#include <frc/math/controller/PIDController.h>
#include <frc/math/numbers/N1.h>
#include <frc/math/numbers/N2.h>
#include <frc/math/system/NumericalIntegration.h>
#include <frc/math/system/plant/DCMotor.h>
#include <frc/math/units/units.h>

#include "Constants.h"
#include "ElevatorIO.h"
#include "subsystems/superstructure/SuperstructureConstants.h"

class ElevatorIOSim : public ElevatorIO {
public:
  static constexpr double carriageMassKg =
      frc::math::units::lbs_to_kilograms(6.0);
  static constexpr double stagesMassKg =
      frc::math::units::lbs_to_kilograms(12.0);
  static constexpr frc::DCMotor gearbox =
      frc::DCMotor::KrakenX60Foc(2).WithReduction(ElevatorIOTalonFX::reduction);

  static constexpr frc::Matrix<frc::N2, frc::N2> A =
      frc::MatBuilder<frc::N2, frc::N2>::fill(
          0, 1, 0,
          -gearbox.KtNMPerAmp() /
              (gearbox.rOhms() * std::pow(Elevator::drumRadius, 2) *
               (carriageMassKg + stagesMassKg) * gearbox.KvRadPerSecPerVolt()));
  static constexpr frc::Vector<frc::N2> B = frc::VecBuilder<frc::N2>::fill(
      0.0, gearbox.KtNMPerAmp() /
               (Elevator::drumRadius * (carriageMassKg + stagesMassKg)));

  ElevatorIOSim();

  void UpdateInputs(ElevatorIOInputsAutoLogged &inputs) override;
  void RunOpenLoop(double output) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void RunPosition(double positionRad, double feedforward) override;
  void SetPID(double kP, double kI, double kD) override;

private:
  frc::Vector<frc::N2> simState;
  double inputTorqueCurrent = 0.0;
  double appliedVolts = 0.0;

  frc::PIDController controller = frc::PIDController(0.0, 0.0, 0.0);
  bool closedLoop = false;
  double feedforward = 0.0;

  void SetInputTorqueCurrent(double torqueCurrent);
  void SetInputVoltage(double voltage);
  void Update(double dt);
};