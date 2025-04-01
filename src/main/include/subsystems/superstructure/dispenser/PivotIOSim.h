// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <numbers>

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
#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/PivotIO.h"

class PivotIOSim : public PivotIO {
public:
  PivotIOSim();

  void UpdateInputs(PivotIOInputs &inputs) override;
  void RunOpenLoop(double output) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void RunPosition(const frc::Rotation2d &position,
                   double feedforward) override;
  void SetPID(double kP, double kI, double kD) override;

private:
  static constexpr double reduction = 3.0;
  static constexpr frc::moment_of_inertia_t moi =
      0.5 *
      frc::kilogram_t(
          org::littletonrobotics::frc2025::subsystems::superstructure::
              elevator::ElevatorIOSim::carriageMassKg) *
      std::pow(org::littletonrobotics::frc2025::subsystems::superstructure::
                   SuperstructureConstants::pivotToTunnelFront.value(),
               2.0);
  static constexpr frc::meter_t cgRadius = frc::units::inch_t(10.0);
  static constexpr frc::DCMotor gearbox =
      frc::DCMotor::KrakenX60Foc(1).WithReduction(reduction);
  static constexpr Eigen::Matrix<double, 2, 2> A =
      (Eigen::Matrix<double, 2, 2>() << 0, 1, 0,
       -gearbox.KtNMPerAmp() /
           (gearbox.KvRadPerSecPerVolt() * gearbox.rOhms() * moi.value()))
          .finished();
  static constexpr Eigen::Vector<double, 2> B =
      (Eigen::Vector<double, 2>() << 0, gearbox.KtNMPerAmp() / moi.value())
          .finished();

  Eigen::Vector<double, 2> simState;
  double inputTorqueCurrent = 0.0;
  double pivotAppliedVolts = 0.0;

  frc::PIDController controller = frc::PIDController(0.0, 0.0, 0.0);
  double feedforward = 0.0;
  bool closedLoop = false;

  void SetInputTorqueCurrent(double torqueCurrent);
  void SetInputVoltage(double voltage);
  void Update(double dt);
};