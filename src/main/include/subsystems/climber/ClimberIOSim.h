// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "ClimberIO.h"
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

class ClimberIOSim : public ClimberIO {
private:
  static constexpr double moi = 0.0;
  static constexpr double cgRadius = 0.2;
  static constexpr frc::DCMotor gearbox =
      frc::DCMotor::KrakenX60Foc(1).WithReduction(ClimberIOTalonFX::reduction);
  static constexpr frc::Matrix<frc::numbers::N2, frc::numbers::N2> A =
      frc::MatBuilder<frc::numbers::N2, frc::numbers::N2>::Fill(
          0, 1, 0,
          -gearbox.KtNMPerAmp() /
              (gearbox.KvRadPerSecPerVolt() * gearbox.rOhms() * moi));
  static constexpr frc::Vector<frc::numbers::N2> B =
      frc::VecBuilder<frc::numbers::N2>::Fill(0, gearbox.KtNMPerAmp() / moi);

  // Climber sim
  frc::Vector<frc::numbers::N2> simState;
  double inputTorqueCurrent = 0.0;
  double appliedVolts = 0.0;

  void Update(double dt);

public:
  ClimberIOSim();

  void UpdateInputs(ClimberIOInputs &inputs) override;
  void RunTorqueCurrent(double current) override;
  void Stop() override;
};