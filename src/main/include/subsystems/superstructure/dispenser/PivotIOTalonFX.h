// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/filter/Debouncer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/current.h"
#include "frc/units/temperature.h"
#include "frc/units/voltage.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/PivotIO.h"

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"

namespace org::littletonrobotics::frc2025::subsystems::superstructure::
    dispenser {

class PivotIOTalonFX : public PivotIO {
public:
  static constexpr double reduction =
      (72.0 / 12.0) * (44.0 / 19.0) * (74.0 / 24.0);

  PivotIOTalonFX();

  void UpdateInputs(PivotIOInputs &inputs) override;
  void RunOpenLoop(double output) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void RunPosition(const frc::Rotation2d &position,
                   double feedforward) override;
  void SetPID(double kP, double kI, double kD) override;
  void SetBrakeMode(bool enabled) override;

private:
  ctre::phoenix6::hardware::TalonFX talon;
  ctre::phoenix6::configs::TalonFXConfiguration Config;
  ctre::phoenix6::signals::StatusSignal<frc::angle::revolution_t>
      internalPosition;
  ctre::phoenix6::signals::StatusSignal<
      frc::angular_velocity::radians_per_second_t>
      internalVelocity;
  ctre::phoenix6::signals::StatusSignal<frc::voltage::volt_t> appliedVolts;
  ctre::phoenix6::signals::StatusSignal<frc::current::ampere_t>
      supplyCurrentAmps;
  ctre::phoenix6::signals::StatusSignal<frc::current::ampere_t>
      torqueCurrentAmps;
  ctre::phoenix6::signals::StatusSignal<frc::temperature::celsius_t> temp;
  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentFOC =
      ctre::phoenix6::controls::TorqueCurrentFOC(0.0).WithUpdateFreqHz(0.0);
  ctre::phoenix6::controls::PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      ctre::phoenix6::controls::PositionTorqueCurrentFOC(0.0).WithUpdateFreqHz(
          0.0);
  ctre::phoenix6::controls::VoltageOut voltageRequest =
      ctre::phoenix6::controls::VoltageOut(0.0).WithUpdateFreqHz(0.0);
  frc::Debouncer motorConnectedDebouncer = frc::Debouncer(0.5);
};

} // namespace
  // org::littletonrobotics::frc2025::subsystems::superstructure::dispenser