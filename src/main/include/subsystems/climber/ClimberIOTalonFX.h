// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "ClimberIO.h"
#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"
#include "frc/filter/Debouncer.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/temperature.h"
#include "units/voltage.h"

class ClimberIOTalonFX : public ClimberIO {
public:
  static constexpr double reduction = 600.0;

private:
  // Hardware
  ctre::phoenix6::hardware::TalonFX talon;

  // Status Signals
  ctre::phoenix6::StatusSignal<units::radian_t> position;
  ctre::phoenix6::StatusSignal<units::radians_per_second_t> velocity;
  ctre::phoenix6::StatusSignal<units::volt_t> appliedVolts;
  ctre::phoenix6::StatusSignal<units::ampere_t> supplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::ampere_t> torqueCurrentAmps;
  ctre::phoenix6::StatusSignal<units::degree_celsius_t> temp;

  // Control Requests
  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentRequest =
      ctre::phoenix6::controls::TorqueCurrentFOC(0.0).WithUpdateFreqHz(0.0);

  // Connected debouncers
  frc::Debouncer motorConnectedDebouncer = frc::Debouncer(0.5);

public:
  ClimberIOTalonFX();

  void UpdateInputs(ClimberIOInputs &inputs) override;
  void RunTorqueCurrent(double current) override;
  void Stop() override;
  void SetBrakeMode(bool enabled) override;
};