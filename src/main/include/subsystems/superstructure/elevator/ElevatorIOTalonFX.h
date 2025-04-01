// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/math/filter/Debouncer.h>
#include <frc/math/units/units.h>

#include "Constants.h"
#include "ElevatorIO.h"
#include "util/PhoenixUtil.h"

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/configs/TalonFXConfiguration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/hardware/TalonFX.hpp>
#include <ctre/phoenix6/signals/InvertedValue.hpp>
#include <ctre/phoenix6/signals/NeutralModeValue.hpp>

class ElevatorIOTalonFX : public ElevatorIO {
public:
  static constexpr double reduction = 4.0 * (1.72 / 1.8);

  ElevatorIOTalonFX();

  void UpdateInputs(ElevatorIOInputsAutoLogged &inputs) override;
  void RunOpenLoop(double output) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void RunPosition(double positionRad, double feedforward) override;
  void SetPID(double kP, double kI, double kD) override;
  void SetBrakeMode(bool enabled) override;

private:
  ctre::phoenix6::hardware::TalonFX talon;
  ctre::phoenix6::hardware::TalonFX followerTalon;

  ctre::phoenix6::configs::TalonFXConfiguration config;

  ctre::phoenix6::StatusSignal<units::radian_t> position;
  ctre::phoenix6::StatusSignal<units::radians_per_second_t> velocity;
  ctre::phoenix6::StatusSignal<units::volt_t> appliedVolts;
  ctre::phoenix6::StatusSignal<units::ampere_t> torqueCurrent;
  ctre::phoenix6::StatusSignal<units::ampere_t> supplyCurrent;
  ctre::phoenix6::StatusSignal<units::degree_Celsius_t> temp;
  ctre::phoenix6::StatusSignal<units::volt_t> followerAppliedVolts;
  ctre::phoenix6::StatusSignal<units::ampere_t> followerTorqueCurrent;
  ctre::phoenix6::StatusSignal<units::ampere_t> followerSupplyCurrent;
  ctre::phoenix6::StatusSignal<units::degree_Celsius_t> followerTemp;

  frc::Debouncer connectedDebouncer = frc::Debouncer(0.5_s);

  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentRequest =
      ctre::phoenix6::controls::TorqueCurrentFOC(0.0).WithUpdateFreqHz(0.0);
  ctre::phoenix6::controls::PositionTorqueCurrentFOC
      positionTorqueCurrentRequest =
          ctre::phoenix6::controls::PositionTorqueCurrentFOC(0.0)
              .WithUpdateFreqHz(0.0);
  ctre::phoenix6::controls::VoltageOut voltageRequest =
      ctre::phoenix6::controls::VoltageOut(0.0).WithUpdateFreqHz(0.0);
};