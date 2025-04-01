// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string>

#include <frc/units/angle.h>
#include <frc/units/angular_velocity.h>
#include <frc/units/current.h>
#include <frc/units/temperature.h>
#include <frc/units/voltage.h>

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/configs/TalonFXConfiguration.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/hardware/TalonFX.hpp>
#include <ctre/phoenix6/signals/InvertedValue.hpp>
#include <ctre/phoenix6/signals/NeutralModeValue.hpp>

#include "util/PhoenixUtil.h"
#include "util/gslam/GenericSlamElevatorIO.h"

class GenericSlamElevatorIOKrakenFOC : public GenericSlamElevatorIO {
public:
  GenericSlamElevatorIOKrakenFOC(int id, const std::string &bus,
                                 int currentLimitAmps, bool invert,
                                 double reduction);

  void UpdateInputs(GenericSlamElevatorIOInputs &inputs) override;
  void RunCurrent(double amps) override;
  void Stop() override;
  void SetBrakeMode(bool enable) override;

private:
  ctre::phoenix6::hardware::TalonFX talon;

  ctre::phoenix6::StatusSignal<frc::units::angle::radian_t> position;
  ctre::phoenix6::StatusSignal<
      frc::units::angular_velocity::radians_per_second_t>
      velocity;
  ctre::phoenix6::StatusSignal<frc::units::voltage::volt_t> appliedVoltage;
  ctre::phoenix6::StatusSignal<frc::units::current::ampere_t> supplyCurrent;
  ctre::phoenix6::StatusSignal<frc::units::current::ampere_t> torqueCurrent;
  ctre::phoenix6::StatusSignal<frc::units::temperature::celsius_t> temp;

  ctre::phoenix6::controls::TorqueCurrentFOC currentControl =
      ctre::phoenix6::controls::TorqueCurrentFOC(0.0).WithUpdateFreqHz(0.0);
  ctre::phoenix6::controls::NeutralOut neutralOut;
};