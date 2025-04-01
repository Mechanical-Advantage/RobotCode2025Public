// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/filter/Debouncer.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/current.h"
#include "frc/units/temperature.h"
#include "frc/units/voltage.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"
#include "org/littletonrobotics/frc2025/util/PhoenixUtil.h"
#include "rev/CANSparkBase.h"
#include "rev/CANSparkFlex.h"
#include "rev/CANSparkMax.h"
#include "rev/RelativeEncoder.h"
#include "rev/SparkBaseConfig.h"
#include "rev/SparkFlexConfig.h"
#include "rev/SparkMaxConfig.h"

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include "ctre/phoenix6/controls/NeutralOut.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/hardware/TalonFX.hpp"
#include "ctre/phoenix6/signals/InvertedValue.hpp"
#include "ctre/phoenix6/signals/NeutralModeValue.hpp"

class RollerSystemIOTalonFX : public RollerSystemIO {
public:
  RollerSystemIOTalonFX(int id, const std::string &bus, int currentLimitAmps,
                        bool invert, bool brake, double reduction);
  ~RollerSystemIOTalonFX() override = default;

  void UpdateInputs(RollerSystemIOInputs &inputs) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void SetCurrentLimit(double currentLimit) override;
  void SetBrakeMode(bool enabled) override;

private:
  ctre::phoenix6::hardware::TalonFX talon;

  ctre::phoenix6::StatusSignal<frc::units::angle::radian_t> position;
  ctre::phoenix6::StatusSignal<
      frc::units::angular_velocity::radians_per_second_t>
      velocity;
  ctre::phoenix6::StatusSignal<frc::units::voltage::volt_t> appliedVoltage;
  ctre::phoenix6::StatusSignal<frc::units::current::ampere_t> supplyCurrent;
  ctre::phoenix6::StatusSignal<frc::units::current::ampere_t> torqueCurrent;
  ctre::phoenix6::StatusSignal<frc::units::temperature::degree_celsius_t>
      tempCelsius;
  ctre::phoenix6::StatusSignal<bool> tempFault;

  ctre::phoenix6::controls::VoltageOut voltageOut =
      ctre::phoenix6::controls::VoltageOut(0.0).WithUpdateFreqHz(0);
  ctre::phoenix6::controls::NeutralOut neutralOut;

  ctre::phoenix6::configs::TalonFXConfiguration config;

  double reduction;

  frc::Debouncer connectedDebouncer = frc::Debouncer(0.5);
};