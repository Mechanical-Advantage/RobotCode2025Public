// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc2/command/Command.h"
#include "frc2/command/SubsystemBase.h"
#include "units/angle.h"
#include "units/time.h"

#include "junction/AutoLogOutput.h"
#include "junction/Logger.h"
#include "util/LoggedTracer.h"
#include "util/LoggedTunableNumber.h"

// Forward declaration
struct ClimberIOInputsAutoLogged;

class Climber : public frc2::SubsystemBase {
public:
  enum class ClimbState { START, DEPLOYED, CLIMBING };

  Climber(class ClimberIO &climberIO);

  void Periodic() override;

  frc2::CommandPtr Deploy();
  frc2::CommandPtr Undeploy();
  frc2::CommandPtr Climb();

  void SetCoastOverride(std::function<bool()> coastOverride);

private:
  static LoggedTunableNumber deployCurrent;
  static LoggedTunableNumber deployAngle;
  static LoggedTunableNumber undeployAngle;
  static LoggedTunableNumber climbCurrent;
  static LoggedTunableNumber climbCurrentRampRate;
  static LoggedTunableNumber climbStopAngle;

  class ClimberIO &climberIO;
  ClimberIOInputsAutoLogged climberInputs;

  std::function<bool()> coastOverride = []() { return false; };

  @AutoLogOutput(key = "Climber/BrakeModeEnabled") bool brakeModeEnabled = true;

  void SetBrakeMode(bool enabled);
};

// Forward declaration of ClimberIO class
class ClimberIO {
public:
  virtual ~ClimberIO() = default;
  virtual void UpdateInputs(ClimberIOInputsAutoLogged &inputs) = 0;
  virtual void RunTorqueCurrent(double current) = 0;
  virtual void SetBrakeMode(bool enabled) = 0;
};

// Forward declaration of ClimberIOInputs struct
struct ClimberIOInputs {
  units::radian_t positionRads;
  // Add other input fields as needed
};

// Forward declaration of ClimberIOInputsAutoLogged struct
struct ClimberIOInputsAutoLogged {
  @AutoLogOutput(key = "Climber/PositionRads") ClimberIOInputs data;
};