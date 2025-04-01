// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include "frc/math/Util.h"
#include "frc/units/length.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/frc2025/util/LoggedTunableNumber.h"
#include "org/littletonrobotics/frc2025/util/gslam/GenericSlamElevator.h"
#include "org/littletonrobotics/junction/AutoLog.h"

class Chariot : public GenericSlamElevator<Chariot::Goal> {
public:
  static constexpr frc::units::meter_t drumRadius = frc::units::inch_t(0.5);

  static LoggedTunableNumber occupiedVolts;
  static LoggedTunableNumber floorIntakeVolts;
  static LoggedTunableNumber halfOutPositionInches;

  enum class SlamElevatorState { IDLING, RETRACTING, EXTENDING };

  struct Goal : public GenericSlamElevatorGoal {
    std::function<double()> slammingCurrent;
    bool stopAtGoal;
    SlamElevatorState state;

    Goal(std::function<double()> slammingCurrent, bool stopAtGoal,
         SlamElevatorState state)
        : slammingCurrent(slammingCurrent), stopAtGoal(stopAtGoal),
          state(state) {}
  };

  Chariot(ChariotIO &chariotIO, RollerSystemIO &rollerIO);
  ~Chariot() override = default;

  void Periodic() override;

  Goal GetGoal() const;
  void SetGoal(Goal goal);

  double GetPosition() const;

  void SetIntakeVolts(double intakeVolts);

  void SetCoastOverride(std::function<bool()> coastOverride);

private:
  RollerSystemIO *rollerIO;
  RollerSystemIO::RollerSystemIOInputs rollerInputs;

  Goal goal = Goal{[]() { return 0.0; }, true, SlamElevatorState::IDLING};
  double intakeVolts = 0.0;
  double home = 0.0;
  double position = 0.0;

  std::function<bool()> coastOverride = []() { return false; };

  AUTO_LOG_OUTPUT(position);
  AUTO_LOG_OUTPUT(home);
};