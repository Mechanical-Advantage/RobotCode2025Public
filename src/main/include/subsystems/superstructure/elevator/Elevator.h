// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cmath>
#include <functional>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/math/MathUtil.h>
#include <frc/math/filter/Debouncer.h>
#include <frc/math/trajectory/TrapezoidProfile.h>
#include <frc/math/units/units.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "ElevatorIO.h"
#include "ElevatorIOInputsAutoLogged.h"
#include "LogInputs.h"
#include "LogOutput.h"
#include "LogProcess.h"
#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "util/EqualsUtil.h"
#include "util/LoggedTracer.h"
#include "util/LoggedTunableNumber.h"

class Elevator {
public:
  static constexpr double drumRadius = frc::math::units::inch_tometer(1.0);

  Elevator(ElevatorIO &io);

  void Periodic();

  void SetGoal(std::function<double()> goal);
  void SetGoal(std::function<frc::TrapezoidProfile::State()> goal);
  void SetOverrides(std::function<bool()> coastOverride,
                    std::function<bool()> disabledOverride);
  frc2::CommandPtr UpStaticCharacterization();
  frc2::CommandPtr DownStaticCharacterization();
  void SetHome();
  frc2::CommandPtr HomingSequence();
  double GetPositionMeters();
  int GetStage();
  double GetGoalMeters();
  frc::TrapezoidProfile::State GetSetpoint() const;
  bool IsHomed() const;
  bool IsAtGoal() const;
  void SetIsEStopped(bool isEStopped);
  void SetHasAlgae(bool hasAlgae);
  void SetStowed(bool stowed);
  bool ShouldEStop() const;

private:
  static LoggedTunableNumber characterizationRampRate;
  static LoggedTunableNumber characterizationUpVelocityThresh;
  static LoggedTunableNumber characterizationDownStartAmps;
  static LoggedTunableNumber characterizationDownVelocityThresh;

  static LoggedTunableNumber kP;
  static LoggedTunableNumber kD;
  static LoggedTunableNumber kS[3];
  static LoggedTunableNumber kG[3];
  static LoggedTunableNumber kA[3];
  static LoggedTunableNumber maxVelocityMetersPerSec;
  static LoggedTunableNumber maxAccelerationMetersPerSec2;
  static LoggedTunableNumber algaeMaxVelocityMetersPerSec;
  static LoggedTunableNumber algaeMaxAccelerationMetersPerSec2;
  static LoggedTunableNumber homingVolts;
  static LoggedTunableNumber homingTimeSecs;
  static LoggedTunableNumber homingVelocityThresh;
  static LoggedTunableNumber tolerance;

  ElevatorIO &io;
  ElevatorIOInputsAutoLogged inputs;

  frc::Alert motorDisconnectedAlert;
  frc::Alert followerDisconnectedAlert;
  std::function<bool()> coastOverride = []() { return false; };
  std::function<bool()> disabledOverride = []() { return false; };

  bool brakeModeEnabled = true;

  frc::TrapezoidProfile profile;
  frc::TrapezoidProfile algaeProfile;
  frc::TrapezoidProfile::State setpoint = frc::TrapezoidProfile::State();
  std::function<frc::TrapezoidProfile::State()> goal = []() {
    return frc::TrapezoidProfile::State();
  };
  bool stopProfile = false;
  bool shouldEStop = false;
  bool isEStopped = false;
  bool hasAlgae = false;

  double homedPosition = 0.0;
  bool homed = false;

  frc::Debouncer homingDebouncer = frc::Debouncer(homingTimeSecs.Get());
  frc::Debouncer toleranceDebouncer =
      frc::Debouncer(0.25, frc::Debouncer::kRising);

  bool atGoal = false;
  bool stowed = false;

  void SetBrakeMode(bool enabled);

  struct StaticCharacterizationState {
    double characterizationOutput = 0.0;
  };
};