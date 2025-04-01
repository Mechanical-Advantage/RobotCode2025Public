// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Elevator.h"

#include <cmath>

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
#include "LogInputs.h"
#include "LogOutput.h"
#include "LogProcess.h"
#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "util/EqualsUtil.h"
#include "util/LoggedTracer.h"
#include "util/LoggedTunableNumber.h"

LoggedTunableNumber
    Elevator::characterizationRampRate("Elevator/CharacterizationRampRate",
                                       0.5);
LoggedTunableNumber Elevator::characterizationUpVelocityThresh(
    "Elevator/CharacterizationUpVelocityThresh", 0.1);
LoggedTunableNumber Elevator::characterizationDownStartAmps(
    "Elevator/CharacterizationDownStartAmps", 0.0);
LoggedTunableNumber Elevator::characterizationDownVelocityThresh(
    "Elevator/CharacterizationDownVelocityThresh", -0.1);

LoggedTunableNumber Elevator::kP("Elevator/kP");
LoggedTunableNumber Elevator::kD("Elevator/kD");
LoggedTunableNumber Elevator::kS[] = {
    LoggedTunableNumber("Elevator/kS/Stage1"),
    LoggedTunableNumber("Elevator/kS/Stage2"),
    LoggedTunableNumber("Elevator/kS/Stage3")};
LoggedTunableNumber Elevator::kG[] = {
    LoggedTunableNumber("Elevator/kG/Stage1"),
    LoggedTunableNumber("Elevator/kG/Stage2"),
    LoggedTunableNumber("Elevator/kG/Stage3")};
LoggedTunableNumber Elevator::kA[] = {
    LoggedTunableNumber("Elevator/kA/Stage1"),
    LoggedTunableNumber("Elevator/kA/Stage2"),
    LoggedTunableNumber("Elevator/kA/Stage3")};
LoggedTunableNumber
    Elevator::maxVelocityMetersPerSec("Elevator/MaxVelocityMetersPerSec", 2.5);
LoggedTunableNumber Elevator::maxAccelerationMetersPerSec2(
    "Elevator/MaxAccelerationMetersPerSec2", 8.0);
LoggedTunableNumber Elevator::algaeMaxVelocityMetersPerSec(
    "Elevator/AlgaeMaxVelocityMetersPerSec", 1.5);
LoggedTunableNumber Elevator::algaeMaxAccelerationMetersPerSec2(
    "Elevator/AlgaeMaxAccelerationMetersPerSec2", 4.0);
LoggedTunableNumber Elevator::homingVolts("Elevator/HomingVolts", -2.0);
LoggedTunableNumber Elevator::homingTimeSecs("Elevator/HomingTimeSecs", 0.25);
LoggedTunableNumber
    Elevator::homingVelocityThresh("Elevator/HomingVelocityThresh", 5.0);
LoggedTunableNumber Elevator::tolerance("Elevator/Tolerance", 0.5);

Elevator::Elevator(ElevatorIO &io)
    : io(io), motorDisconnectedAlert("Elevator leader motor disconnected!",
                                     frc::Alert::AlertType::kWarning),
      followerDisconnectedAlert("Elevator follower motor disconnected!",
                                frc::Alert::AlertType::kWarning),
      profile(frc::TrapezoidProfile::Constraints(
          maxVelocityMetersPerSec.Get(), maxAccelerationMetersPerSec2.Get())) {
  switch (Constants::GetRobot()) {
  case RobotType::COMPBOT:
  case RobotType::DEVBOT:
    kP.InitDefault(600);
    kD.InitDefault(10);
    for (int stage = 0; stage < 3; stage++) {
      kS[stage].InitDefault(0);
      kG[stage].InitDefault(0);
      kA[stage].InitDefault(0);
    }
    break;
  case RobotType::SIMBOT:
    kP.InitDefault(5000);
    kD.InitDefault(2000);
    for (int stage = 0; stage < 3; stage++) {
      kS[stage].InitDefault(5);
      kG[stage].InitDefault(50);
      kA[stage].InitDefault(0);
    }
    break;
  }
}

void Elevator::Periodic() {
  io.UpdateInputs(inputs);
  Logger::ProcessInputs("Elevator", inputs);

  motorDisconnectedAlert.Set(!inputs.data.motorConnected && !Robot::IsJITing());
  followerDisconnectedAlert.Set(!inputs.data.followerConnected &&
                                !Robot::IsJITing());

  if (kP.HasChanged(this->hashCode()) || kD.HasChanged(this->hashCode())) {
    io.SetPID(kP.Get(), 0.0, kD.Get());
  }
  if (maxVelocityMetersPerSec.HasChanged(this->hashCode()) ||
      maxAccelerationMetersPerSec2.HasChanged(this->hashCode())) {
    profile = frc::TrapezoidProfile::Constraints(
        maxVelocityMetersPerSec.Get(), maxAccelerationMetersPerSec2.Get());
  }
  if (algaeMaxVelocityMetersPerSec.HasChanged(this->hashCode()) ||
      algaeMaxAccelerationMetersPerSec2.HasChanged(this->hashCode())) {
    algaeProfile = frc::TrapezoidProfile::Constraints(
        algaeMaxVelocityMetersPerSec.Get(),
        algaeMaxAccelerationMetersPerSec2.Get());
  }

  SetBrakeMode(!coastOverride());

  const bool shouldRunProfile = !stopProfile && !coastOverride() &&
                                !disabledOverride() && homed && !isEStopped &&
                                frc::DriverStation::IsEnabled();
  Logger::RecordOutput("Elevator/RunningProfile", shouldRunProfile);
  bool outOfTolerance =
      std::abs(GetPositionMeters() - setpoint.position) > tolerance.Get();
  shouldEStop =
      toleranceDebouncer.Calculate(outOfTolerance && shouldRunProfile);
  if (shouldRunProfile) {
    frc::TrapezoidProfile::State goalState(
        frc::math::MathUtil::Clamp(goal().position, 0.0,
                                   SuperstructureConstants::elevatorMaxTravel),
        goal().velocity);
    double previousVelocity = setpoint.velocity;
    setpoint = (hasAlgae ? algaeProfile : profile)
                   .Calculate(Constants::loopPeriodSecs, setpoint, goalState);
    if (setpoint.position < 0.0 ||
        setpoint.position > SuperstructureConstants::elevatorMaxTravel) {
      setpoint = frc::TrapezoidProfile::State(
          frc::math::MathUtil::Clamp(
              setpoint.position, 0.0,
              SuperstructureConstants::elevatorMaxTravel),
          0.0);
    }

    double accel =
        (setpoint.velocity - previousVelocity) / Constants::loopPeriodSecs;
    io.RunPosition(setpoint.position / drumRadius + homedPosition,
                   kS[GetStage()].Get() *
                           std::copysign(1.0, setpoint.velocity) +
                       kG[GetStage()].Get() + kA[GetStage()].Get() * accel);

    atGoal = EqualsUtil::EpsilonEquals(setpoint.position, goalState.position) &&
             EqualsUtil::EpsilonEquals(setpoint.velocity, goalState.velocity);

    Logger::RecordOutput("Elevator/Profile/SetpointPositionMeters",
                         setpoint.position);
    Logger::RecordOutput("Elevator/Profile/SetpointVelocityMetersPerSec",
                         setpoint.velocity);
    Logger::RecordOutput("Elevator/Profile/GoalPositionMeters",
                         goalState.position);
    Logger::RecordOutput("Elevator/Profile/GoalVelocityMetersPerSec",
                         goalState.velocity);
  } else {
    setpoint = frc::TrapezoidProfile::State(GetPositionMeters(), 0.0);

    Logger::RecordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
    Logger::RecordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
    Logger::RecordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
    Logger::RecordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
  }

  if (isEStopped) {
    io.Stop();
  }

  Logger::RecordOutput("Elevator/CoastOverride", coastOverride());
  Logger::RecordOutput("Elevator/DisabledOverride", disabledOverride());
  Logger::RecordOutput("Elevator/MeasuredVelocityMetersPerSec",
                       inputs.data.velocityRadPerSec * drumRadius);

  LoggedTracer::Record("Elevator");
}

void Elevator::SetGoal(std::function<double()> goal) {
  SetGoal([goal]() { return frc::TrapezoidProfile::State(goal(), 0.0); });
}

void Elevator::SetGoal(std::function<frc::TrapezoidProfile::State()> goal) {
  atGoal = false;
  this->goal = goal;
}

void Elevator::SetOverrides(std::function<bool()> coastOverride,
                            std::function<bool()> disabledOverride) {
  this->coastOverride = coastOverride;
  this->disabledOverride = disabledOverride;
}

void Elevator::SetBrakeMode(bool enabled) {
  if (brakeModeEnabled == enabled)
    return;
  brakeModeEnabled = enabled;
  io.SetBrakeMode(brakeModeEnabled);
}

frc2::CommandPtr Elevator::UpStaticCharacterization() {
  StaticCharacterizationState state;
  frc::Timer timer;
  return frc2::Commands::StartRun(
             [this, &timer, &state]() {
               stopProfile = true;
               timer.Restart();
             },
             [this, &timer, &state]() {
               stopProfile = true;
               state.characterizationOutput =
                   characterizationRampRate.Get() * timer.Get();
               io.RunOpenLoop(state.characterizationOutput);
               Logger::RecordOutput("Elevator/CharacterizationOutputUp",
                                    state.characterizationOutput);
             })
      .Until([this]() {
        return inputs.data.velocityRadPerSec >=
               characterizationUpVelocityThresh.Get();
      })
      .AndThen(io.Stop())
      .AndThen(frc2::Commands::Idle())
      .FinallyDo([this, &timer, &state]() {
        stopProfile = false;
        timer.Stop();
        Logger::RecordOutput("Elevator/CharacterizationOutputUp",
                             state.characterizationOutput);
      });
}

frc2::CommandPtr Elevator::DownStaticCharacterization() {
  StaticCharacterizationState state;
  frc::Timer timer;
  return frc2::Commands::StartRun(
             [this, &timer, &state]() {
               stopProfile = true;
               timer.Restart();
             },
             [this, &timer, &state]() {
               state.characterizationOutput =
                   characterizationDownStartAmps.Get() -
                   characterizationRampRate.Get() * timer.Get();
               io.RunOpenLoop(state.characterizationOutput);
               Logger::RecordOutput("Elevator/CharacterizationOutputDown",
                                    state.characterizationOutput);
             })
      .Until([this]() {
        return inputs.data.velocityRadPerSec <=
               characterizationDownVelocityThresh.Get();
      })
      .AndThen(io.Stop())
      .AndThen(frc2::Commands::Idle())
      .FinallyDo([this, &timer, &state]() {
        stopProfile = false;
        timer.Stop();
        Logger::RecordOutput("Elevator/CharacterizationOutputDown",
                             state.characterizationOutput);
      });
}

void Elevator::SetHome() {
  homedPosition = inputs.data.positionRad;
  homed = true;
}

frc2::CommandPtr Elevator::HomingSequence() {
  return frc2::Commands::StartRun(
             [this]() {
               stopProfile = true;
               homed = false;
               homingDebouncer = frc::Debouncer(homingTimeSecs.Get());
               homingDebouncer.Calculate(false);
             },
             [this]() {
               if (disabledOverride() || coastOverride())
                 return;
               io.RunVolts(homingVolts.Get());
               homed = homingDebouncer.Calculate(
                   std::abs(inputs.data.velocityRadPerSec) <=
                   homingVelocityThresh.Get());
             })
      .Until([this]() { return homed; })
      .AndThen([this]() { SetHome(); })
      .FinallyDo([this]() { stopProfile = false; });
}

double Elevator::GetPositionMeters() {
  return (inputs.data.positionRad - homedPosition) * drumRadius;
}

int Elevator::GetStage() {
  if (GetPositionMeters() <= SuperstructureConstants::stage1ToStage2Height) {
    return 0;
  } else if (GetPositionMeters() <=
             SuperstructureConstants::stage2ToStage3Height) {
    return 1;
  } else {
    return 2;
  }
}

double Elevator::GetGoalMeters() { return goal().position; }

frc::TrapezoidProfile::State Elevator::GetSetpoint() const { return setpoint; }

bool Elevator::IsHomed() const { return homed; }

bool Elevator::IsAtGoal() const { return atGoal; }

void Elevator::SetIsEStopped(bool isEStopped) { this->isEStopped = isEStopped; }

void Elevator::SetHasAlgae(bool hasAlgae) { this->hasAlgae = hasAlgae; }

void Elevator::SetStowed(bool stowed) { this->stowed = stowed; }

bool Elevator::ShouldEStop() const { return shouldEStop; }
