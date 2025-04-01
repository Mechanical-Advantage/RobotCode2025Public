// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/superstructure/dispenser/Dispenser.h"

#include <cmath>

#include "frc/DriverStation.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/Robot.h"
#include "org/littletonrobotics/frc2025/subsystems/leds/Leds.h"
#include "org/littletonrobotics/frc2025/util/EqualsUtil.h"
#include "org/littletonrobotics/junction/Logger.h"

frc::Rotation2d Dispenser::minAngle = frc::Rotation2d::Degrees(-90.0);
frc::Rotation2d Dispenser::maxAngle = frc::Rotation2d::Degrees(20.5);

LoggedTunableNumber Dispenser::kP = LoggedTunableNumber("Dispenser/kP");
LoggedTunableNumber Dispenser::kD = LoggedTunableNumber("Dispenser/kD");
LoggedTunableNumber Dispenser::kS = LoggedTunableNumber("Dispenser/kS");
LoggedTunableNumber Dispenser::kG = LoggedTunableNumber("Dispenser/kG");
LoggedTunableNumber Dispenser::maxVelocityDegPerSec =
    LoggedTunableNumber("Dispenser/MaxVelocityDegreesPerSec", 1500.0);
LoggedTunableNumber Dispenser::maxAccelerationDegPerSec2 =
    LoggedTunableNumber("Dispenser/MaxAccelerationDegreesPerSec2", 2500.0);
LoggedTunableNumber Dispenser::algaeMaxVelocityDegPerSec =
    LoggedTunableNumber("Dispenser/AlgaeMaxVelocityDegreesPerSec", 300.0);
LoggedTunableNumber Dispenser::algaeMaxAccelerationDegPerSec2 =
    LoggedTunableNumber("Dispenser/AlgaeMaxAccelerationDegreesPerSec2", 400.0);
LoggedTunableNumber Dispenser::staticCharacterizationVelocityThresh =
    LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
LoggedTunableNumber Dispenser::staticCharacterizationRampRate =
    LoggedTunableNumber("Dispenser/StaticCharacterizationRampRate", 0.2);
LoggedTunableNumber Dispenser::algaeVelocityThresh =
    LoggedTunableNumber("Dispenser/AlgaeVelocityThreshold", 45.0);
LoggedTunableNumber Dispenser::coralProxThreshold =
    LoggedTunableNumber("Dispenser/CoralProxThresh", 0.06);
LoggedTunableNumber Dispenser::gripperHoldVolts =
    LoggedTunableNumber("Dispenser/GripperHoldVolts", 5.0);
LoggedTunableNumber Dispenser::gripperIntakeVolts =
    LoggedTunableNumber("Dispenser/GripperIntakeVolts", 5.0);
LoggedTunableNumber Dispenser::gripperEjectVolts =
    LoggedTunableNumber("Dispenser/GripperEjectVolts", -12.0);
LoggedTunableNumber Dispenser::gripperL1EjectVolts =
    LoggedTunableNumber("Dispenser/GripperL1EjectVolts", -5.0);
LoggedTunableNumber Dispenser::gripperCurrentLimit =
    LoggedTunableNumber("Dispenser/GripperCurrentLimit", 50.0);
LoggedTunableNumber Dispenser::tunnelDispenseVolts =
    LoggedTunableNumber("Dispenser/TunnelDispenseVolts", 6.0);
LoggedTunableNumber Dispenser::tunnelL1DispenseVolts =
    LoggedTunableNumber("Dispenser/TunnelL1DispenseVolts", 5.0);
LoggedTunableNumber Dispenser::tunnelIntakeVolts =
    LoggedTunableNumber("Dispenser/TunnelIntakeVolts", 3.0);
LoggedTunableNumber Dispenser::tolerance =
    LoggedTunableNumber("Dispenser/Tolerance", 0.4);
LoggedTunableNumber Dispenser::intakeReverseVolts =
    LoggedTunableNumber("Dispenser/IntakeReverseVolts", -1.8);
LoggedTunableNumber Dispenser::intakeReverseTime =
    LoggedTunableNumber("Dispenser/IntakeReverseTime", 0.1);
LoggedTunableNumber Dispenser::homingTimeSecs =
    LoggedTunableNumber("Dispenser/HomingTimeSecs", 0.2);
LoggedTunableNumber Dispenser::homingVolts =
    LoggedTunableNumber("Dispenser/HomingVolts", 3.0);
LoggedTunableNumber Dispenser::homingVelocityThresh =
    LoggedTunableNumber("Dispenser/HomingVelocityThreshold", 0.1);

namespace {
void InitDefaults() {
  switch (Constants::GetRobot()) {
  case Constants::RobotType::SIMBOT:
    Dispenser::kP.InitDefault(4000);
    Dispenser::kD.InitDefault(1000);
    Dispenser::kS.InitDefault(1.2);
    Dispenser::kG.InitDefault(0.0);
    break;
  default:
    Dispenser::kP.InitDefault(12000.0);
    Dispenser::kD.InitDefault(120.0);
    Dispenser::kS.InitDefault(0);
    Dispenser::kG.InitDefault(0);
    break;
  }
}
} // namespace

Dispenser::Dispenser(PivotIO &pivotIO, RollerSystemIO &tunnelIO,
                     RollerSystemIO &gripperIO, CoralSensorIO &coralSensorIO)
    : pivotIO(&pivotIO), tunnelIO(&tunnelIO), gripperIO(&gripperIO),
      coralSensorIO(&coralSensorIO),
      profile(frc::TrapezoidProfile::Constraints(
          frc::units::degrees_per_second_t(maxVelocityDegPerSec.Get()).value(),
          frc::units::degrees_per_second_squared_t(
              maxAccelerationDegPerSec2.Get())
              .value())),
      intakingReverseTimer() {
  InitDefaults();
  intakingReverseTimer.Start();
}

void Dispenser::Periodic() {
  pivotIO->UpdateInputs(pivotInputs);
  Logger::ProcessInputs("Dispenser/Pivot", pivotInputs);
  tunnelIO->UpdateInputs(tunnelInputs);
  Logger::ProcessInputs("Dispenser/Tunnel", tunnelInputs);
  gripperIO->UpdateInputs(gripperInputs);
  Logger::ProcessInputs("Dispenser/Gripper", gripperInputs);
  coralSensorIO->UpdateInputs(coralSensorInputs);
  Logger::ProcessInputs("Dispenser/CoralSensor", coralSensorInputs);

  pivotMotorDisconnectedAlert.Set(!pivotInputs.data.motorConnected &&
                                  Constants::GetRobot() ==
                                      Constants::RobotType::COMPBOT &&
                                  !Robot::IsJITing());
  tunnelDisconnectedAlert.Set(!tunnelInputs.data.connected &&
                              !Robot::IsJITing());
  gripperDisconnectedAlert.Set(!gripperInputs.data.connected &&
                               Constants::GetRobot() ==
                                   Constants::RobotType::COMPBOT &&
                               !Robot::IsJITing());

  if (kP.HasChanged(this->GetHashCode()) ||
      kD.HasChanged(this->GetHashCode())) {
    pivotIO->SetPID(kP.Get(), 0.0, kD.Get());
  }
  if (maxVelocityDegPerSec.HasChanged(this->GetHashCode()) ||
      maxAccelerationDegPerSec2.HasChanged(this->GetHashCode())) {
    profile = frc::TrapezoidProfile::Constraints(
        frc::units::degrees_per_second_t(maxVelocityDegPerSec.Get()).value(),
        frc::units::degrees_per_second_squared_t(
            maxAccelerationDegPerSec2.Get())
            .value());
  }
  if (algaeMaxVelocityDegPerSec.HasChanged(this->GetHashCode()) ||
      algaeMaxAccelerationDegPerSec2.HasChanged(this->GetHashCode())) {
    algaeProfile = frc::TrapezoidProfile::Constraints(
        frc::units::degrees_per_second_t(algaeMaxVelocityDegPerSec.Get())
            .value(),
        frc::units::degrees_per_second_squared_t(
            algaeMaxAccelerationDegPerSec2.Get())
            .value());
  }
  if (gripperCurrentLimit.HasChanged(this->GetHashCode())) {
    gripperIO->SetCurrentLimit(gripperCurrentLimit.Get());
  }

  SetBrakeMode(!coastOverride());

  const bool shouldRunProfile = !stopProfile && !coastOverride() &&
                                !disabledOverride() && !isEStopped &&
                                frc::DriverStation::IsEnabled();
  Logger::RecordOutput("Dispenser/RunningProfile", shouldRunProfile);

  const bool outOfTolerance =
      std::abs(GetPivotAngle().Radians() - setpoint.position) > tolerance.Get();
  shouldEStop =
      toleranceDebouncer.Calculate(outOfTolerance && shouldRunProfile);
  if (shouldRunProfile) {
    frc::TrapezoidProfile::State goalState(
        frc::math::Clamp(goal(), minAngle.Radians(), maxAngle.Radians()), 0.0);
    setpoint = (HasAlgae() ? algaeProfile : profile)
                   .Calculate(Constants::loopPeriodSecs, setpoint, goalState);
    pivotIO->RunPosition(frc::Rotation2d(setpoint.position -
                                         maxAngle.Radians() +
                                         homingOffset.Radians()),
                         kS.Get() * std::copysign(1.0, setpoint.velocity) +
                             kG.Get() * GetPivotAngle().Cos());
    atGoal = EqualsUtil::EpsilonEquals(setpoint.position, goalState.position) &&
             EqualsUtil::EpsilonEquals(setpoint.velocity, 0.0);

    Logger::RecordOutput("Dispenser/Profile/SetpointAngleRad",
                         setpoint.position);
    Logger::RecordOutput("Dispenser/Profile/SetpointAngleRadPerSec",
                         setpoint.velocity);
    Logger::RecordOutput("Dispenser/Profile/GoalAngleRad", goalState.position);
  } else {
    setpoint = frc::TrapezoidProfile::State(GetPivotAngle().Radians(), 0.0);

    Logger::RecordOutput("Dispenser/Profile/SetpointAngleRad", 0.0);
    Logger::RecordOutput("Dispenser/Profile/SetpointAngleRadPerSec", 0.0);
    Logger::RecordOutput("Dispenser/Profile/GoalAngleRad", 0.0);
  }

  Leds::GetInstance().coralGrabbed = false;
  if (!isEStopped) {
    double intakeVolts = tunnelVolts;
    if (isIntaking && !HasCoral()) {
      intakingReverseTimer.Restart();
    } else if (intakingReverseTimer.Get() <
               frc::units::second_t(intakeReverseTime.Get())) {
      intakeVolts = intakeReverseVolts.Get();
    } else if (isIntaking) {
      Leds::GetInstance().coralGrabbed = true;
      intakeVolts = 0.0;
    }
    tunnelIO->RunVolts(intakeVolts);

    switch (gripperGoal) {
    case GripperGoal::IDLE:
      gripperIO->Stop();
      break;
    case GripperGoal::GRIP:
      if (HasAlgae()) {
        gripperIO->RunVolts(gripperHoldVolts.Get());
      } else {
        gripperIO->RunVolts(gripperIntakeVolts.Get());
      }
      break;
    case GripperGoal::EJECT:
      gripperIO->RunVolts(gripperEjectVolts.Get());
      break;
    case GripperGoal::L1_EJECT:
      gripperIO->RunVolts(gripperL1EjectVolts.Get());
      break;
    }
  } else {
    pivotIO->Stop();
    tunnelIO->Stop();
    gripperIO->Stop();
  }

  if (Constants::GetRobot() != Constants::RobotType::SIMBOT) {
    if (std::abs(gripperInputs.data.torqueCurrentAmps) >= 5.0) {
      hasAlgae = algaeDebouncer.Calculate(
          std::abs(gripperInputs.data.velocityRadsPerSec) <=
          algaeVelocityThresh.Get());
    } else {
      algaeDebouncer.Calculate(hasAlgae);
    }
    if (tunnelInputs.data.torqueCurrentAmps >= 5.0 &&
        tunnelInputs.data.velocityRadsPerSec > 0.0) {
      hasCoral =
          (Constants::GetRobot() != Constants::RobotType::DEVBOT)
              ? coralDebouncer.Calculate(coralSensorInputs.data.valid &&
                                         coralSensorInputs.data.distanceMeters <
                                             coralProxThreshold.Get())
              : coralDebouncer.Calculate(pivotInputs.data.velocityRadPerSec <
                                         1.0);
    } else {
      coralDebouncer.Calculate(hasCoral);
    }
  } else {
    bool algaeButtonPressed = frc::DriverStation::GetStickButtonPressed(2, 1);
    bool coralButtonPressed = frc::DriverStation::GetStickButtonPressed(2, 2);
    if (algaeButtonPressed && !lastAlgaeButtonPressed) {
      hasAlgae = !hasAlgae;
    }
    if (coralButtonPressed && !lastCoralButtonPressed) {
      hasCoral = !hasCoral;
    }
    lastAlgaeButtonPressed = algaeButtonPressed;
    lastCoralButtonPressed = coralButtonPressed;
  }

  frc::SmartDashboard::PutBoolean("Has Coral?", HasCoral());
  frc::SmartDashboard::PutBoolean("Has Algae?", HasAlgae());

  frc::SmartDashboard::PutString("Coral Threshold Offset",
                                 std::to_string(coralThresholdOffset));

  Logger::RecordOutput("Dispenser/CoastOverride", coastOverride());
  Logger::RecordOutput("Dispenser/DisabledOverride", disabledOverride());

  LoggedTracer::Record("Dispenser");
}

void Dispenser::SetGoal(std::function<frc::Rotation2d()> goal) {
  this->goal = [goal]() {
    return frc::math::InputModulus(goal().Radians(), -3.0 * M_PI / 2.0,
                                   M_PI / 2.0);
  };
  atGoal = false;
}

double Dispenser::GetGoal() const { return goal(); }

bool Dispenser::HasAlgae() const {
  return hasAlgae && !disableGamePieceDetectionOverride();
}

bool Dispenser::HasCoral() const {
  return hasCoral || disableGamePieceDetectionOverride();
}

frc::Rotation2d Dispenser::GetPivotAngle() const {
  return pivotInputs.data.position + maxAngle - homingOffset;
}

void Dispenser::ResetHasCoral() {
  hasCoral = false;
  coralDebouncer =
      frc::Debouncer(coralDebounceTime, frc::Debouncer::DebounceType::kRising);
  coralDebouncer.Calculate(false);
}

void Dispenser::ResetHasAlgae() {
  hasAlgae = false;
  algaeDebouncer =
      frc::Debouncer(algaeDebounceTime, frc::Debouncer::DebounceType::kRising);
  algaeDebouncer.Calculate(false);
}

void Dispenser::SetOverrides(
    std::function<bool()> coastOverride, std::function<bool()> disabledOverride,
    std::function<bool()> disableGamePieceDetectionOverride) {
  this->coastOverride = coastOverride;
  this->disabledOverride = disabledOverride;
  this->disableGamePieceDetectionOverride = disableGamePieceDetectionOverride;
}

void Dispenser::SetBrakeMode(bool enabled) {
  if (brakeModeEnabled == enabled)
    return;
  brakeModeEnabled = enabled;
  pivotIO->SetBrakeMode(enabled);
}

frc2::CommandPtr Dispenser::StaticCharacterization() {
  struct StaticCharacterizationState {
    double characterizationOutput = 0.0;
  };

  StaticCharacterizationState state;
  frc::Timer timer;
  return frc2::cmd::RunOnce([this, &timer]() {
           stopProfile = true;
           timer.Restart();
         })
      .AndThen(frc2::cmd::Run([this, &state, &timer]() {
        state.characterizationOutput =
            staticCharacterizationRampRate.Get() * timer.Get().value();
        pivotIO->RunOpenLoop(state.characterizationOutput);
        Logger::RecordOutput("Dispenser/StaticCharacterizationOutput",
                             state.characterizationOutput);
      }))
      .Until([this]() {
        return pivotInputs.data.velocityRadPerSec >=
               staticCharacterizationVelocityThresh.Get();
      })
      .AndThen([this]() { pivotIO->Stop(); })
      .AndThen(frc2::cmd::Idle())
      .FinallyDo([this, &timer, &state]() {
        stopProfile = false;
        timer.Stop();
        Logger::RecordOutput("Dispenser/CharacterizationOutput",
                             state.characterizationOutput);
      });
}

frc2::CommandPtr Dispenser::HomingSequence() {
  return frc2::cmd::RunOnce([this]() {
           stopProfile = true;
           homingDebouncer = frc::Debouncer(
               homingTimeSecs.Get(), frc::Debouncer::DebounceType::kRising);
           homingDebouncer.Calculate(false);
         })
      .AndThen(frc2::cmd::Run([this]() {
        if (disabledOverride() || coastOverride())
          return;
        pivotIO->RunVolts(homingVolts.Get());
      }))
      .RaceWith(
          frc2::cmd::RunOnce([]() {}).AndThen(frc2::cmd::WaitUntil([this]() {
            return homingDebouncer.Calculate(
                std::abs(pivotInputs.data.velocityRadPerSec) <=
                homingVelocityThresh.Get());
          })))
      .AndThen([this]() { homingOffset = pivotInputs.data.position; })
      .FinallyDo([this]() { stopProfile = false; });
}

frc::TrapezoidProfile::State Dispenser::GetSetpoint() const { return setpoint; }

bool Dispenser::IsAtGoal() const { return atGoal; }

void Dispenser::SetShouldEStop(bool shouldEStop) {
  this->shouldEStop = shouldEStop;
}

void Dispenser::SetIsEStopped(bool isEStopped) {
  this->isEStopped = isEStopped;
}

void Dispenser::SetIsIntaking(bool isIntaking) {
  this->isIntaking = isIntaking;
}

void Dispenser::SetTunnelVolts(double tunnelVolts) {
  this->tunnelVolts = tunnelVolts;
}

void Dispenser::SetGripperGoal(GripperGoal gripperGoal) {
  this->gripperGoal = gripperGoal;
}

void Dispenser::SetHasCoral(bool hasCoral) { this->hasCoral = hasCoral; }

void Dispenser::SetDoNotStopIntaking(bool doNotStopIntaking) {
  this->doNotStopIntaking = doNotStopIntaking;
}

void Dispenser::SetCoralThresholdOffset(double coralThresholdOffset) {
  this->coralThresholdOffset = coralThresholdOffset;
}

double Dispenser::GetCoralThresholdOffset() const {
  return coralThresholdOffset;
}
