// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include "frc/Timer.h"
#include "frc/filter/Debouncer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc/units/angle.h"
#include "frc/units/angular_velocity.h"
#include "frc/units/time.h"
#include "frc/units/voltage.h"
#include "frc2/command/Command.h"
#include "frc2/command/Commands.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/Robot.h"
#include "org/littletonrobotics/frc2025/subsystems/leds/Leds.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/sensors/CoralSensorIO.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/frc2025/util/LoggedTunableNumber.h"
#include "org/littletonrobotics/junction/AutoLog.h"

class Dispenser {
public:
  static constexpr frc::Rotation2d minAngle = frc::Rotation2d::Degrees(-90.0);
  static constexpr frc::Rotation2d maxAngle = frc::Rotation2d::Degrees(20.5);

  static LoggedTunableNumber kP;
  static LoggedTunableNumber kD;
  static LoggedTunableNumber kS;
  static LoggedTunableNumber kG;
  static LoggedTunableNumber maxVelocityDegPerSec;
  static LoggedTunableNumber maxAccelerationDegPerSec2;
  static LoggedTunableNumber algaeMaxVelocityDegPerSec;
  static LoggedTunableNumber algaeMaxAccelerationDegPerSec2;
  static LoggedTunableNumber staticCharacterizationVelocityThresh;
  static LoggedTunableNumber staticCharacterizationRampRate;
  static LoggedTunableNumber algaeVelocityThresh;
  static LoggedTunableNumber coralProxThreshold;
  static LoggedTunableNumber gripperHoldVolts;
  static LoggedTunableNumber gripperIntakeVolts;
  static LoggedTunableNumber gripperEjectVolts;
  static LoggedTunableNumber gripperL1EjectVolts;
  static LoggedTunableNumber gripperCurrentLimit;
  static LoggedTunableNumber tunnelDispenseVolts;
  static LoggedTunableNumber tunnelL1DispenseVolts;
  static LoggedTunableNumber tunnelIntakeVolts;
  static LoggedTunableNumber tolerance;
  static LoggedTunableNumber intakeReverseVolts;
  static LoggedTunableNumber intakeReverseTime;
  static LoggedTunableNumber homingTimeSecs;
  static LoggedTunableNumber homingVolts;
  static LoggedTunableNumber homingVelocityThresh;

  enum class GripperGoal { IDLE, GRIP, EJECT, L1_EJECT };

  Dispenser(PivotIO &pivotIO, RollerSystemIO &tunnelIO,
            RollerSystemIO &gripperIO, CoralSensorIO &coralSensorIO);
  ~Dispenser() = default;

  void Periodic();

  void SetGoal(std::function<frc::Rotation2d()> goal);
  double GetGoal() const;

  bool HasAlgae() const;
  bool HasCoral() const;

  frc::Rotation2d GetPivotAngle() const;

  void ResetHasCoral();
  void ResetHasAlgae();

  void SetOverrides(std::function<bool()> coastOverride,
                    std::function<bool()> disabledOverride,
                    std::function<bool()> disableGamePieceDetectionOverride);

  frc2::CommandPtr StaticCharacterization();
  frc2::CommandPtr HomingSequence();

  frc::TrapezoidProfile::State GetSetpoint() const;
  bool IsAtGoal() const;
  void SetShouldEStop(bool shouldEStop);
  void SetIsEStopped(bool isEStopped);
  void SetIsIntaking(bool isIntaking);
  void SetTunnelVolts(double tunnelVolts);
  void SetGripperGoal(GripperGoal gripperGoal);
  void SetHasCoral(bool hasCoral);
  void SetDoNotStopIntaking(bool doNotStopIntaking);
  void SetCoralThresholdOffset(double coralThresholdOffset);
  double GetCoralThresholdOffset() const;

private:
  PivotIO *pivotIO;
  PivotIO::PivotIOInputs pivotInputs;
  RollerSystemIO *tunnelIO;
  RollerSystemIO::RollerSystemIOInputs tunnelInputs;
  RollerSystemIO *gripperIO;
  RollerSystemIO::RollerSystemIOInputs gripperInputs;
  CoralSensorIO *coralSensorIO;
  CoralSensorIO::CoralSensorIOInputs coralSensorInputs;

  std::function<bool()> coastOverride = []() { return false; };
  std::function<bool()> disabledOverride = []() { return false; };
  std::function<bool()> disableGamePieceDetectionOverride = []() {
    return false;
  };

  bool brakeModeEnabled = true;

  frc::TrapezoidProfile profile;
  frc::TrapezoidProfile algaeProfile;
  frc::TrapezoidProfile::State setpoint = frc::TrapezoidProfile::State();
  std::function<double()> goal = []() { return 0.0; };
  bool stopProfile = false;
  bool shouldEStop = false;
  bool isEStopped = false;
  bool isIntaking = false;
  frc::Timer intakingReverseTimer;

  bool atGoal = false;

  double tunnelVolts = 0.0;
  GripperGoal gripperGoal = GripperGoal::IDLE;

  bool hasCoral = false;
  bool hasAlgae = false;
  bool doNotStopIntaking = false;

  static constexpr double coralDebounceTime = 0.1;
  static constexpr double algaeDebounceTime = 0.4;
  frc::Debouncer coralDebouncer =
      frc::Debouncer(coralDebounceTime, frc::Debouncer::DebounceType::kRising);
  frc::Debouncer algaeDebouncer =
      frc::Debouncer(algaeDebounceTime, frc::Debouncer::DebounceType::kRising);
  frc::Debouncer toleranceDebouncer =
      frc::Debouncer(0.25, frc::Debouncer::DebounceType::kRising);
  frc::Debouncer homingDebouncer = frc::Debouncer(0.25);

  double coralThresholdOffset = 0.0;
  frc::Rotation2d homingOffset = frc::Rotation2d::Zero();

  frc::Alert pivotMotorDisconnectedAlert = frc::Alert(
      "Dispenser pivot motor disconnected!", frc::Alert::AlertType::kWarning);
  frc::Alert tunnelDisconnectedAlert = frc::Alert(
      "Dispenser tunnel disconnected!", frc::Alert::AlertType::kWarning);
  frc::Alert gripperDisconnectedAlert = frc::Alert(
      "Dispenser gripper disconnected!", frc::Alert::AlertType::kWarning);

  bool lastAlgaeButtonPressed = false;
  bool lastCoralButtonPressed = false;

  void SetBrakeMode(bool enabled);

  struct StaticCharacterizationState {
    double characterizationOutput = 0.0;
  };

  AUTO_LOG_OUTPUT(brakeModeEnabled);
  AUTO_LOG_OUTPUT(atGoal);
  AUTO_LOG_OUTPUT(getPivotAngle());
  AUTO_LOG_OUTPUT(hasAlgae());
  AUTO_LOG_OUTPUT(hasCoral());
  AUTO_LOG_OUTPUT(coralThresholdOffset);
  AUTO_LOG_OUTPUT(homingOffset);
};