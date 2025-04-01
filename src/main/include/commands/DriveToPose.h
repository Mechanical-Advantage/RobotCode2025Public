// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Units.h>
#include <frc2/command/CommandBase.h>

#include "constants.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

class DriveToPose : public frc2::CommandBase {
public:
  DriveToPose(Drive &drive, std::function<frc::Pose2d()> target);
  DriveToPose(Drive &drive, std::function<frc::Pose2d()> target,
              std::function<frc::Pose2d()> robot);
  DriveToPose(Drive &drive, std::function<frc::Pose2d()> target,
              std::function<frc::Pose2d()> robot,
              std::function<frc::Translation2d()> linearFF,
              std::function<double()> omegaFF);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

  bool AtGoal();
  bool WithinTolerance(double driveTolerance, frc::Rotation2d thetaTolerance);
  bool IsRunning() const { return running_; }

private:
  static LoggedTunableNumber drivekP;
  static LoggedTunableNumber drivekD;
  static LoggedTunableNumber thetakP;
  static LoggedTunableNumber thetakD;
  static LoggedTunableNumber driveMaxVelocity;
  static LoggedTunableNumber driveMaxVelocitySlow;
  static LoggedTunableNumber driveMaxAcceleration;
  static LoggedTunableNumber thetaMaxVelocity;
  static LoggedTunableNumber thetaMaxAcceleration;
  static LoggedTunableNumber driveTolerance;
  static LoggedTunableNumber thetaTolerance;
  static LoggedTunableNumber ffMinRadius;
  static LoggedTunableNumber ffMaxRadius;

  Drive &drive_;
  std::function<frc::Pose2d()> target_;
  std::function<frc::Pose2d()> robot_;
  std::function<frc::Translation2d()> linearFF_;
  std::function<double()> omegaFF_;

  frc::ProfiledPIDController driveController_;
  frc::ProfiledPIDController thetaController_;

  frc::Translation2d lastSetpointTranslation_{frc::Translation2d{}};
  double driveErrorAbs_ = 0.0;
  double thetaErrorAbs_ = 0.0;
  bool running_ = false;
};

} // namespace org::littletonrobotics::frc2025::commands