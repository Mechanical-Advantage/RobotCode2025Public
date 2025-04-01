// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Units.h>
#include <frc2/command/CommandBase.h>

#include "junction/autologoutput.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "subsystems/drive/trajectory/holonomictrajectory.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

class DriveTrajectory : public frc2::CommandBase {
public:
  DriveTrajectory(
      Drive &drive,
      subsystems::drive::trajectory::HolonomicTrajectory trajectory);
  DriveTrajectory(Drive &drive,
                  subsystems::drive::trajectory::HolonomicTrajectory trajectory,
                  bool mirror);
  DriveTrajectory(Drive &drive,
                  subsystems::drive::trajectory::HolonomicTrajectory trajectory,
                  std::function<frc::Pose2d()> robot, bool mirror);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  void SetOverrideRotation(std::optional<frc::Rotation2d> rotation);

private:
  static LoggedTunableNumber linearkP;
  static LoggedTunableNumber linearkD;
  static LoggedTunableNumber thetakP;
  static LoggedTunableNumber thetakD;
  static LoggedTunableNumber overrideMaxVelocity;
  static LoggedTunableNumber overrideMaxAcceleration;

  Drive &drive_;
  subsystems::drive::trajectory::HolonomicTrajectory trajectory_;
  frc::Timer timer_;
  std::function<frc::Pose2d()> robotPose_;
  bool mirror_;

  std::optional<frc::Rotation2d> overrideRotation_;
  AutoLogOutput<bool> isOverrideRotation_{false};

  frc::PIDController xController_{0.0, 0.0, 0.0};
  frc::PIDController yController_{0.0, 0.0, 0.0};
  frc::PIDController thetaController_{0.0, 0.0, 0.0};
  frc::ProfiledPIDController overrideThetaController_{
      0.0, 0.0, 0.0, frc::TrapezoidProfile::Constraints{0.0, 0.0}};
};

} // namespace org::littletonrobotics::frc2025::commands