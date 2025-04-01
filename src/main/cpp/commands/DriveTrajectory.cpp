// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "drivetrajectory.h"

#include <cmath>
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

#include "constants.h"
#include "junction/logger.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "util/alliancefliputil.h"
#include "util/loggedtunablenumber.h"
#include "util/mirrorutil.h"

namespace org::littletonrobotics::frc2025::commands {

LoggedTunableNumber DriveTrajectory::linearkP{
    "DriveTrajectory/LinearkP",
    Constants::GetRobot() == Constants::Robot::COMPBOT ||
            Constants::GetRobot() == Constants::Robot::DEVBOT
        ? 8.0
        : 4.0};
LoggedTunableNumber DriveTrajectory::linearkD{"DriveTrajectory/LinearkD", 0.0};
LoggedTunableNumber DriveTrajectory::thetakP{
    "DriveTrajectory/ThetakP",
    Constants::GetRobot() == Constants::Robot::COMPBOT ||
            Constants::GetRobot() == Constants::Robot::DEVBOT
        ? 4.0
        : 12.0};
LoggedTunableNumber DriveTrajectory::thetakD{"DriveTrajectory/thetakD", 0.0};
LoggedTunableNumber DriveTrajectory::overrideMaxVelocity{
    "DriveTrajectory/OverrideMaxVelocity", frc::Units::DegreesToRadians(360)};
LoggedTunableNumber DriveTrajectory::overrideMaxAcceleration{
    "DriveTrajectory/OverrideMaxAcceleration",
    frc::Units::DegreesToRadians(720)};

DriveTrajectory::DriveTrajectory(
    Drive &drive, subsystems::drive::trajectory::HolonomicTrajectory trajectory)
    : DriveTrajectory{drive, trajectory, false} {}

DriveTrajectory::DriveTrajectory(
    Drive &drive, subsystems::drive::trajectory::HolonomicTrajectory trajectory,
    bool mirror)
    : DriveTrajectory{
          drive, trajectory,
          [&]() { return RobotState::GetInstance().GetEstimatedPose(); },
          mirror} {}

DriveTrajectory::DriveTrajectory(
    Drive &drive, subsystems::drive::trajectory::HolonomicTrajectory trajectory,
    std::function<frc::Pose2d()> robot, bool mirror)
    : drive_{drive}, trajectory_{trajectory}, robotPose_{robot},
      mirror_{mirror}, thetaController_{0.0, 0.0, 0.0},
      overrideThetaController_{0.0, 0.0, 0.0,
                               frc::TrapezoidProfile::Constraints{0.0, 0.0}} {
  thetaController_.EnableContinuousInput(-M_PI, M_PI);
  overrideThetaController_.EnableContinuousInput(-M_PI, M_PI);
  AddRequirements({&drive_});
}

void DriveTrajectory::Initialize() {
  timer_.Restart();

  xController_.Reset();
  yController_.Reset();
  thetaController_.Reset();

  std::vector<frc::Pose2d> trajectoryPoses;
  for (const auto &pose : trajectory_.GetTrajectoryPoses()) {
    trajectoryPoses.push_back(
        AllianceFlipUtil::Apply(mirror_ ? MirrorUtil::Apply(pose) : pose));
  }
  Logger::RecordOutput("Trajectory/TrajectoryPoses", trajectoryPoses);
}

void DriveTrajectory::Execute() {
  if (linearkP.HasChanged(this->GetHashCode()) ||
      linearkD.HasChanged(this->GetHashCode())) {
    xController_.SetPID(linearkP.Get(), 0.0, linearkD.Get());
    yController_.SetPID(linearkP.Get(), 0.0, linearkD.Get());
  }
  if (thetakP.HasChanged(this->GetHashCode()) ||
      thetakD.HasChanged(this->GetHashCode())) {
    thetaController_.SetPID(thetakP.Get(), 0.0, thetakD.Get());
    overrideThetaController_.SetPID(thetakP.Get(), 0.0, thetakD.Get());
  }
  if (overrideMaxVelocity.HasChanged(this->GetHashCode()) ||
      overrideMaxAcceleration.HasChanged(this->GetHashCode())) {
    overrideThetaController_.SetConstraints(frc::TrapezoidProfile::Constraints{
        overrideMaxVelocity.Get(), overrideMaxAcceleration.Get()});
  }

  frc::Pose2d robot = robotPose_();
  auto setpointState = AllianceFlipUtil::Apply(
      mirror_ ? MirrorUtil::Apply(trajectory_.Sample(timer_.Get()))
              : trajectory_.Sample(timer_.Get()));

  double xFeedback = xController_.Calculate(robot.X(), setpointState.x());
  double yFeedback = yController_.Calculate(robot.Y(), setpointState.y());
  double thetaFeedback = thetaController_.Calculate(robot.Rotation().Radians(),
                                                    setpointState.theta());

  std::vector<frc::Translation2d::Vector> moduleForces;
  for (const auto &forces : setpointState.module_forces()) {
    moduleForces.emplace_back(
        frc::Translation2d{forces.fx(), forces.fy()}
            .RotateBy(frc::Rotation2d{setpointState.theta()}.UnaryMinus())
            .ToVector());
  }

  double finalOmega =
      overrideRotation_.has_value()
          ? overrideThetaController_.Calculate(
                robot.Rotation().Radians(), overrideRotation_.value().Radians())
          : thetaFeedback + setpointState.omega();

  drive_.RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                         xFeedback + setpointState.vx(),
                         yFeedback + setpointState.vy(), finalOmega,
                         frc::Rotation2d::Degrees(robot.Rotation().Degrees())),
                     moduleForces);

  Logger::RecordOutput("Trajectory/RobotPose", robot);
  Logger::RecordOutput("Trajectory/SetpointPose", setpointState.pose());
  Logger::RecordOutput(
      "Trajectory/Feedback",
      frc::Pose2d{xFeedback, yFeedback, frc::Rotation2d{thetaFeedback}});
  Logger::RecordOutput("Trajectory/VelocityFeedforward",
                       frc::Pose2d{setpointState.vx(), setpointState.vy(),
                                   frc::Rotation2d{setpointState.omega()}});
  Logger::RecordOutput(
      "Trajectory/OverrideRotation",
      overrideRotation_.has_value()
          ? frc::Pose2d{setpointState.pose().Translation(),
                        frc::Rotation2d{
                            overrideThetaController_.GetSetpoint().position}}
          : frc::Pose2d{});
}

void DriveTrajectory::End(bool interrupted) {}

bool DriveTrajectory::IsFinished() {
  return timer_.Get() >= trajectory_.GetDuration();
}

void DriveTrajectory::SetOverrideRotation(
    std::optional<frc::Rotation2d> rotation) {
  overrideRotation_ = rotation;
  if (!isOverrideRotation_.get()) {
    overrideThetaController_.Reset(
        robotPose_().Rotation().Radians(),
        RobotState::GetInstance().GetRobotVelocity().omega);
  }
  isOverrideRotation_.set(rotation.has_value());
}

} // namespace org::littletonrobotics::frc2025::commands