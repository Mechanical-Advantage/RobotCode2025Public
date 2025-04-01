// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "drivetopose.h"

#include <cmath>
#include <functional>
#include <limits>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Units.h>
#include <frc2/command/CommandHelper.h>

#include "constants.h"
#include "junction/logger.h"
#include "robotstate.h"
#include "subsystems/drive/drive.h"
#include "subsystems/drive/driveconstants.h"
#include "util/geomutil.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

LoggedTunableNumber DriveToPose::drivekP{"DriveToPose/DrivekP", 0.8};
LoggedTunableNumber DriveToPose::drivekD{"DriveToPose/DrivekD", 0.0};
LoggedTunableNumber DriveToPose::thetakP{"DriveToPose/ThetakP", 4.0};
LoggedTunableNumber DriveToPose::thetakD{"DriveToPose/ThetakD", 0.0};
LoggedTunableNumber DriveToPose::driveMaxVelocity{
    "DriveToPose/DriveMaxVelocity", 3.8};
LoggedTunableNumber DriveToPose::driveMaxVelocitySlow{
    "DriveToPose/DriveMaxVelocitySlow"};
LoggedTunableNumber DriveToPose::driveMaxAcceleration{
    "DriveToPose/DriveMaxAcceleration", 3.0};
LoggedTunableNumber DriveToPose::thetaMaxVelocity{
    "DriveToPose/ThetaMaxVelocity", frc::Units::DegreesToRadians(360.0)};
LoggedTunableNumber DriveToPose::thetaMaxAcceleration{
    "DriveToPose/ThetaMaxAcceleration", 8.0};
LoggedTunableNumber DriveToPose::driveTolerance{"DriveToPose/DriveTolerance",
                                                0.01};
LoggedTunableNumber DriveToPose::thetaTolerance{
    "DriveToPose/ThetaTolerance", frc::Units::DegreesToRadians(1.0)};
LoggedTunableNumber DriveToPose::ffMinRadius{"DriveToPose/FFMinRadius", 0.05};
LoggedTunableNumber DriveToPose::ffMaxRadius{"DriveToPose/FFMaxRadius", 0.1};

DriveToPose::DriveToPose(Drive &drive, std::function<frc::Pose2d()> target)
    : drive_{drive}, target_{target},
      driveController_{0.0, 0.0, 0.0,
                       frc::TrapezoidProfile::Constraints{0.0, 0.0},
                       Constants::loopPeriodSecs},
      thetaController_{0.0, 0.0, 0.0,
                       frc::TrapezoidProfile::Constraints{0.0, 0.0},
                       Constants::loopPeriodSecs} {
  thetaController_.EnableContinuousInput(-M_PI, M_PI);
  AddRequirements({&drive_});
}

DriveToPose::DriveToPose(Drive &drive, std::function<frc::Pose2d()> target,
                         std::function<frc::Pose2d()> robot)
    : DriveToPose{drive, target} {
  robot_ = robot;
}

DriveToPose::DriveToPose(Drive &drive, std::function<frc::Pose2d()> target,
                         std::function<frc::Pose2d()> robot,
                         std::function<frc::Translation2d()> linearFF,
                         std::function<double()> omegaFF)
    : DriveToPose{drive, target, robot} {
  linearFF_ = linearFF;
  omegaFF_ = omegaFF;
}

void DriveToPose::Initialize() {
  frc::Pose2d currentPose = robot_();
  frc::ChassisSpeeds fieldVelocity =
      RobotState::GetInstance().GetFieldVelocity();
  frc::Translation2d linearFieldVelocity{fieldVelocity.vx, fieldVelocity.vy};

  driveController_.Reset(
      currentPose.Translation().Distance(target_().Translation()),
      std::min(0.0, -linearFieldVelocity
                         .RotateBy(target_()
                                       .Translation()
                                       .Minus(currentPose.Translation())
                                       .Angle()
                                       .UnaryMinus())
                         .X()));
  thetaController_.Reset(currentPose.Rotation().Radians(), fieldVelocity.omega);
  lastSetpointTranslation_ = currentPose.Translation();
}

void DriveToPose::Execute() {
  running_ = true;

  if (driveMaxVelocity.HasChanged(this->GetHashCode()) ||
      driveMaxVelocitySlow.HasChanged(this->GetHashCode()) ||
      driveMaxAcceleration.HasChanged(this->GetHashCode()) ||
      driveTolerance.HasChanged(this->GetHashCode()) ||
      thetaMaxVelocity.HasChanged(this->GetHashCode()) ||
      thetaMaxAcceleration.HasChanged(this->GetHashCode()) ||
      thetaTolerance.HasChanged(this->GetHashCode()) ||
      drivekP.HasChanged(this->GetHashCode()) ||
      drivekD.HasChanged(this->GetHashCode()) ||
      thetakP.HasChanged(this->GetHashCode()) ||
      thetakD.HasChanged(this->GetHashCode())) {
    driveController_.SetP(drivekP.Get());
    driveController_.SetD(drivekD.Get());
    driveController_.SetConstraints(frc::TrapezoidProfile::Constraints{
        driveMaxVelocity.Get(), driveMaxAcceleration.Get()});
    driveController_.SetTolerance(driveTolerance.Get());
    thetaController_.SetP(thetakP.Get());
    thetaController_.SetD(thetakD.Get());
    thetaController_.SetConstraints(frc::TrapezoidProfile::Constraints{
        thetaMaxVelocity.Get(), thetaMaxAcceleration.Get()});
    thetaController_.SetTolerance(thetaTolerance.Get());
  }

  frc::Pose2d currentPose = robot_();
  frc::Pose2d targetPose = target_();

  double currentDistance =
      currentPose.Translation().Distance(targetPose.Translation());
  double ffScaler = std::clamp((currentDistance - ffMinRadius.Get()) /
                                   (ffMaxRadius.Get() - ffMinRadius.Get()),
                               0.0, 1.0);
  driveErrorAbs_ = currentDistance;
  driveController_.Reset(
      lastSetpointTranslation_.Distance(targetPose.Translation()),
      driveController_.GetSetpoint().velocity);
  double driveVelocityScalar =
      driveController_.GetSetpoint().velocity * ffScaler +
      driveController_.Calculate(driveErrorAbs_, 0.0);
  if (currentDistance < driveController_.GetPositionTolerance())
    driveVelocityScalar = 0.0;
  lastSetpointTranslation_ =
      frc::Pose2d{
          targetPose.Translation(),
          frc::Rotation2d{std::atan2(
              currentPose.Translation().Y() - targetPose.Translation().Y(),
              currentPose.Translation().X() - targetPose.Translation().X())}}
          .TransformBy(GeomUtil::ToTransform2d(
              driveController_.GetSetpoint().position, 0.0))
          .Translation();

  double thetaVelocity =
      thetaController_.GetSetpoint().velocity * ffScaler +
      thetaController_.Calculate(currentPose.Rotation().Radians(),
                                 targetPose.Rotation().Radians());
  thetaErrorAbs_ =
      std::abs(currentPose.Rotation().Minus(targetPose.Rotation()).Radians());
  if (thetaErrorAbs_ < thetaController_.GetPositionTolerance())
    thetaVelocity = 0.0;

  frc::Translation2d driveVelocity =
      frc::Pose2d{
          frc::Translation2d{},
          frc::Rotation2d{std::atan2(
              currentPose.Translation().Y() - targetPose.Translation().Y(),
              currentPose.Translation().X() - targetPose.Translation().X())}}
          .TransformBy(GeomUtil::ToTransform2d(driveVelocityScalar, 0.0))
          .Translation();

  const double linearS = linearFF_().Norm() * 3.0;
  const double thetaS = std::abs(omegaFF_()) * 3.0;
  driveVelocity = driveVelocity.Interpolate(
      linearFF_().Times(DriveConstants::maxLinearSpeed), linearS);
  thetaVelocity = std::lerp(
      thetaVelocity, omegaFF_() * DriveConstants::maxAngularSpeed, thetaS);

  drive_.RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      driveVelocity.X(), driveVelocity.Y(), thetaVelocity,
      currentPose.Rotation()));

  Logger::RecordOutput("DriveToPose/DistanceMeasured", currentDistance);
  Logger::RecordOutput("DriveToPose/DistanceSetpoint",
                       driveController_.GetSetpoint().position);
  Logger::RecordOutput("DriveToPose/ThetaMeasured",
                       currentPose.Rotation().Radians());
  Logger::RecordOutput("DriveToPose/ThetaSetpoint",
                       thetaController_.GetSetpoint().position);
  Logger::RecordOutput(
      "DriveToPose/Setpoint",
      std::vector<frc::Pose2d>{frc::Pose2d{
          lastSetpointTranslation_,
          frc::Rotation2d{thetaController_.GetSetpoint().position}}});
  Logger::RecordOutput("DriveToPose/Goal",
                       std::vector<frc::Pose2d>{targetPose});
}

void DriveToPose::End(bool interrupted) {
  drive_.Stop();
  running_ = false;
  Logger::RecordOutput("DriveToPose/Setpoint", std::vector<frc::Pose2d>{});
  Logger::RecordOutput("DriveToPose/Goal", std::vector<frc::Pose2d>{});
}

bool DriveToPose::AtGoal() {
  return running_ && driveController_.AtGoal() && thetaController_.AtGoal();
}

bool DriveToPose::WithinTolerance(double driveTolerance,
                                  frc::Rotation2d thetaTolerance) {
  return running_ && std::abs(driveErrorAbs_) < driveTolerance &&
         std::abs(thetaErrorAbs_) < thetaTolerance.Radians();
}

} // namespace org::littletonrobotics::frc2025::commands
