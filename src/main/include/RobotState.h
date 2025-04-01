// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/numbers/N1.h>
#include <frc/numbers/N3.h>
#include <frc/util/Units.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "junction/AutoLogOutput.h"
#include "junction/Logger.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/vision/VisionConstants.h"
#include "util/AllianceFlipUtil.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"

class RobotState {
public:
  static RobotState &GetInstance();

  void ResetPose(frc::Pose2d pose);
  void AddOdometryObservation(const OdometryObservation &observation);
  void AddVisionObservation(const VisionObservation &observation);
  void AddTxTyObservation(const TxTyObservation &observation);
  void AddDriveSpeeds(const frc::ChassisSpeeds &speeds);
  frc::ChassisSpeeds GetFieldVelocity();
  std::optional<frc::Pose2d> GetTxTyPose(int tagId);
  frc::Pose2d GetReefPose(int face, const frc::Pose2d &finalPose);
  void AddAlgaeTxTyObservation(const AlgaeTxTyObservation &observation);
  std::set<frc::Translation2d> GetAlgaeTranslations();
  frc::Rotation2d GetRotation();
  void PeriodicLog();

  struct OdometryObservation {
    std::array<frc::SwerveModulePosition, 4> wheelPositions;
    std::optional<frc::Rotation2d> gyroAngle;
    double timestamp;
  };

  struct VisionObservation {
    frc::Pose2d visionPose;
    double timestamp;
    Eigen::Matrix<double, 3, 1> stdDevs;
  };

  struct TxTyObservation {
    int tagId;
    int camera;
    std::array<double, 4> tx;
    std::array<double, 4> ty;
    double distance;
    double timestamp;
  };

  struct TxTyPoseRecord {
    frc::Pose2d pose;
    double distance;
    double timestamp;
  };

  struct AlgaeTxTyObservation {
    int camera;
    std::array<double, 4> tx;
    std::array<double, 4> ty;
    double timestamp;
  };

  struct AlgaePoseRecord {
    frc::Translation2d translation;
    double timestamp;
  };

  frc::Pose2d GetOdometryPose() const { return odometryPose; }
  frc::Pose2d GetEstimatedPose() const { return estimatedPose; }
  frc::ChassisSpeeds GetRobotVelocity() const { return robotVelocity; }
  void SetDistanceToBranch(std::optional<double> distance) {
    distanceToBranch = distance;
  }
  std::optional<double> GetDistanceToBranch() const { return distanceToBranch; }

private:
  RobotState();

  static RobotState *instance;

  static constexpr double poseBufferSizeSec = 2.0;
  static constexpr double algaePersistanceTime = 2.0;
  static Eigen::Matrix<double, 3, 1> odometryStateStdDevs =
      Eigen::Matrix<double, 3, 1>::Map((double[]){0.003, 0.003, 0.002});
  static std::map<int, frc::Pose2d> tagPoses2d;

  static LoggedTunableNumber txTyObservationStaleSecs;
  static LoggedTunableNumber minDistanceTagPoseBlend;
  static LoggedTunableNumber maxDistanceTagPoseBlend;

  frc::Pose2d odometryPose;
  frc::Pose2d estimatedPose;

  frc::TimeInterpolatableBuffer<frc::Pose2d> poseBuffer =
      frc::TimeInterpolatableBuffer<frc::Pose2d>::CreateBuffer(
          poseBufferSizeSec);
  Eigen::Matrix<double, 3, 1> qStdDevs;

  frc::SwerveDriveKinematics kinematics;
  std::array<frc::SwerveModulePosition, 4> lastWheelPositions = {
      frc::SwerveModulePosition(), frc::SwerveModulePosition(),
      frc::SwerveModulePosition(), frc::SwerveModulePosition()};
  frc::Rotation2d gyroOffset = frc::Rotation2d::Degrees(0.0);

  std::map<int, TxTyPoseRecord> txTyPoses;
  std::set<AlgaePoseRecord> algaePoses;

  frc::ChassisSpeeds robotVelocity;
  std::optional<double> distanceToBranch;

  JUnction::AutoLogOutput<frc::Pose2d> odometryPoseLog{
      "RobotState/OdometryPose"};
  JUnction::AutoLogOutput<frc::Pose2d> estimatedPoseLog{
      "RobotState/EstimatedPose"};
  JUnction::AutoLogOutput<frc::ChassisSpeeds> robotVelocityLog{
      "RobotState/RobotVelocity"};
  JUnction::AutoLogOutput<frc::ChassisSpeeds> fieldVelocityLog{
      "RobotState/FieldVelocity"};
};