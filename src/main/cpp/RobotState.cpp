// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "RobotState.h"

#include <cmath>
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
#include <frc/util/Units.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "junction/Logger.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/vision/VisionConstants.h"
#include "util/AllianceFlipUtil.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"

RobotState *RobotState::instance = nullptr;

LoggedTunableNumber RobotState::txTyObservationStaleSecs(
    "RobotState/TxTyObservationStaleSeconds", 0.5);
LoggedTunableNumber
    RobotState::minDistanceTagPoseBlend("RobotState/MinDistanceTagPoseBlend",
                                        frc::Units::InchesToMeters(24.0));
LoggedTunableNumber
    RobotState::maxDistanceTagPoseBlend("RobotState/MaxDistanceTagPoseBlend",
                                        frc::Units::InchesToMeters(36.0));

std::map<int, frc::Pose2d> RobotState::tagPoses2d;

RobotState &RobotState::GetInstance() {
  if (instance == nullptr) {
    instance = new RobotState();
  }
  return *instance;
}

RobotState::RobotState() {
  for (int i = 0; i < 3; ++i) {
    qStdDevs(i, 0) = std::pow(odometryStateStdDevs(i, 0), 2);
  }
  kinematics = frc::SwerveDriveKinematics(DriveConstants::moduleTranslations);

  for (int i = 1; i <= FieldConstants::aprilTagCount; i++) {
    tagPoses2d[i] =
        FieldConstants::defaultAprilTagType.GetLayout().GetTagPose(i).value_or(
            frc::Pose2d());
    txTyPoses[i] = TxTyPoseRecord(frc::Pose2d(), INFINITY, -1.0);
  }
}

void RobotState::ResetPose(frc::Pose2d pose) {
  gyroOffset = pose.Rotation() - (odometryPose.Rotation() - gyroOffset);
  estimatedPose = pose;
  odometryPose = pose;
  poseBuffer.Clear();
}

void RobotState::AddOdometryObservation(
    const OdometryObservation &observation) {
  frc::Twist2d twist =
      kinematics.ToTwist2d(lastWheelPositions, observation.wheelPositions);
  lastWheelPositions = observation.wheelPositions;
  frc::Pose2d lastOdometryPose = odometryPose;
  odometryPose = odometryPose.Exp(twist);
  if (observation.gyroAngle.has_value()) {
    frc::Rotation2d angle = observation.gyroAngle.value() + gyroOffset;
    odometryPose = frc::Pose2d(odometryPose.Translation(), angle);
  }
  poseBuffer.AddSample(observation.timestamp, odometryPose);
  frc::Twist2d finalTwist = lastOdometryPose.Log(odometryPose);
  estimatedPose = estimatedPose.Exp(finalTwist);
}

void RobotState::AddVisionObservation(const VisionObservation &observation) {
  if (poseBuffer.GetInternalBuffer().rbegin()->first - poseBufferSizeSec >
      observation.timestamp) {
    return;
  }

  auto sample = poseBuffer.GetSample(observation.timestamp);
  if (!sample.has_value()) {
    return;
  }

  frc::Transform2d sampleToOdometryTransform(sample.value(), odometryPose);
  frc::Transform2d odometryToSampleTransform(odometryPose, sample.value());
  frc::Pose2d estimateAtTime = estimatedPose + odometryToSampleTransform;

  std::array<double, 3> r;
  for (int i = 0; i < 3; ++i) {
    r[i] = observation.stdDevs(i, 0) * observation.stdDevs(i, 0);
  }

  Eigen::Matrix3d visionK = Eigen::Matrix3d::Zero();
  for (int row = 0; row < 3; ++row) {
    double stdDev = qStdDevs(row, 0);
    if (stdDev == 0.0) {
      visionK(row, row) = 0.0;
    } else {
      visionK(row, row) = stdDev / (stdDev + std::sqrt(stdDev * r[row]));
    }
  }

  frc::Transform2d transform(estimateAtTime, observation.visionPose);
  Eigen::Vector3d kTimesTransform =
      visionK * Eigen::Vector3d(transform.X(), transform.Y(),
                                transform.Rotation().Radians());
  frc::Transform2d scaledTransform(kTimesTransform(0), kTimesTransform(1),
                                   frc::Rotation2d(kTimesTransform(2)));

  estimatedPose = estimateAtTime + scaledTransform + sampleToOdometryTransform;
}

void RobotState::AddTxTyObservation(const TxTyObservation &observation) {
  if (txTyPoses.count(observation.tagId) &&
      txTyPoses[observation.tagId].timestamp >= observation.timestamp) {
    return;
  }

  auto sample = poseBuffer.GetSample(observation.timestamp);
  if (!sample.has_value()) {
    return;
  }
  frc::Rotation2d robotRotation =
      estimatedPose.TransformBy(frc::Transform2d(odometryPose, sample.value()))
          .Rotation();

  double tx = 0.0;
  double ty = 0.0;
  for (int i = 0; i < 4; i++) {
    tx += observation.tx[i];
    ty += observation.ty[i];
  }
  tx /= 4.0;
  ty /= 4.0;

  frc::Pose3d cameraPose =
      VisionConstants::cameras[observation.camera].pose.value();

  frc::Translation2d camToTagTranslation =
      frc::Pose3d(frc::Translation3d(), frc::Rotation3d(0, ty, -tx))
          .TransformBy(
              frc::Transform3d(frc::Translation3d(observation.distance, 0, 0),
                               frc::Rotation3d()))
          .Translation()
          .RotateBy(frc::Rotation3d(0, cameraPose.Rotation().Y(), 0))
          .ToTranslation2d();
  frc::Rotation2d camToTagRotation = robotRotation +
                                     cameraPose.ToPose2d().Rotation() +
                                     camToTagTranslation.Angle();
  frc::Pose2d tagPose2d = tagPoses2d[observation.tagId];
  if (tagPose2d == frc::Pose2d())
    return;
  frc::Translation2d fieldToCameraTranslation =
      frc::Pose2d(tagPose2d.Translation(),
                  camToTagRotation + frc::Rotation2d::Degrees(180.0))
          .TransformBy(GeomUtil::ToTransform2d(camToTagTranslation.Norm(), 0.0))
          .Translation();
  frc::Pose2d robotPose =
      frc::Pose2d(fieldToCameraTranslation,
                  robotRotation + cameraPose.ToPose2d().Rotation())
          .TransformBy(frc::Transform2d(cameraPose.ToPose2d(), frc::Pose2d()));
  robotPose = frc::Pose2d(robotPose.Translation(), robotRotation);

  txTyPoses[observation.tagId] = TxTyPoseRecord(
      robotPose, camToTagTranslation.Norm(), observation.timestamp);
}

void RobotState::AddDriveSpeeds(const frc::ChassisSpeeds &speeds) {
  robotVelocity = speeds;
}

frc::ChassisSpeeds RobotState::GetFieldVelocity() {
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotVelocity,
                                                     GetRotation());
}

std::optional<frc::Pose2d> RobotState::GetTxTyPose(int tagId) {
  if (!txTyPoses.count(tagId)) {
    return std::nullopt;
  }
  TxTyPoseRecord data = txTyPoses[tagId];
  if (frc::Timer::GetFPGATimestamp() - data.timestamp >=
      txTyObservationStaleSecs.Get()) {
    return std::nullopt;
  }
  auto sample = poseBuffer.GetSample(data.timestamp);
  return sample.has_value()
             ? std::optional<frc::Pose2d>(
                   data.pose + frc::Transform2d(sample.value(), odometryPose))
             : std::nullopt;
}

frc::Pose2d RobotState::GetReefPose(int face, const frc::Pose2d &finalPose) {
  bool isRed = AllianceFlipUtil::ShouldFlip();
  int tagId;
  switch (face) {
  case 1:
    tagId = isRed ? 6 : 19;
    break;
  case 2:
    tagId = isRed ? 11 : 20;
    break;
  case 3:
    tagId = isRed ? 10 : 21;
    break;
  case 4:
    tagId = isRed ? 9 : 22;
    break;
  case 5:
    tagId = isRed ? 8 : 17;
    break;
  default:
    tagId = isRed ? 7 : 18;
    break;
  }

  auto tagPose = GetTxTyPose(tagId);
  if (!tagPose.has_value()) {
    return GetInstance().GetEstimatedPose();
  }

  double t = frc::MathUtil::Clamp(
      (GetEstimatedPose().Translation().Distance(finalPose.Translation()) -
       minDistanceTagPoseBlend.Get()) /
          (maxDistanceTagPoseBlend.Get() - minDistanceTagPoseBlend.Get()),
      0.0, 1.0);
  return GetEstimatedPose().Interpolate(tagPose.value(), 1.0 - t);
}

void RobotState::AddAlgaeTxTyObservation(
    const AlgaeTxTyObservation &observation) {
  auto oldOdometryPose = poseBuffer.GetSample(observation.timestamp);
  if (!oldOdometryPose.has_value()) {
    return;
  }
  frc::Pose2d fieldToRobot = estimatedPose.TransformBy(
      frc::Transform2d(odometryPose, oldOdometryPose.value()));
  frc::Pose3d robotToCamera =
      VisionConstants::cameras[observation.camera].pose.value();

  double tx = 0.0;
  double ty = 0.0;
  for (int i = 0; i < 4; i++) {
    tx += observation.tx[i];
    ty += observation.ty[i];
  }
  tx /= 4.0;
  ty /= 4.0;
  double cameraToAlgaeAngle = -robotToCamera.Rotation().Y() - ty;
  if (cameraToAlgaeAngle >= 0) {
    return;
  }
  double cameraToAlgaeNorm =
      (FieldConstants::algaeDiameter / 2 - robotToCamera.Z()) /
      std::tan(-robotToCamera.Rotation().Y() - ty) / std::cos(-tx);
  frc::Pose2d fieldToCamera =
      fieldToRobot.TransformBy(robotToCamera.ToPose2d().ToTransform2d());
  frc::Pose2d fieldToAlgae =
      fieldToCamera
          .TransformBy(
              frc::Transform2d(frc::Translation2d(), frc::Rotation2d(-tx)))
          .TransformBy(frc::Transform2d(
              frc::Translation2d(cameraToAlgaeNorm, 0), frc::Rotation2d()));
  frc::Translation2d fieldToAlgaeTranslation2d = fieldToAlgae.Translation();
  AlgaePoseRecord algaePoseRecord(fieldToAlgaeTranslation2d,
                                  observation.timestamp);

  std::set<AlgaePoseRecord> newAlgaePoses;
  for (const auto &x : algaePoses) {
    if (x.translation.Distance(fieldToAlgaeTranslation2d) >
        FieldConstants::algaeDiameter * 0.8) {
      newAlgaePoses.insert(x);
    }
  }
  newAlgaePoses.insert(algaePoseRecord);
  algaePoses = newAlgaePoses;
}

std::set<frc::Translation2d> RobotState::GetAlgaeTranslations() {
  std::set<frc::Translation2d> result;
  for (const auto &x : algaePoses) {
    if (frc::Timer::GetFPGATimestamp() - x.timestamp < algaePersistanceTime) {
      result.insert(x.translation);
    }
  }
  return result;
}

frc::Rotation2d RobotState::GetRotation() { return estimatedPose.Rotation(); }

void RobotState::PeriodicLog() {
  std::array<frc::Pose2d, FieldConstants::aprilTagCount + 1> tagPoses;
  for (int i = 0; i < FieldConstants::aprilTagCount + 1; i++) {
    tagPoses[i] = GetTxTyPose(i).value_or(frc::Pose2d());
  }
  Logger::RecordOutput("RobotState/TxTyPoses", tagPoses);

  std::vector<frc::Translation3d> algaeTranslations;
  for (const auto &translation : GetAlgaeTranslations()) {
    algaeTranslations.emplace_back(translation.X(), translation.Y(),
                                   FieldConstants::algaeDiameter / 2);
  }
  Logger::RecordOutput("RobotState/AlgaePoses", algaeTranslations);
}
