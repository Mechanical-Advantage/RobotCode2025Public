// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "FieldConstants.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <frc/Filesystem.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/util/Units.h>

#include "Constants.h"

std::array<frc::Pose2d, 6> FieldConstants::Reef::centerFaces;
std::vector<std::map<FieldConstants::ReefLevel, frc::Pose3d>>
    FieldConstants::Reef::branchPositions;
std::vector<std::map<FieldConstants::ReefLevel, frc::Pose2d>>
    FieldConstants::Reef::branchPositions2d;

FieldConstants::ReefLevelData
FieldConstants::GetReefLevelData(ReefLevel level) {
  switch (level) {
  case ReefLevel::kL1:
    return {frc::Units::InchesToMeters(25.0), 0};
  case ReefLevel::kL2:
    return {frc::Units::InchesToMeters(
                31.875 - std::cos(frc::Units::DegreesToRadians(35.0)) * 0.625),
            -35};
  case ReefLevel::kL3:
    return {frc::Units::InchesToMeters(
                47.625 - std::cos(frc::Units::DegreesToRadians(35.0)) * 0.625),
            -35};
  case ReefLevel::kL4:
    return {frc::Units::InchesToMeters(72), -90};
  default:
    return {frc::Units::InchesToMeters(72), -90};
  }
}

FieldConstants::AprilTagLayoutTypeData
FieldConstants::GetAprilTagLayoutTypeData(AprilTagLayoutType type) {
  AprilTagLayoutTypeData data;
  std::string name;
  switch (type) {
  case AprilTagLayoutType::kOfficial:
    name = "2025-official";
    break;
  case AprilTagLayoutType::kNoBarge:
    name = "2025-no-barge";
    break;
  case AprilTagLayoutType::kBlueReef:
    name = "2025-blue-reef";
    break;
  case AprilTagLayoutType::kRedReef:
    name = "2025-red-reef";
    break;
  case AprilTagLayoutType::kFieldBorder:
    name = "2025-field-border";
    break;
  default:
    name = "2025-official";
    break;
  }

  std::filesystem::path filePath;
  if (Constants::disableHAL) {
    filePath = std::filesystem::path("src") / "main" / "deploy" / "apriltags" /
               (fieldType == FieldConstants::FieldType::kAndymark ? "andymark"
                                                                  : "welded") /
               "2025-official.json";
  } else {
    filePath = frc::filesystem::GetDeployDirectory() / "apriltags" /
               (fieldType == FieldConstants::FieldType::kAndymark ? "andymark"
                                                                  : "welded") /
               (name + ".json");
  }

  try {
    data.layout = frc::AprilTagFieldLayout(filePath.string());
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to load AprilTag layout: " +
                             filePath.string());
  }

  try {
    std::ifstream file(filePath);
    std::stringstream buffer;
    buffer << file.rdbuf();
    data.layoutString = buffer.str();
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to serialize AprilTag layout JSON " +
                             name + "for Northstar");
  }

  return data;
}

namespace {
void InitializeReefData() {
  auto aprilTagLayout = FieldConstants::GetAprilTagLayoutTypeData(
                            FieldConstants::AprilTagLayoutType::kOfficial)
                            .layout;
  FieldConstants::Reef::centerFaces[0] =
      aprilTagLayout.GetTagPose(18).value().ToPose2d();
  FieldConstants::Reef::centerFaces[1] =
      aprilTagLayout.GetTagPose(19).value().ToPose2d();
  FieldConstants::Reef::centerFaces[2] =
      aprilTagLayout.GetTagPose(20).value().ToPose2d();
  FieldConstants::Reef::centerFaces[3] =
      aprilTagLayout.GetTagPose(21).value().ToPose2d();
  FieldConstants::Reef::centerFaces[4] =
      aprilTagLayout.GetTagPose(22).value().ToPose2d();
  FieldConstants::Reef::centerFaces[5] =
      aprilTagLayout.GetTagPose(17).value().ToPose2d();

  for (int face = 0; face < 6; face++) {
    std::map<FieldConstants::ReefLevel, frc::Pose3d> fillRight;
    std::map<FieldConstants::ReefLevel, frc::Pose3d> fillLeft;
    std::map<FieldConstants::ReefLevel, frc::Pose2d> fillRight2d;
    std::map<FieldConstants::ReefLevel, frc::Pose2d> fillLeft2d;
    for (int levelInt = 0; levelInt < 4; ++levelInt) {
      auto level = static_cast<FieldConstants::ReefLevel>(levelInt);
      FieldConstants::ReefLevelData levelData =
          FieldConstants::GetReefLevelData(level);
      frc::Pose2d poseDirection(FieldConstants::Reef::center,
                                frc::Rotation2d::Degrees(180 - (60 * face)));
      double adjustX = frc::Units::InchesToMeters(30.738);
      double adjustY = frc::Units::InchesToMeters(6.469);

      frc::Pose3d rightBranchPose(
          frc::Translation3d(poseDirection
                                 .TransformBy(frc::Transform2d(
                                     adjustX, adjustY, frc::Rotation2d::Zero()))
                                 .X(),
                             poseDirection
                                 .TransformBy(frc::Transform2d(
                                     adjustX, adjustY, frc::Rotation2d::Zero()))
                                 .Y(),
                             levelData.height),
          frc::Rotation3d(0, frc::Units::DegreesToRadians(levelData.pitch),
                          poseDirection.Rotation().Radians()));
      frc::Pose3d leftBranchPose(
          frc::Translation3d(
              poseDirection
                  .TransformBy(frc::Transform2d(adjustX, -adjustY,
                                                frc::Rotation2d::Zero()))
                  .X(),
              poseDirection
                  .TransformBy(frc::Transform2d(adjustX, -adjustY,
                                                frc::Rotation2d::Zero()))
                  .Y(),
              levelData.height),
          frc::Rotation3d(0, frc::Units::DegreesToRadians(levelData.pitch),
                          poseDirection.Rotation().Radians()));

      fillRight[level] = rightBranchPose;
      fillLeft[level] = leftBranchPose;
      fillRight2d[level] = rightBranchPose.ToPose2d();
      fillLeft2d[level] = leftBranchPose.ToPose2d();
    }
    FieldConstants::Reef::branchPositions.push_back(fillRight);
    FieldConstants::Reef::branchPositions.push_back(fillLeft);
    FieldConstants::Reef::branchPositions2d.push_back(fillRight2d);
    FieldConstants::Reef::branchPositions2d.push_back(fillLeft2d);
  }
}

struct InitializeFieldData {
  InitializeFieldData() { InitializeReefData(); }
};

InitializeFieldData init;
} // namespace
