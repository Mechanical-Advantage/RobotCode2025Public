// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

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

class FieldConstants {
public:
  enum class FieldType { kAndymark, kWelded };

  static constexpr FieldType fieldType = FieldType::kWelded;

  static constexpr double fieldLength =
      AprilTagLayoutType::kOfficial.GetLayout().GetFieldLength();
  static constexpr double fieldWidth =
      AprilTagLayoutType::kOfficial.GetLayout().GetFieldWidth();
  static constexpr double startingLineX = frc::Units::InchesToMeters(299.438);
  static constexpr double algaeDiameter = frc::Units::InchesToMeters(16);

  class Processor {
  public:
    static constexpr frc::Pose2d centerFace = frc::Pose2d(
        AprilTagLayoutType::kOfficial.GetLayout().GetTagPose(16).value().X(), 0,
        frc::Rotation2d::Degrees(90));
    static constexpr frc::Pose2d opposingCenterFace = frc::Pose2d(
        AprilTagLayoutType::kOfficial.GetLayout().GetTagPose(3).value().X(),
        fieldWidth, frc::Rotation2d::Degrees(-90));
  };

  class Barge {
  public:
    static constexpr double netWidth = frc::Units::InchesToMeters(40.0);
    static constexpr double netHeight = frc::Units::InchesToMeters(88.0);

    static constexpr frc::Translation2d farCage =
        frc::Translation2d(frc::Units::InchesToMeters(345.428),
                           frc::Units::InchesToMeters(286.779));
    static constexpr frc::Translation2d middleCage =
        frc::Translation2d(frc::Units::InchesToMeters(345.428),
                           frc::Units::InchesToMeters(242.855));
    static constexpr frc::Translation2d closeCage =
        frc::Translation2d(frc::Units::InchesToMeters(345.428),
                           frc::Units::InchesToMeters(199.947));

    static constexpr double deepHeight = frc::Units::InchesToMeters(3.125);
    static constexpr double shallowHeight = frc::Units::InchesToMeters(30.125);
  };

  class CoralStation {
  public:
    static constexpr double stationLength = frc::Units::InchesToMeters(79.750);
    static constexpr frc::Pose2d rightCenterFace = frc::Pose2d(
        frc::Units::InchesToMeters(33.526), frc::Units::InchesToMeters(25.824),
        frc::Rotation2d::Degrees(144.011 - 90));
    static constexpr frc::Pose2d leftCenterFace =
        frc::Pose2d(rightCenterFace.X(), fieldWidth - rightCenterFace.Y(),
                    frc::Rotation2d(-rightCenterFace.Rotation().Radians()));
  };

  class Reef {
  public:
    static constexpr double faceLength = frc::Units::InchesToMeters(36.792600);
    static constexpr frc::Translation2d center = frc::Translation2d(
        frc::Units::InchesToMeters(176.746), fieldWidth / 2.0);
    static constexpr double faceToZoneLine = frc::Units::InchesToMeters(12);

    static std::array<frc::Pose2d, 6> centerFaces;
    static std::vector<std::map<ReefLevel, frc::Pose3d>> branchPositions;
    static std::vector<std::map<ReefLevel, frc::Pose2d>> branchPositions2d;
  };

  class StagingPositions {
  public:
    static constexpr double separation = frc::Units::InchesToMeters(72.0);
    static constexpr frc::Pose2d middleIceCream =
        frc::Pose2d(frc::Units::InchesToMeters(48), fieldWidth / 2.0,
                    frc::Rotation2d::Zero());
    static constexpr frc::Pose2d leftIceCream =
        frc::Pose2d(frc::Units::InchesToMeters(48),
                    middleIceCream.Y() + separation, frc::Rotation2d::Zero());
    static constexpr frc::Pose2d rightIceCream =
        frc::Pose2d(frc::Units::InchesToMeters(48),
                    middleIceCream.Y() - separation, frc::Rotation2d::Zero());
  };

  enum class ReefLevel { kL1, kL2, kL3, kL4 };

  struct ReefLevelData {
    double height;
    double pitch;
  };

  static ReefLevelData GetReefLevelData(ReefLevel level);

  static constexpr double aprilTagWidth = frc::Units::InchesToMeters(6.50);
  static constexpr int aprilTagCount = 22;
  static constexpr AprilTagLayoutType defaultAprilTagType =
      AprilTagLayoutType::kNoBarge;

  enum class AprilTagLayoutType {
    kOfficial,
    kNoBarge,
    kBlueReef,
    kRedReef,
    kFieldBorder
  };

  class AprilTagLayoutTypeData {
  public:
    frc::AprilTagFieldLayout layout;
    std::string layoutString;
  };

  static AprilTagLayoutTypeData
  GetAprilTagLayoutTypeData(AprilTagLayoutType type);

  struct CoralObjective {
    int branchId;
    ReefLevel reefLevel;
  };

  struct AlgaeObjective {
    int id;
    bool low;
    AlgaeObjective(int id) : id(id), low(id % 2 == 1) {}
  };

private:
  FieldConstants() = delete;
};