// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/DriveTrajectories.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/commands/AutoScoreCommands.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/TrajectoryGenerationHelpers.h"
#include "org/littletonrobotics/frc2025/util/GeomUtil.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"
#include "units/length.h"

std::map<std::string, std::vector<PathSegment>> DriveTrajectories::paths;
std::vector<std::function<std::map<std::string, std::vector<PathSegment>>(
    const std::set<std::string> &)>>
    DriveTrajectories::suppliedPaths;
std::array<frc::Pose2d, 4> DriveTrajectories::upInTheWaterScoringPoses;

void DriveTrajectories::InitializePaths() {
  // Drive straight path
  // (Used for preload of trajectory classes in drive constructor)
  paths["driveStraight"] = {
      PathSegment()
          .NewBuilder()
          .AddPoseWaypoint(frc::Pose2d())
          .AddPoseWaypoint(
              frc::Pose2d(3.0_m, 2.0_m, frc::Rotation2d::Degrees(180.0)))
          .Build()};

  paths["BLOB"] = {PathSegment()
                       .NewBuilder()
                       .AddPoseWaypoint(frc::Pose2d())
                       .AddTranslationWaypoint(frc::Translation2d(2.0_m, 3.0_m))
                       .AddPoseWaypoint(frc::Pose2d(
                           0.0_m, 3.0_m, frc::Rotation2d::Degrees(270.0)))
                       .AddPoseWaypoint(frc::Pose2d(
                           2.0_m, 0.6_m, frc::Rotation2d::Degrees(30.0)))
                       .Build()};

  // Super up in the water auto
  double coralScoringVelocity = 1.0;
  double elevatorUpVelocity = org::littletonrobotics::frc2025::subsystems::
                                  drive::DriveConstants::maxLinearSpeed *
                              0.5;
  double elevatorDownVelocity = org::littletonrobotics::frc2025::subsystems::
                                    drive::DriveConstants::maxLinearSpeed *
                                0.7;
  double movingCoralDegreeOffset = -8.0;
  double movingCoralLineup = 0.3;

  for (int i = 0; i < 4; ++i) {
    upInTheWaterScoringPoses[i] = org::littletonrobotics::frc2025::commands::
        AutoScoreCommands::GetCoralScorePose(
            org::littletonrobotics::frc2025::commands::CoralObjective(
                i == 3 ? 0 : i + 9,
                org::littletonrobotics::frc2025::commands::ReefLevel::L4));
  }

  upInTheWaterScoringPoses[0] = upInTheWaterScoringPoses[0].TransformBy(
      org::littletonrobotics::frc2025::util::GeomUtil::ToTransform2d(
          -units::inch_t(6.0), 0.0_m));

  frc::Pose2d preMovingScore = frc::Pose2d(
      upInTheWaterScoringPoses[0]
          .TransformBy(
              org::littletonrobotics::frc2025::util::GeomUtil::ToTransform2d(
                  0.0_m, -movingCoralLineup / 2.0))
          .Translation(),
      upInTheWaterScoringPoses[0].Rotation() +
          frc::Rotation2d::Degrees(movingCoralDegreeOffset));

  frc::Pose2d postMovingScore = preMovingScore.TransformBy(
      org::littletonrobotics::frc2025::util::GeomUtil::ToTransform2d(
          0.0_m, movingCoralLineup));

  paths["SuperUpInTheWater1Score"] = {
      PathSegment()
          .NewBuilder()
          .AddPoseWaypoint(frc::Pose2d(
              org::littletonrobotics::frc2025::FieldConstants::startingLineX -
                  org::littletonrobotics::frc2025::subsystems::drive::
                          DriveConstants::robotWidth /
                      2.0,
              org::littletonrobotics::frc2025::FieldConstants::fieldWidth -
                  org::littletonrobotics::frc2025::FieldConstants::Barge::
                      closeCage.Y() +
                  org::littletonrobotics::frc2025::subsystems::drive::
                          DriveConstants::robotWidth /
                      2.0,
              frc::Rotation2d::Degrees(90.0)))
          .AddPoseWaypoint(preMovingScore)
          .SetMaxVelocity(elevatorUpVelocity * 0.7)
          .Build(),
      PathSegment()
          .NewBuilder()
          .AddWaypoints(
              Waypoint()
                  .NewBuilder()
                  .WithPose(postMovingScore)
                  .WithLinearVelocity(frc::Translation2d(
                      coralScoringVelocity, (postMovingScore.Translation() -
                                             preMovingScore.Translation())
                                                .Angle()))
                  .Build())
          .SetMaxVelocity(coralScoringVelocity)
          .SetStraightLine(true)
          .SetMaxOmega(0.0)
          .Build()};

  paths["SuperUpInTheWater1Intake"] = {
      PathSegment()
          .NewBuilder()
          .AddWaypoints(GetLastWaypoint("SuperUpInTheWater1Score"))
          .AddPoseWaypoint(GetNearestIntakingPose(
              GetLastWaypoint("SuperUpInTheWater1Score").Pose()))
          .SetMaxVelocity(elevatorDownVelocity)
          .Build()};

  for (int i = 0; i < 4; ++i) {
    frc::Pose2d scoringPose = upInTheWaterScoringPoses[i];
    PathSegment::Builder firstSegment = PathSegment().NewBuilder();
    if (i == 0) {
      firstSegment.AddPoseWaypoint(frc::Pose2d(
          org::littletonrobotics::frc2025::FieldConstants::startingLineX -
              org::littletonrobotics::frc2025::subsystems::drive::
                      DriveConstants::robotWidth /
                  2.0,
          org::littletonrobotics::frc2025::FieldConstants::fieldWidth -
              org::littletonrobotics::frc2025::FieldConstants::Barge::closeCage
                  .Y() +
              org::littletonrobotics::frc2025::subsystems::drive::
                      DriveConstants::robotWidth /
                  2.0,
          frc::Rotation2d::Degrees(90.0)));
    } else {
      firstSegment.AddWaypoints(
          GetLastWaypoint("UpInTheWater" + std::to_string(i) + "Intake"));
    }
    paths["UpInTheWater" + std::to_string(i + 1) + "Score"] = {
        firstSegment
            .AddPoseWaypoint(scoringPose.TransformBy(
                org::littletonrobotics::frc2025::util::GeomUtil::ToTransform2d(
                    -DriveTrajectories::upInTheWaterLineupDistance,
                    i == 3 || i == 0 ? -0.2_m : 0.0_m)))
            .SetMaxVelocity(elevatorUpVelocity)
            .Build(),
        PathSegment()
            .NewBuilder()
            .AddPoseWaypoint(scoringPose)
            .SetMaxVelocity(coralScoringVelocity)
            .Build()};
    if (i == 3)
      continue;
    paths["UpInTheWater" + std::to_string(i + 1) + "Intake"] = {
        PathSegment()
            .NewBuilder()
            .AddWaypoints(GetLastWaypoint("UpInTheWater" +
                                          std::to_string(i + 1) + "Score"))
            .AddPoseWaypoint(GetNearestIntakingPose(scoringPose))
            .SetMaxVelocity(elevatorDownVelocity)
            .Build()};
  }
}

frc::Pose2d DriveTrajectories::GetNearestIntakingPose(const frc::Pose2d &pose) {
  return org::littletonrobotics::frc2025::FieldConstants::CoralStation::
      rightCenterFace.TransformBy(
          org::littletonrobotics::frc2025::util::GeomUtil::ToTransform2d(
              org::littletonrobotics::frc2025::subsystems::drive::
                          DriveConstants::robotWidth /
                      2.0 +
                  units::inch_t(3.0),
              frc::math::Clamp(
                  pose.RelativeTo(
                          org::littletonrobotics::frc2025::FieldConstants::
                              CoralStation::rightCenterFace)
                      .Y(),
                  -org::littletonrobotics::frc2025::FieldConstants::
                              CoralStation::stationLength /
                          2 +
                      units::inch_t(16.0),
                  org::littletonrobotics::frc2025::FieldConstants::
                              CoralStation::stationLength /
                          2 -
                      units::inch_t(16.0))));
}

frc::Waypoint
DriveTrajectories::GetLastWaypoint(const std::string &trajectoryName) {
  const std::vector<PathSegment> &trajectory = paths.at(trajectoryName);
  return trajectory.back().GetWaypoints(trajectory.back().GetWaypointsCount() -
                                        1);
}

std::map<std::string, std::vector<PathSegment>> DriveTrajectories::paths;
std::vector<std::function<std::map<std::string, std::vector<PathSegment>>(
    const std::set<std::string> &)>>
    DriveTrajectories::suppliedPaths;
std::array<frc::Pose2d, 4> DriveTrajectories::upInTheWaterScoringPoses;

// Initialize static members
static bool initialized = false;

struct DriveTrajectoriesInitializer {
  DriveTrajectoriesInitializer() {
    if (!initialized) {
      DriveTrajectories::InitializePaths();
      initialized = true;
    }
  }
};

static DriveTrajectoriesInitializer initializer;
