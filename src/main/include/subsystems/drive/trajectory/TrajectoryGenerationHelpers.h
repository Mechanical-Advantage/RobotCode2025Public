// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"

class TrajectoryGenerationHelpers {
public:
  static frc::Pose2d GetPose(const VehicleState &state);
  static frc::Pose2d GetPose(const Waypoint &waypoint);
  static VehicleVelocityConstraint
  CreateVelocityConstraint(const VehicleState &state);
  static VehicleVelocityConstraint
  EndVelocityConstraint(const Trajectory &trajectory);
  static PathSegment::Builder
  AddContinuationWaypoint(PathSegment::Builder builder,
                          const Trajectory &trajectory);
  static PathSegment::Builder
  AddContinuationWaypoint(PathSegment::Builder builder,
                          const VehicleState &state);
  static PathSegment::Builder
  AddTranslationWaypoint(PathSegment::Builder builder,
                         const frc::Translation2d &translation);
  static PathSegment::Builder AddPoseWaypoint(PathSegment::Builder builder,
                                              const frc::Pose2d &pose);
  static PathSegment::Builder
  AddTranslationWaypoint(PathSegment::Builder builder,
                         const frc::Translation2d &translation, int samples);
  static PathSegment::Builder AddPoseWaypoint(PathSegment::Builder builder,
                                              const frc::Pose2d &pose,
                                              int samples);
  static Waypoint::Builder
  WithTranslation(Waypoint::Builder builder,
                  const frc::Translation2d &translation);
  static Waypoint::Builder WithPose(Waypoint::Builder builder,
                                    const frc::Pose2d &pose);
  static Waypoint::Builder
  WithLinearVelocity(Waypoint::Builder builder,
                     const frc::Translation2d &linearVelocity);
};