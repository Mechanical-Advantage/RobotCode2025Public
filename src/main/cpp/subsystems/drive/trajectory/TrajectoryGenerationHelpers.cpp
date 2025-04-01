// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/TrajectoryGenerationHelpers.h"

frc::Pose2d TrajectoryGenerationHelpers::GetPose(const VehicleState &state) {
  return frc::Pose2d(state.x(), state.y(), frc::Rotation2d(state.theta()));
}

frc::Pose2d TrajectoryGenerationHelpers::GetPose(const Waypoint &waypoint) {
  return frc::Pose2d(waypoint.x(), waypoint.y(),
                     frc::Rotation2d(waypoint.heading_constraint()));
}

VehicleVelocityConstraint TrajectoryGenerationHelpers::CreateVelocityConstraint(
    const VehicleState &state) {
  VehicleVelocityConstraint constraint;
  constraint.set_vx(state.vx());
  constraint.set_vy(state.vy());
  constraint.set_omega(state.omega());
  return constraint;
}

VehicleVelocityConstraint TrajectoryGenerationHelpers::EndVelocityConstraint(
    const Trajectory &trajectory) {
  return CreateVelocityConstraint(
      trajectory.states(trajectory.states_size() - 1).state());
}

PathSegment::Builder TrajectoryGenerationHelpers::AddContinuationWaypoint(
    PathSegment::Builder builder, const Trajectory &trajectory) {
  frc::Pose2d endPose =
      GetPose(trajectory.states(trajectory.states_size() - 1).state());
  return builder.add_waypoints(
      WithPose(Waypoint::Builder(), endPose)
          .set_vehicle_velocity(EndVelocityConstraint(trajectory)));
}

PathSegment::Builder TrajectoryGenerationHelpers::AddContinuationWaypoint(
    PathSegment::Builder builder, const VehicleState &state) {
  return builder.add_waypoints(
      WithPose(Waypoint::Builder(), GetPose(state))
          .set_vehicle_velocity(CreateVelocityConstraint(state)));
}

PathSegment::Builder TrajectoryGenerationHelpers::AddTranslationWaypoint(
    PathSegment::Builder builder, const frc::Translation2d &translation) {
  return builder.add_waypoints(
      WithTranslation(Waypoint::Builder(), translation));
}

PathSegment::Builder
TrajectoryGenerationHelpers::AddPoseWaypoint(PathSegment::Builder builder,
                                             const frc::Pose2d &pose) {
  return builder.add_waypoints(WithPose(Waypoint::Builder(), pose));
}

PathSegment::Builder TrajectoryGenerationHelpers::AddTranslationWaypoint(
    PathSegment::Builder builder, const frc::Translation2d &translation,
    int samples) {
  return builder.add_waypoints(
      WithTranslation(Waypoint::Builder(), translation).set_samples(samples));
}

PathSegment::Builder TrajectoryGenerationHelpers::AddPoseWaypoint(
    PathSegment::Builder builder, const frc::Pose2d &pose, int samples) {
  return builder.add_waypoints(
      WithPose(Waypoint::Builder(), pose).set_samples(samples));
}

Waypoint::Builder TrajectoryGenerationHelpers::WithTranslation(
    Waypoint::Builder builder, const frc::Translation2d &translation) {
  return builder.set_x(translation.X()).set_y(translation.Y());
}

Waypoint::Builder
TrajectoryGenerationHelpers::WithPose(Waypoint::Builder builder,
                                      const frc::Pose2d &pose) {
  return WithTranslation(builder, pose.Translation())
      .set_heading_constraint(pose.Rotation().Radians());
}

Waypoint::Builder TrajectoryGenerationHelpers::WithLinearVelocity(
    Waypoint::Builder builder, const frc::Translation2d &linearVelocity) {
  VehicleVelocityConstraint constraint;
  constraint.set_vx(linearVelocity.X());
  constraint.set_vy(linearVelocity.Y());
  constraint.set_omega(0);
  return builder.set_vehicle_velocity(constraint);
}