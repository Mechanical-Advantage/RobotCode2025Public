// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryGenerationHelpers {
  /**
   * Gets the pose of the state
   *
   * @param state The {@link VehicleState}
   * @return {@link Pose2d} of the state
   */
  public static Pose2d getPose(VehicleState state) {
    return new Pose2d(state.getX(), state.getY(), Rotation2d.fromRadians(state.getTheta()));
  }

  public static Pose2d getPose(Waypoint waypoint) {
    // Eeek could be empty for theta
    return new Pose2d(
        waypoint.getX(), waypoint.getY(), Rotation2d.fromRadians(waypoint.getHeadingConstraint()));
  }

  /**
   * Creates a {@link VehicleVelocityConstraint} from a {@link VehicleState}, useful for chaining a
   * generated trajectory into a new trajectory.
   *
   * @param state The {@link VehicleState}
   * @return The {@link VehicleVelocityConstraint} representing the velocity of the state.
   */
  public static VehicleVelocityConstraint createVelocityConstraint(VehicleState state) {
    return VehicleVelocityConstraint.newBuilder()
        .setVx(state.getVx())
        .setVy(state.getVy())
        .setOmega(state.getOmega())
        .build();
  }

  public static VehicleVelocityConstraint endVelocityConstraint(Trajectory trajectory) {
    return createVelocityConstraint(
        trajectory.getStates(trajectory.getStatesCount() - 1).getState());
  }

  /**
   * Adds a waypoint to an existing {@link PathSegment.Builder} which is a continuation of a
   * previously generated trajectory. The pose and velocity of the waypoint are set to the ending
   * pose and velocity of the trajectory.
   *
   * @param builder The {@link PathSegment.Builder}.
   * @param trajectory The generated trajectory.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addContinuationWaypoint(
      PathSegment.Builder builder, Trajectory trajectory) {
    Pose2d endPose = getPose(trajectory.getStates(trajectory.getStatesCount() - 1).getState());
    return builder.addWaypoints(
        withPose(Waypoint.newBuilder(), endPose)
            .setVehicleVelocity(endVelocityConstraint(trajectory)));
  }

  /**
   * Adds a waypoint to an existing {@link PathSegment.Builder} which is a continuation of a
   * previously generated vehicle state. The pose and velocity of the waypoint are set to match the
   * given vehicle state
   *
   * @param builder The {@link PathSegment.Builder}.
   * @param state The generated vehicle state.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addContinuationWaypoint(
      PathSegment.Builder builder, VehicleState state) {
    return builder.addWaypoints(
        withPose(Waypoint.newBuilder(), getPose(state))
            .setVehicleVelocity(createVelocityConstraint(state)));
  }

  /**
   * Adds a translation waypoint to an existing {@link PathSegment.Builder} without setting any
   * other constraints.
   *
   * @param builder The {@link PathSegment.Builder}.
   * @param translation The translation.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addTranslationWaypoint(
      PathSegment.Builder builder, Translation2d translation) {
    return builder.addWaypoints(withTranslation(Waypoint.newBuilder(), translation));
  }

  /**
   * Adds a waypoint to an existing {@link PathSegment.Builder} without setting any other
   * costraints.
   *
   * @param builder The {@link PathSegment.Builder}
   * @param pose The pose.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addPoseWaypoint(PathSegment.Builder builder, Pose2d pose) {
    return builder.addWaypoints(withPose(Waypoint.newBuilder(), pose));
  }

  /**
   * Adds a translation waypoint to an existing {@link PathSegment.Builder} without setting any
   * other constraints.
   *
   * @param builder The {@link PathSegment.Builder}.
   * @param translation The translation.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addTranslationWaypoint(
      PathSegment.Builder builder, Translation2d translation, int samples) {
    return builder.addWaypoints(
        withTranslation(Waypoint.newBuilder(), translation).setSamples(samples));
  }

  /**
   * Adds a waypoint to an existing {@link PathSegment.Builder} without setting any other
   * costraints.
   *
   * @param builder The {@link PathSegment.Builder}
   * @param pose The pose.
   * @return The {@link PathSegment.Builder} with the new waypoint added.
   */
  public static PathSegment.Builder addPoseWaypoint(
      PathSegment.Builder builder, Pose2d pose, int samples) {
    return builder.addWaypoints(withPose(Waypoint.newBuilder(), pose).setSamples(samples));
  }

  /**
   * Adds {@link Translation2d} to an existing {@link Waypoint.Builder}
   *
   * @param builder The {@link Waypoint.Builder}
   * @param translation The translation.
   * @return The {@link Waypoint.Builder} with the translation added.
   */
  public static Waypoint.Builder withTranslation(
      Waypoint.Builder builder, Translation2d translation) {
    return builder.setX(translation.getX()).setY(translation.getY());
  }

  /**
   * Adds {@link Pose2d} to an existing {@link Waypoint.Builder}
   *
   * @param builder The {@link Waypoint.Builder}
   * @param pose The pose.
   * @return The {@link Waypoint.Builder} with the pose added.
   */
  public static Waypoint.Builder withPose(Waypoint.Builder builder, Pose2d pose) {
    return withTranslation(builder, pose.getTranslation())
        .setHeadingConstraint(pose.getRotation().getRadians());
  }

  public static Waypoint.Builder withLinearVelocity(
      Waypoint.Builder builder, Translation2d linearVelocity) {
    return builder.setVehicleVelocity(
        VehicleVelocityConstraint.newBuilder()
            .setVx(linearVelocity.getX())
            .setVy(linearVelocity.getY())
            .setOmega(0)
            .build());
  }
}
