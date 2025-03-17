// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive.trajectory;

import static org.littletonrobotics.frc2025.FieldConstants.*;
import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.*;
import java.util.function.Function;
import java.util.stream.IntStream;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.commands.AutoScoreCommands;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.util.GeomUtil;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();
  public static final List<Function<Set<String>, Map<String, List<PathSegment>>>> suppliedPaths =
      new ArrayList<>(); // List of functions that take a set of completed paths and return a map of

  // trajectories to generate (or null if they cannot be generated yet)

  // Drive straight path
  // (Used for preload of trajectory classes in drive constructor)
  static {
    paths.put(
        "driveStraight",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(Pose2d.kZero)
                .addPoseWaypoint(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180.0)))
                .build()));

    paths.put(
        "BLOB",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(Pose2d.kZero)
                .addTranslationWaypoint(new Translation2d(2, 3))
                .addPoseWaypoint(new Pose2d(0, 3, Rotation2d.fromDegrees(270.0)))
                .addPoseWaypoint(new Pose2d(2, 0.6, Rotation2d.fromDegrees(30.0)))
                .build()));
  }

  public static final double upInTheWaterLineupDistance = 0.3;
  public static final Pose2d[] upInTheWaterScoringPoses =
      IntStream.rangeClosed(1, 4)
          .mapToObj(
              i ->
                  AutoScoreCommands.getCoralScorePose(
                      new CoralObjective(i == 4 ? 0 : i + 8, ReefLevel.L4), false))
          .toArray(Pose2d[]::new);

  // Super up in the water auto
  static {
    final double coralScoringVelocity = 1.0;
    final double elevatorUpVelocity = DriveConstants.maxLinearSpeed * 0.5;
    final double elevatorDownVelocity = DriveConstants.maxLinearSpeed * 0.7;
    final double movingCoralDegreeOffset = -8.0;
    final double movingCoralLineup = 0.3;

    upInTheWaterScoringPoses[0] =
        upInTheWaterScoringPoses[0].transformBy(
            GeomUtil.toTransform2d(-Units.inchesToMeters(6.0), 0.0));
    final Pose2d preMovingScore =
        new Pose2d(
            upInTheWaterScoringPoses[0]
                .transformBy(GeomUtil.toTransform2d(0.0, -movingCoralLineup / 2.0))
                .getTranslation(),
            upInTheWaterScoringPoses[0]
                .getRotation()
                .plus(Rotation2d.fromDegrees(movingCoralDegreeOffset)));
    final Pose2d postMovingScore =
        preMovingScore.transformBy(GeomUtil.toTransform2d(0.0, movingCoralLineup));
    paths.put(
        "SuperUpInTheWater1Score",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                        startingLineX - DriveConstants.robotWidth / 2.0,
                        fieldWidth - Barge.closeCage.getY() + DriveConstants.robotWidth / 2.0,
                        Rotation2d.kCCW_Pi_2))
                .addPoseWaypoint(preMovingScore)
                .setMaxVelocity(elevatorUpVelocity * 0.7)
                .build(),
            PathSegment.newBuilder()
                .addWaypoints(
                    Waypoint.newBuilder()
                        .withPose(postMovingScore)
                        .withLinearVelocity(
                            new Translation2d(
                                coralScoringVelocity,
                                postMovingScore
                                    .getTranslation()
                                    .minus(preMovingScore.getTranslation())
                                    .getAngle()))
                        .build())
                .setMaxVelocity(coralScoringVelocity)
                .setStraightLine(true)
                .setMaxOmega(0.0)
                .build()));
    paths.put(
        "SuperUpInTheWater1Intake",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("SuperUpInTheWater1Score"))
                .addPoseWaypoint(
                    getNearestIntakingPose(getLastWaypoint("SuperUpInTheWater1Score").getPose()))
                .setMaxVelocity(elevatorDownVelocity)
                .build()));
    for (int i = 0; i < 4; i++) {
      Pose2d scoringPose = upInTheWaterScoringPoses[i];
      var firstSegment = PathSegment.newBuilder();
      if (i == 0) {
        firstSegment =
            firstSegment.addPoseWaypoint(
                new Pose2d(
                    startingLineX - DriveConstants.robotWidth / 2.0,
                    fieldWidth - Barge.closeCage.getY() + DriveConstants.robotWidth / 2.0,
                    Rotation2d.kCCW_Pi_2));
      } else {
        firstSegment = firstSegment.addWaypoints(getLastWaypoint("UpInTheWater" + i + "Intake"));
      }
      paths.put(
          "UpInTheWater" + (i + 1) + "Score",
          List.of(
              firstSegment
                  .addPoseWaypoint(
                      scoringPose.transformBy(
                          GeomUtil.toTransform2d(
                              -upInTheWaterLineupDistance, i == 3 || i == 0 ? -0.2 : 0.0)))
                  .setMaxVelocity(elevatorUpVelocity)
                  .build(),
              PathSegment.newBuilder()
                  .addPoseWaypoint(scoringPose)
                  .setMaxVelocity(coralScoringVelocity)
                  .build()));
      if (i == 3) continue;
      paths.put(
          "UpInTheWater" + (i + 1) + "Intake",
          List.of(
              PathSegment.newBuilder()
                  .addWaypoints(getLastWaypoint("UpInTheWater" + (i + 1) + "Score"))
                  .addPoseWaypoint(getNearestIntakingPose(scoringPose))
                  .setMaxVelocity(elevatorDownVelocity)
                  .build()));
    }
  }

  private static Pose2d getNearestIntakingPose(Pose2d pose) {
    return CoralStation.rightCenterFace.transformBy(
        GeomUtil.toTransform2d(
            DriveConstants.robotWidth / 2.0 + Units.inchesToMeters(3.0),
            MathUtil.clamp(
                pose.relativeTo(CoralStation.rightCenterFace).getY(),
                -FieldConstants.CoralStation.stationLength / 2 + Units.inchesToMeters(16),
                FieldConstants.CoralStation.stationLength / 2 - Units.inchesToMeters(16))));
  }

  /** Returns the last waypoint of a trajectory. */
  public static Waypoint getLastWaypoint(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    return trajectory
        .get(trajectory.size() - 1)
        .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
  }
}
