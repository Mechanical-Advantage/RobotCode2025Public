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
    final double elevatorUpVelocity = DriveConstants.maxLinearSpeed * 0.65;
    final double movingCoralDegreeOffset = -10.0;
    final double coralLineup = Units.inchesToMeters(12.0);
    final double movingCoralLineup = 0.3;
    final double transitionVelocity = 1.2;

    final Pose2d movingScorePose =
        upInTheWaterScoringPoses[0].transformBy(
            GeomUtil.toTransform2d(-Units.inchesToMeters(5.0), 0.0));
    final Pose2d preMovingScore =
        new Pose2d(
            movingScorePose
                .transformBy(GeomUtil.toTransform2d(0.0, -movingCoralLineup))
                .getTranslation(),
            movingScorePose.getRotation().plus(Rotation2d.fromDegrees(movingCoralDegreeOffset)));
    final Pose2d postMovingScore =
        new Pose2d(
            movingScorePose
                .transformBy(GeomUtil.toTransform2d(0.0, movingCoralLineup))
                .getTranslation(),
            movingScorePose.getRotation().plus(Rotation2d.fromDegrees(movingCoralDegreeOffset)));
    paths.put(
        "SuperUpInTheWater1Score",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                        startingLineX - DriveConstants.robotWidth / 2.0,
                        fieldWidth - Barge.closeCage.getY(),
                        Rotation2d.kCCW_Pi_2))
                .addPoseWaypoint(preMovingScore)
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
                .setStraightLine(true)
                .setMaxVelocity(coralScoringVelocity)
                .build()));
    paths.put(
        "SuperUpInTheWater1Intake",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("SuperUpInTheWater1Score"))
                .addPoseWaypoint(
                    getNearestIntakingPose(getLastWaypoint("SuperUpInTheWater1Score").getPose()))
                .build()));
    for (int i = 0; i < 4; i++) {
      Pose2d scoringPose = upInTheWaterScoringPoses[i];
      var firstSegment = PathSegment.newBuilder();
      if (i == 0) {
        firstSegment =
            firstSegment.addPoseWaypoint(
                new Pose2d(
                    startingLineX - DriveConstants.robotWidth / 2.0,
                    fieldWidth - Barge.closeCage.getY(),
                    Rotation2d.kCCW_Pi_2));
      } else {
        firstSegment = firstSegment.addWaypoints(getLastWaypoint("UpInTheWater" + i + "Intake"));
      }
      var lineupPose =
          scoringPose.transformBy(
              GeomUtil.toTransform2d(-coralLineup, i == 3 || i == 0 ? -0.2 : 0.0));
      paths.put(
          "UpInTheWater" + (i + 1) + "Score",
          List.of(
              firstSegment
                  .addWaypoints(
                      Waypoint.newBuilder()
                          .withPose(lineupPose)
                          .withLinearVelocity(
                              new Translation2d(
                                  transitionVelocity,
                                  scoringPose
                                      .getTranslation()
                                      .minus(lineupPose.getTranslation())
                                      .getAngle())))
                  .setMaxVelocity(elevatorUpVelocity)
                  .build()));
      if (i == 3) continue;
      paths.put(
          "UpInTheWater" + (i + 1) + "Intake",
          List.of(
              PathSegment.newBuilder()
                  .addPoseWaypoint(scoringPose)
                  .addPoseWaypoint(getNearestIntakingPose(scoringPose))
                  .build()));
    }
  }

  private static Pose2d getNearestIntakingPose(Pose2d pose) {
    return CoralStation.rightCenterFace.transformBy(
        GeomUtil.toTransform2d(
            DriveConstants.robotWidth / 2.0 + Units.inchesToMeters(7.0),
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
