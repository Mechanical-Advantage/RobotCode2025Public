// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import static org.littletonrobotics.frc2025.FieldConstants.fieldWidth;
import static org.littletonrobotics.frc2025.FieldConstants.startingLineX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AutoTracker.IntakingLocation;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.intake.Intake;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.*;

@RequiredArgsConstructor
public class AutoBuilder {
  private static final LoggedTunableNumber autoDangerTime =
      new LoggedTunableNumber("Auto/AutoDangerTime", 14.9);
  private static final LoggedTunableNumber intakeRestartTime =
      new LoggedTunableNumber("Auto/IntakeRestartTime", 1.2);

  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final BooleanSupplier upInThePush;

  private final AutoTracker autoTracker = new AutoTracker();

  public Command upInTheAirAuto() {
    Timer autoTimer = new Timer();
    Container<Integer> coralIndex = new Container<>(0);
    Container<ReefLevel> level = new Container<>(null);
    Container<Optional<CoralObjective>> coralObjective = new Container<>(Optional.empty());
    return Commands.runOnce(
            () -> {
              RobotState.getInstance().resetPose(autoTracker.getStartingPose());
              superstructure.setAutoStart();
              autoTimer.restart();
              coralIndex.value = 0;
              level.value = autoTracker.getLevel().orElse(null);
            })
        .andThen(
            getUpInThePush(),
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          coralIndex.value++;
                          level.value = autoTracker.getLevel().orElse(null);
                          if (level.value != null) {
                            coralObjective.value = autoTracker.getCoralObjective(level.value);
                          } else {
                            coralObjective.value = Optional.empty();
                          }
                        }),
                    // Score coral
                    AutoScoreCommands.autoScore(
                            drive, superstructure, () -> level.value, () -> coralObjective.value)
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                coralObjective.value.ifPresent(autoTracker::setObjectiveScored);
                                System.out.printf(
                                    "Coral #" + coralIndex.value + " scored at %.2f seconds \n",
                                    autoTimer.get());
                              }
                            })
                        .deadlineFor(
                            Commands.either(
                                intake.deploy(), intake.intake(), () -> coralIndex.value == 1)),
                    // Begin intake
                    intakeSequence())
                .repeatedly()
                .until(
                    () ->
                        level.value == null
                            || (autoTimer.hasElapsed(autoDangerTime.get())
                                && AutoScoreCommands.withinDistanceToReef(
                                    RobotState.getInstance().getEstimatedPose(),
                                    AutoScoreCommands.minDistanceReefClearL4.get())
                                && superstructure.readyForL4())),
            new DriveToPose(
                    drive,
                    () ->
                        RobotState.getInstance()
                            .getEstimatedPose()
                            .transformBy(GeomUtil.toTransform2d(-1.0, 0.0)))
                .deadlineFor(superstructure.runGoal(SuperstructureState.STOW)))
        .until(() -> autoTimer.hasElapsed(15.0));
  }

  public Command intakeSequence() {
    Timer intakeTimer = new Timer();
    Container<IntakingLocation> previousIntakingLocation = new Container<>(null);
    Container<IntakingLocation> currentIntakingLocation = new Container<>(null);
    Container<Boolean> hasBackedUp = new Container<>(false);
    return Commands.sequence(
            new DriveToPose(
                    drive,
                    () -> {
                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                      if (currentIntakingLocation.value == null) {
                        if (AutoScoreCommands.withinDistanceToReef(
                            robot, AutoScoreCommands.minDistanceReefClearL4.get())) {
                          return robot.transformBy(GeomUtil.toTransform2d(-0.5, 0.0));
                        } else {
                          return robot;
                        }
                      }

                      Pose2d intakePose =
                          AutoConstants.getIntakePose(robot, currentIntakingLocation.value);
                      if ((AutoScoreCommands.withinDistanceToReef(robot, 0.6)
                              && superstructure.readyForL4())
                          || AutoScoreCommands.withinDistanceToReef(robot, 0.9)) {
                        final double yError = intakePose.relativeTo(robot).getY();
                        return robot
                            .transformBy(
                                GeomUtil.toTransform2d(
                                    -1.8, Math.abs(yError) > 0.5 ? Math.signum(yError) * 0.5 : 0.0))
                            .transformBy(
                                GeomUtil.toTransform2d(
                                        robot
                                            .getRotation()
                                            .interpolate(intakePose.getRotation(), 0.05)
                                            .minus(robot.getRotation()))
                                    .times(0.35));
                      }
                      if (currentIntakingLocation.value == IntakingLocation.STATION) {
                        var robotToStationIntakePose = robot.relativeTo(intakePose);
                        if (Math.abs(robotToStationIntakePose.getX()) <= 0.15
                            && Math.abs(robotToStationIntakePose.getY()) <= 0.15
                            && Math.abs(robotToStationIntakePose.getRotation().getDegrees())
                                <= 10.0) {
                          hasBackedUp.value = true;
                        }
                        if (hasBackedUp.value)
                          return intakePose.transformBy(GeomUtil.toTransform2d(-1.4, 0.0));
                        return intakePose;
                      } else {
                        return intakePose.transformBy(
                            GeomUtil.toTransform2d(
                                MathUtil.clamp(
                                        (robot
                                                    .getTranslation()
                                                    .getDistance(intakePose.getTranslation())
                                                - 0.2)
                                            / 1.2,
                                        0.0,
                                        1.0)
                                    * 0.6,
                                0.0));
                      }
                    })
                .beforeStarting(
                    () -> {
                      currentIntakingLocation.value =
                          autoTracker
                              .getIntakeLocation(previousIntakingLocation.value)
                              .orElse(null);
                      hasBackedUp.value = false;
                    })
                .finallyDo(() -> previousIntakingLocation.value = currentIntakingLocation.value)
                .raceWith(new SuppliedWaitCommand(intakeRestartTime))
                .repeatedly()
                .until(() -> autoTracker.getNearestValidCoral().isPresent())
                .deadlineFor(intake.intake()),
            IntakeCommands.autoIntake(
                    drive,
                    intake,
                    autoTracker::getNearestValidCoral,
                    () -> intake.isCoralIndexed() || superstructure.hasCoral())
                .beforeStarting(intakeTimer::restart)
                .until(() -> autoTracker.getNearestValidCoral().isEmpty()))
        .repeatedly()
        .deadlineFor(
            Commands.waitUntil(
                    () -> {
                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                      Pose2d flippedRobot = AllianceFlipUtil.apply(robot);
                      return AutoScoreCommands.outOfDistanceToReef(
                              robot, AutoScoreCommands.minDistanceReefClearL4.get())
                          || Math.abs(
                                  FieldConstants.Reef.center
                                      .minus(flippedRobot.getTranslation())
                                      .getAngle()
                                      .minus(flippedRobot.getRotation())
                                      .getDegrees())
                              >= AutoScoreCommands.minAngleReefClear.get();
                    })
                .andThen(superstructure.runGoal(SuperstructureState.CORAL_INTAKE)))
        .until(() -> intake.isCoralIndexed() || superstructure.hasCoral())
        .beforeStarting(() -> previousIntakingLocation.value = null);
  }

  public Command upInTheSimplicityAuto() {
    final var objective = new CoralObjective(7, ReefLevel.L4);
    return Commands.runOnce(
            () -> {
              RobotState.getInstance()
                  .resetPose(
                      AllianceFlipUtil.apply(
                          MirrorUtil.apply(
                              new Pose2d(
                                  startingLineX - DriveConstants.robotWidth / 2.0,
                                  fieldWidth / 2.0,
                                  Rotation2d.kPi))));
              superstructure.setAutoStart();
            })
        .andThen(
            getUpInThePush(),
            AutoScoreCommands.autoScore(
                drive,
                superstructure,
                objective::reefLevel,
                () -> Optional.of(MirrorUtil.apply(objective))),
            new DriveToPose(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            AutoScoreCommands.getCoralScorePose(objective, false)
                                .plus(new Transform2d(-0.5, 0.0, Rotation2d.kZero))))
                .withTimeout(3.0)
                .deadlineFor(
                    superstructure.runGoal(
                        Superstructure.getScoringState(objective.reefLevel(), false, false))));
  }

  public Command upInTheInspirationalAuto() {
    return Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetPose(
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                RobotState.getInstance().getEstimatedPose().getTranslation(),
                                Rotation2d.kPi))))
        .andThen(
            new DriveToPose(
                    drive,
                    () -> RobotState.getInstance().getEstimatedPose(),
                    () -> RobotState.getInstance().getEstimatedPose(),
                    () ->
                        new Translation2d((AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0) * -1.0, 0.0),
                    () -> 0.0)
                .withTimeout(0.6));
  }

  private Command getUpInThePush() {
    double upInThePushSecs = 0.5;
    return new DriveToPose(
            drive,
            () -> {
              Pose2d robot = RobotState.getInstance().getEstimatedPose();
              return new Pose2d(
                  robot
                      .getTranslation()
                      .plus(new Translation2d(AllianceFlipUtil.shouldFlip() ? -0.1 : 0.1, 0.0)),
                  robot.getRotation());
            })
        .withTimeout(upInThePushSecs)
        .onlyIf(upInThePush);
  }
}
