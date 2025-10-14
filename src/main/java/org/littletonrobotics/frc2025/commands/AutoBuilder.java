// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import static org.littletonrobotics.frc2025.FieldConstants.fieldWidth;
import static org.littletonrobotics.frc2025.FieldConstants.startingLineX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.util.*;

@RequiredArgsConstructor
public class AutoBuilder {
  private static final LoggedTunableNumber scoreCancelSecs =
      new LoggedTunableNumber("AutoBuilder/ScoreCancelSeconds", 1.5);
  private static final LoggedTunableNumber intakeTimeSecs =
      new LoggedTunableNumber("AutoBuilder/IntakeTimeSecs", 0.15);

  private final Drive drive;
  private final Superstructure superstructure;
  private final RollerSystem funnel;
  private final BooleanSupplier upInThePush;

  private final double upInThePushSecs = 0.5;

  public Command upInTheWaterAuto(boolean isSuper) {

    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(8, isSuper ? ReefLevel.L4 : ReefLevel.L2),
          new CoralObjective(10, ReefLevel.L4),
          new CoralObjective(11, ReefLevel.L4),
          new CoralObjective(0, ReefLevel.L2)
        };
    Container<Integer> currentObjectiveIndex = new Container<>();

    var driveToStation = new DriveToStation(drive, true);
    Timer autoTimer = new Timer();
    Timer intakeTimer = new Timer();
    Timer coralIndexedTimer = new Timer();
    return Commands.runOnce(
            () -> {
              RobotState.getInstance()
                  .resetPose(
                      AllianceFlipUtil.apply(
                          MirrorUtil.apply(
                              new Pose2d(
                                  startingLineX - DriveConstants.robotWidth / 2.0,
                                  fieldWidth - FieldConstants.Barge.closeCage.getY(),
                                  Rotation2d.kCCW_Pi_2))));
              superstructure.setAutoStart();
              autoTimer.restart();
              currentObjectiveIndex.value = 0;
            })
        .andThen(
            getUpInThePush(),
            Commands.sequence(
                    // Intake
                    driveToStation
                        .deadlineFor(
                            AutoScoreCommands.superstructureAimAndEject(
                                    superstructure,
                                    () -> coralObjectives[currentObjectiveIndex.value].reefLevel(),
                                    () ->
                                        Optional.of(
                                            MirrorUtil.apply(
                                                coralObjectives[currentObjectiveIndex.value])),
                                    () -> true,
                                    () -> true,
                                    () -> false)
                                .until(
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
                                .andThen(IntakeCommands.intake(superstructure, funnel)))
                        .until(
                            () -> {
                              if (!driveToStation.withinTolerance(
                                  Units.inchesToMeters(5.0), Rotation2d.fromDegrees(5.0))) {
                                intakeTimer.restart();
                              }
                              return superstructure.hasCoral()
                                  || intakeTimer.hasElapsed(intakeTimeSecs.get());
                            }),
                    // Score
                    AutoScoreCommands.autoScore(
                            drive,
                            superstructure,
                            funnel,
                            () -> coralObjectives[currentObjectiveIndex.value].reefLevel(),
                            () ->
                                Optional.of(
                                    MirrorUtil.apply(coralObjectives[currentObjectiveIndex.value])))
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                System.out.printf(
                                    "Scored Coral #"
                                        + (currentObjectiveIndex.value + 1)
                                        + " at %.2f\n",
                                    autoTimer.get());
                                currentObjectiveIndex.value++;
                              }
                            })
                        .beforeStarting(coralIndexedTimer::restart)
                        .raceWith(
                            Commands.waitUntil(
                                    () -> coralIndexedTimer.hasElapsed(scoreCancelSecs.get()))
                                .andThen(Commands.idle().onlyIf(superstructure::hasCoral))))
                .repeatedly()
                .until(() -> currentObjectiveIndex.value >= coralObjectives.length))
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        RobotState.getInstance()
                            .getEstimatedPose()
                            .transformBy(
                                GeomUtil.toTransform2d(
                                    -AutoScoreCommands.minDistanceReefClearL4.get(), 0.0)))
                .until(
                    () ->
                        AutoScoreCommands.outOfDistanceToReef(
                            RobotState.getInstance().getEstimatedPose(),
                            AutoScoreCommands.minDistanceReefClearL4.get())));
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
                funnel,
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
