// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands.auto;

import static org.littletonrobotics.frc2025.FieldConstants.fieldWidth;
import static org.littletonrobotics.frc2025.FieldConstants.startingLineX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;
import java.util.stream.IntStream;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.*;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.MirrorUtil;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final RollerSystem funnel;
  private final ObjectiveTracker objectiveTracker;

  private final double intakeTimeSeconds = 0.35;
  private final double coralEjectTimeSeconds = 0.3;

  public Command superUpInTheWaterAuto() {
    HolonomicTrajectory upInTheWater1Score = new HolonomicTrajectory("SuperUpInTheWater1Score");
    HolonomicTrajectory upInTheWater1Intake = new HolonomicTrajectory("SuperUpInTheWater1Intake");

    ReefLevel level =
        Constants.getRobot() == Constants.RobotType.DEVBOT ? ReefLevel.L3 : ReefLevel.L4;
    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(9, level),
          new CoralObjective(10, level),
          new CoralObjective(11, level),
          new CoralObjective(0, level),
        };

    final Timer autoTimer = new Timer();
    return Commands.runOnce(
            () -> {
              AutoCommands.resetPose(upInTheWater1Score, true);
              autoTimer.restart();
              superstructure.setAutoStart();
            })
        .andThen(
            // First score and intake
            Commands.sequence(
                    AutoCommands.coralScoringTrajectory(
                        drive, upInTheWater1Score, coralObjectives[0], true),
                    new DriveTrajectory(drive, upInTheWater1Intake, true))
                .deadlineFor(
                    Commands.sequence(
                        AutoCommands.superstructureAimAndEjectCommand(
                            superstructure,
                            coralObjectives[0],
                            true,
                            () ->
                                autoTimer.hasElapsed(
                                    upInTheWater1Score.getDuration() - coralEjectTimeSeconds)),
                        Commands.runOnce(
                            () -> System.out.printf("Scored Coral #1 at %.2f\n", autoTimer.get())),
                        objectiveTracker.requestScored(() -> MirrorUtil.apply(coralObjectives[0])),
                        IntakeCommands.intake(superstructure, funnel))),

            // Rest of them
            Commands.sequence(
                IntStream.rangeClosed(2, 4)
                    .mapToObj(
                        coralScoreIndex ->
                            getUpInTheWaterSequence(coralScoreIndex, coralObjectives, autoTimer))
                    .toArray(Command[]::new)));
  }

  public Command upInTheWaterAuto() {
    ReefLevel level =
        Constants.getRobot() == Constants.RobotType.DEVBOT ? ReefLevel.L3 : ReefLevel.L4;
    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(9, level),
          new CoralObjective(10, level),
          new CoralObjective(11, level),
          new CoralObjective(0, level),
        };

    final Timer autoTimer = new Timer();
    return Commands.runOnce(
            () -> {
              AutoCommands.resetPose(new HolonomicTrajectory("UpInTheWater1Score"), true);
              autoTimer.restart();
              superstructure.setAutoStart();
            })
        .andThen(
            Commands.sequence(
                IntStream.rangeClosed(1, 4)
                    .mapToObj(
                        coralScoreIndex -> {
                          return getUpInTheWaterSequence(
                              coralScoreIndex, coralObjectives, autoTimer);
                        })
                    .toArray(Command[]::new)));
  }

  private SequentialCommandGroup getUpInTheWaterSequence(
      int coralScoreIndex, CoralObjective[] coralObjectives, Timer autoTimer) {
    CoralObjective coralObjective = coralObjectives[coralScoreIndex - 1];
    HolonomicTrajectory coralScoringTrajectory =
        new HolonomicTrajectory("UpInTheWater" + coralScoreIndex + "Score");
    HolonomicTrajectory coralIntakingTrajectory =
        coralScoreIndex != 4
            ? new HolonomicTrajectory("UpInTheWater" + coralScoreIndex + "Intake")
            : null;

    DriveTrajectory driveScoringCommand =
        AutoCommands.coralScoringTrajectory(drive, coralScoringTrajectory, coralObjective, true);
    DriveTrajectory driveIntakingCommand =
        coralScoreIndex != 4 ? new DriveTrajectory(drive, coralIntakingTrajectory, true) : null;

    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.sequence(
                    IntakeCommands.intake(superstructure, funnel)
                        .until(() -> timer.hasElapsed(coralScoringTrajectory.getDuration() - 1.6)),
                    AutoCommands.superstructureAimAndEjectCommand(
                        superstructure,
                        coralObjective,
                        true,
                        () ->
                            timer.hasElapsed(
                                coralScoringTrajectory.getDuration()
                                    - coralEjectTimeSeconds / 2.0)),
                    Commands.runOnce(
                        () ->
                            System.out.printf(
                                "Scored Coral #" + coralScoreIndex + " at %.2f\n",
                                autoTimer.get())),
                    objectiveTracker.requestScored(() -> MirrorUtil.apply(coralObjective)),
                    coralScoreIndex != 4
                        ? Commands.waitUntil(
                                () ->
                                    AutoScore.outOfDistanceToReef(
                                        RobotState.getInstance().getEstimatedPose(), 0.10))
                            .andThen(
                                IntakeCommands.intake(superstructure, funnel)
                                    .until(
                                        () ->
                                            timer.hasElapsed(
                                                coralScoringTrajectory.getDuration()
                                                    + coralIntakingTrajectory.getDuration()
                                                    + intakeTimeSeconds)))
                        : superstructure
                            .runGoal(
                                Superstructure.getScoringState(coralObjective.reefLevel(), false))
                            .until(() -> autoTimer.hasElapsed(15.3)))
                .deadlineFor(
                    // Drive sequence
                    Commands.sequence(
                        driveScoringCommand,
                        coralScoreIndex != 4 ? driveIntakingCommand : Commands.none())));
  }

  public Command upInTheWeedsAuto(boolean isElims) {
    final double intakeTimeSeconds = 0.15;
    final double driveToStationBiasSeconds = 0.2;

    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(9, isElims ? ReefLevel.L4 : ReefLevel.L2),
          new CoralObjective(10, ReefLevel.L4),
          new CoralObjective(0, ReefLevel.L2),
        };

    Timer autoTimer = new Timer();
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
            })
        .andThen(
            new DriveToPose(
                    drive,
                    () -> AutoScore.getCoralScorePose(MirrorUtil.apply(coralObjectives[0])),
                    () -> RobotState.getInstance().getEstimatedPose(),
                    () -> AllianceFlipUtil.apply(new Translation2d(-3.0, 0.0)),
                    () -> 0.0)
                .until(() -> AutoCommands.isXCrossed(startingLineX - 0.5, true)),
            Commands.sequence(
                IntStream.rangeClosed(0, 2)
                    .mapToObj(
                        index -> {
                          var driveToStation = new DriveToStation(drive, true);
                          Debouncer intakingDebouncer = new Debouncer(intakeTimeSeconds);
                          return AutoScore.getAutoScoreCommand(
                                  drive,
                                  superstructure,
                                  funnel,
                                  objectiveTracker::requestScored,
                                  () -> coralObjectives[index].reefLevel(),
                                  () -> Optional.of(MirrorUtil.apply(coralObjectives[index])))
                              .withTimeout(4.0)
                              .andThen(
                                  Commands.runOnce(
                                      () -> {
                                        System.out.printf(
                                            "Scored Coral #" + (index + 1) + " at %.2f\n",
                                            autoTimer.get());
                                        intakingDebouncer.calculate(false);
                                      }),
                                  new DriveToStation(
                                          drive,
                                          () ->
                                              AutoScore.getCoralScorePose(
                                                      MirrorUtil.apply(coralObjectives[index]))
                                                  .getTranslation()
                                                  .minus(FieldConstants.Reef.center)
                                                  .times(
                                                      AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                                          () -> 0.0,
                                          true)
                                      .alongWith(
                                          superstructure.runGoal(
                                              Superstructure.getScoringState(
                                                  coralObjectives[index].reefLevel(), false)))
                                      .withTimeout(driveToStationBiasSeconds),
                                  driveToStation
                                      .alongWith(IntakeCommands.intake(superstructure, funnel))
                                      .until(
                                          () ->
                                              intakingDebouncer.calculate(
                                                  driveToStation.isRunning()
                                                      && driveToStation.withinTolerance(
                                                          Units.inchesToMeters(5.0),
                                                          Rotation2d.fromDegrees(10.0)))));
                        })
                    .toArray(Command[]::new)));
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
}
