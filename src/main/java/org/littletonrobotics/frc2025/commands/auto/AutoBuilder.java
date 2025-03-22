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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.IntStream;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.*;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.MirrorUtil;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final RollerSystem funnel;
  private final BooleanSupplier upInThePush;

  private final double upInThePushSecs = 0.5;

  public Command upInTheWaterAuto(boolean isSuper) {
    final double intakeTimeSeconds = 0.15;
    final double driveToStationBiasSeconds = 0.2;

    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(8, isSuper ? ReefLevel.L4 : ReefLevel.L2),
          new CoralObjective(10, ReefLevel.L4),
          new CoralObjective(11, ReefLevel.L4),
          new CoralObjective(0, ReefLevel.L2)
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
            getUpInThePush(),
            Commands.sequence(
                IntStream.rangeClosed(0, 3)
                    .mapToObj(
                        index -> {
                          var driveToStation = new DriveToStation(drive, true);
                          Debouncer intakingDebouncer = new Debouncer(intakeTimeSeconds);
                          return AutoScoreCommands.autoScore(
                                  drive,
                                  superstructure,
                                  funnel,
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
                                              AutoScoreCommands.getCoralScorePose(
                                                      MirrorUtil.apply(coralObjectives[index]),
                                                      false)
                                                  .getTranslation()
                                                  .minus(FieldConstants.Reef.center)
                                                  .times(
                                                      AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                                          () -> 0.0,
                                          true)
                                      .alongWith(
                                          superstructure.runGoal(
                                              Superstructure.getScoringState(
                                                  coralObjectives[index].reefLevel(),
                                                  false,
                                                  false)))
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
                        AutoScoreCommands.getCoralScorePose(objective, false)
                            .plus(new Transform2d(-0.5, 0.0, Rotation2d.kZero)))
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
