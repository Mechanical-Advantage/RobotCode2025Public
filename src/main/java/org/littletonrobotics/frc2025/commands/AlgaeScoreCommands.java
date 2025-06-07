// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class AlgaeScoreCommands {
  private static final LoggedTunableNumber processLineupXOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupXOffset", 0.1);
  private static final LoggedTunableNumber processLineupYOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupYOffset", 0.0);
  private static final LoggedTunableNumber processLineupClear =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupClear", 0.3);
  private static final LoggedTunableNumber processEjectDegOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessEjectDegreeOffset", 15.0);
  private static final LoggedTunableNumber throwRaiseThetaToleranceDeg =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowRaiseThetaToleranceDeg", 20.0);
  private static final LoggedTunableNumber throwDriveStart =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveStart", 2.0);
  private static final LoggedTunableNumber throwDrivePreEnd =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDrivePreEnd", 1.5);
  private static final LoggedTunableNumber throwDriveEnd =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveEnd", 1.25);
  private static final LoggedTunableNumber throwReadyLinearTolerance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyLinearTolerance", 0.5);
  private static final LoggedTunableNumber throwReadyThetaToleranceDeg =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyThetaToleranceDegrees", 15.0);
  private static final LoggedTunableNumber forceProcessorMaxDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ForceProcessorMaxDistance", 0.5);

  @Accessors(fluent = true)
  @Getter
  private static boolean shouldForceProcess = false;

  @Getter
  private static final LoggedTunableNumber lookaheadSecs =
      new LoggedTunableNumber("AlgaeScoreCommands/LookaheadSecs", 0.75);

  public static Command process(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Supplier<Command> joystickDrive,
      BooleanSupplier onOpposingSide,
      BooleanSupplier holdingButton,
      boolean eject,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    Container<Pose2d> goalPose = new Container<>(Pose2d.kZero);
    return Commands.either(
            joystickDrive.get(),
            new DriveToPose(
                    drive,
                    () -> {
                      goalPose.value =
                          AllianceFlipUtil.apply(
                                  onOpposingSide.getAsBoolean()
                                      ? FieldConstants.Processor.opposingCenterFace
                                      : FieldConstants.Processor.centerFace)
                              .transformBy(
                                  GeomUtil.toTransform2d(
                                      DriveConstants.robotWidth / 2.0
                                          + processLineupXOffset.get()
                                          + (!eject
                                                  && superstructure.getState()
                                                      != SuperstructureState.PRE_PROCESS
                                              ? processLineupClear.get()
                                              : 0.0),
                                      processLineupYOffset.get()))
                              .transformBy(
                                  GeomUtil.toTransform2d(
                                      Rotation2d.kPi.plus(
                                          Rotation2d.fromDegrees(
                                              eject ? processEjectDegOffset.get() : 0.0))));
                      return AutoScoreCommands.getDriveTarget(
                          RobotState.getInstance().getEstimatedPose(), goalPose.value);
                    },
                    RobotState.getInstance()::getEstimatedPose,
                    () ->
                        DriveCommands.getLinearVelocityFromJoysticks(
                                driverX.getAsDouble(), driverY.getAsDouble())
                            .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                    () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()))
                .onlyWhile(holdingButton)
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoScoring = true,
                        () -> Leds.getInstance().autoScoring = false))
                .andThen(joystickDrive.get()),
            disableAlgaeScoreAutoAlign)
        .alongWith(
            eject
                ? superstructure.runGoal(SuperstructureState.PROCESS)
                : superstructure.runGoal(SuperstructureState.PRE_PROCESS),
            Commands.run(
                () ->
                    shouldForceProcess =
                        !disableAlgaeScoreAutoAlign.getAsBoolean()
                            && superstructure.getState() == SuperstructureState.PRE_PROCESS
                            && RobotState.getInstance()
                                    .getEstimatedPose()
                                    .getTranslation()
                                    .getDistance(goalPose.value.getTranslation())
                                < forceProcessorMaxDistance.get()))
        .finallyDo(() -> shouldForceProcess = false);
  }

  public static Command netScore(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      Command joystickDrive,
      BooleanSupplier eject,
      Command controllerRumbleCommand,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    var autoAlignCommand =
        new DriveToPose(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        MathUtil.interpolate(
                            FieldConstants.fieldLength / 2.0 - throwDriveStart.get(),
                            FieldConstants.fieldLength / 2.0
                                - (superstructure.getState() == SuperstructureState.THROW
                                        || superstructure.getState()
                                            == SuperstructureState.PRE_THROW
                                    ? throwDriveEnd.get()
                                    : throwDrivePreEnd.get()),
                            RobotState.getInstance().getElevatorExtensionPercent()),
                        MathUtil.clamp(
                            AllianceFlipUtil.applyY(
                                RobotState.getInstance().getEstimatedPose().getY()),
                            FieldConstants.fieldWidth / 2 + DriveConstants.robotWidth,
                            FieldConstants.fieldWidth - DriveConstants.robotWidth),
                        Rotation2d.kZero)),
            RobotState.getInstance()::getEstimatedPose,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> 0);

    return Commands.parallel(
            Commands.either(joystickDrive, autoAlignCommand, disableAlgaeScoreAutoAlign)
                .alongWith(
                    Commands.waitUntil(
                        () ->
                            autoAlignCommand.withinTolerance(
                                Double.POSITIVE_INFINITY,
                                Rotation2d.fromDegrees(throwRaiseThetaToleranceDeg.get()))),
                    superstructure.runGoal(
                        () ->
                            eject.getAsBoolean()
                                ? SuperstructureState.THROW
                                : SuperstructureState.PRE_THROW)),
            Commands.waitUntil(
                    () -> {
                      Leds.getInstance().ready = false;
                      return autoAlignCommand.isRunning()
                          && autoAlignCommand.withinTolerance(
                              throwReadyLinearTolerance.get(),
                              Rotation2d.fromDegrees(throwReadyThetaToleranceDeg.get()))
                          && superstructure.getState() == SuperstructureState.PRE_THROW;
                    })
                .andThen(
                    Commands.runOnce(() -> Leds.getInstance().ready = true),
                    controllerRumbleCommand
                        .withTimeout(0.1)
                        .andThen(Commands.waitSeconds(0.1).repeatedly())))
        .beforeStarting(() -> Leds.getInstance().autoScoring = true)
        .finallyDo(
            () -> {
              Leds.getInstance().ready = false;
              Leds.getInstance().autoScoring = false;
            });
  }
}
