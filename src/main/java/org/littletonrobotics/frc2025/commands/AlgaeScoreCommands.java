// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class AlgaeScoreCommands {
  private static final LoggedTunableNumber processLineupXOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupXOffset", 0.12);
  private static final LoggedTunableNumber processLineupYOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessLineupYOffset", 0.1);
  private static final LoggedTunableNumber processEjectDegOffset =
      new LoggedTunableNumber("AlgaeScoreCommands/ProcessEjectDegreeOffset", 15.0);
  private static final LoggedTunableNumber throwLineupDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowLineupDistance", 0.6);
  private static final LoggedTunableNumber throwDriveDistance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveDistance", 0.4);
  private static final LoggedTunableNumber throwDriveVelocity =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowDriveVelocity", 1.5);
  public static final LoggedTunableNumber throwGripperEjectTime =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowGripperEjectTime", 0.5);
  private static final LoggedTunableNumber throwReadyLinearTolerance =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyLinearTolerance", 0.4);
  private static final LoggedTunableNumber throwReadyThetaToleranceDeg =
      new LoggedTunableNumber("AlgaeScoreCommands/ThrowReadyThetaToleranceDegrees", 10.0);

  public static Command process(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier onOpposingSide,
      boolean eject,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    return Commands.either(
            joystickDrive,
            new DriveToPose(
                    drive,
                    () ->
                        AutoScoreCommands.getDriveTarget(
                            RobotState.getInstance().getEstimatedPose(),
                            AllianceFlipUtil.apply(
                                    onOpposingSide.getAsBoolean()
                                        ? FieldConstants.Processor.opposingCenterFace
                                        : FieldConstants.Processor.centerFace)
                                .transformBy(
                                    GeomUtil.toTransform2d(
                                        DriveConstants.robotWidth / 2.0
                                            + processLineupXOffset.get(),
                                        processLineupYOffset.get()))
                                .transformBy(
                                    GeomUtil.toTransform2d(
                                        Rotation2d.kPi.plus(
                                            Rotation2d.fromDegrees(
                                                eject ? processEjectDegOffset.get() : 0.0))))),
                    RobotState.getInstance()::getEstimatedPose,
                    () ->
                        DriveCommands.getLinearVelocityFromJoysticks(
                                driverX.getAsDouble(), driverY.getAsDouble())
                            .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                    () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()))
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoScoring = true,
                        () -> Leds.getInstance().autoScoring = false)),
            disableAlgaeScoreAutoAlign)
        .alongWith(
            eject
                ? superstructure.runGoal(SuperstructureState.PROCESSED)
                : superstructure.runGoal(SuperstructureState.ALGAE_STOW));
  }

  public static Command netThrowLineup(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverY,
      Command joystickDrive,
      BooleanSupplier disableAlgaeScoreAutoAlign) {
    var autoAlignCommand =
        new DriveToPose(
            drive,
            () ->
                new Pose2d(
                    AllianceFlipUtil.applyX(
                        FieldConstants.fieldLength / 2.0
                            - FieldConstants.Barge.netWidth / 2.0
                            - FieldConstants.algaeDiameter
                            - SuperstructureConstants.pivotToTunnelFront
                                * Math.cos(20.5 / 180.0 * Math.PI)
                            - SuperstructureConstants.elevatorMaxTravel
                                * SuperstructureConstants.elevatorAngle.getCos()
                            - SuperstructureConstants.dispenserOrigin2d.getX()
                            // Get Position of robot with algae up against the net rim
                            - AlgaeScoreCommands.throwLineupDistance
                                .get() // Move back by lineup distance
                        ),
                    RobotState.getInstance().getEstimatedPose().getY(),
                    AllianceFlipUtil.apply(Rotation2d.kZero)),
            RobotState.getInstance()::getEstimatedPose,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(0, driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> 0);

    return Commands.either(joystickDrive, autoAlignCommand, disableAlgaeScoreAutoAlign)
        .alongWith(
            Commands.waitUntil(
                    () ->
                        disableAlgaeScoreAutoAlign.getAsBoolean()
                            || (autoAlignCommand.isRunning()
                                && autoAlignCommand.withinTolerance(
                                    throwReadyLinearTolerance.get(),
                                    Rotation2d.fromDegrees(throwReadyThetaToleranceDeg.get()))))
                .andThen(superstructure.runGoal(SuperstructureState.PRE_THROWN)));
  }

  public static Command netThrowScore(Drive drive, Superstructure superstructure) {
    Container<Pose2d> startPose = new Container<>();
    Timer driveTimer = new Timer();
    return Commands.runOnce(
            () -> {
              startPose.value = RobotState.getInstance().getEstimatedPose();
              driveTimer.restart();
            })
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        startPose.value.transformBy(
                            GeomUtil.toTransform2d(
                                Math.min(
                                    driveTimer.get() * throwDriveVelocity.get(),
                                    throwDriveDistance.get()),
                                0)))
                .alongWith(superstructure.runGoal(SuperstructureState.THROWN)));
  }
}
