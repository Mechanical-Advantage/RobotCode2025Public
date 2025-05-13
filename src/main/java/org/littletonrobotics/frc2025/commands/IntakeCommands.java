// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.intake.Intake;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2025.util.*;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {
  private static final LoggedTunableNumber lookAheadSecs =
      new LoggedTunableNumber("IntakeCommands/LookAheadSecs", 0.1);
  public static final LoggedTunableNumber angleDifferenceWeight =
      new LoggedTunableNumber("IntakeCommands/AngleDifferenceWeight", 0.3);
  private static final LoggedTunableNumber coralMaxDistance =
      new LoggedTunableNumber("IntakeCommands/CoralMaxDistance", 1.5);
  public static final LoggedTunableNumber coralMaxAngleDeg =
      new LoggedTunableNumber("IntakeCommands/CoralMaxAngleDegrees", 60.0);
  private static final LoggedTunableNumber coralMaxXDistance =
      new LoggedTunableNumber("IntakeCommands/SLA/CoralMaxXDistance", 1.0);
  private static final LoggedTunableNumber coralMinXDistance =
      new LoggedTunableNumber("IntakeCommands/SLA/CoralMinXDistance", 0.1);
  private static final LoggedTunableNumber coralMaxYDistance =
      new LoggedTunableNumber("IntakeCommands/SLA/CoralMaxYDistance", 1.0);
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("IntakeCommands/SLA/DrivekP", 1.0);
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("IntakeCommands/SLA/DrivekD", 0.0);
  private static final LoggedTunableNumber driveMinOutput =
      new LoggedTunableNumber("IntakeCommands/SLA/MinOutput", 0.1);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("IntakeCommands/SLA/ThetaTolerance", 45.0);
  private static final LoggedTunableNumber intakingOffset =
      new LoggedTunableNumber("IntakeCommands/IntakingOffset", Units.inchesToMeters(4.0));
  private static final LoggedTunableNumber algaeSearchSpeed =
      new LoggedTunableNumber("IntakeCommands/Algae/SearchSpeed", 2.0);
  private static final LoggedTunableNumber algaeKp =
      new LoggedTunableNumber("IntakeCommands/Algae/kP", 4.0);
  private static final LoggedTunableNumber algaeKd =
      new LoggedTunableNumber("IntakeCommands/Algae/kD", 0.0);

  private IntakeCommands() {}

  public static Command simpleLittleAutomation(
      Drive drive,
      Intake intake,
      BooleanSupplier hasCoral,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Command controllerRumble,
      BooleanSupplier robotRelative,
      BooleanSupplier disableIntakeAutoAlign) {

    @SuppressWarnings("resource")
    var driveController = new PIDController(0.0, 0.0, 0.0);
    Object tunableId = new Object();
    Container<Double> robotToCoralError = new Container<>(-1.0);

    return Commands.either(
            joystickDrive,
            drive
                .run(
                    () -> {
                      if (drivekP.hasChanged(tunableId.hashCode())
                          || drivekD.hasChanged(tunableId.hashCode())) {
                        driveController.setPID(drivekP.get(), 0.0, drivekD.get());
                      }

                      Translation2d linearVelocity =
                          DriveCommands.getLinearVelocityFromJoysticks(
                              driverX.getAsDouble(), driverY.getAsDouble());
                      double omega = DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble());
                      ChassisSpeeds wantedSpeeds =
                          new ChassisSpeeds(
                              linearVelocity.getX() * DriveConstants.maxLinearSpeed,
                              linearVelocity.getY() * DriveConstants.maxLinearSpeed,
                              omega * DriveConstants.maxAngularSpeed);
                      wantedSpeeds =
                          robotRelative.getAsBoolean()
                              ? wantedSpeeds
                              : ChassisSpeeds.fromFieldRelativeSpeeds(
                                  wantedSpeeds,
                                  AllianceFlipUtil.shouldFlip()
                                      ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                                      : RobotState.getInstance().getRotation());

                      // Use joysticks to infer wanted coral
                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                      Pose2d predictedRobot =
                          robot.exp(wantedSpeeds.toTwist2d(lookAheadSecs.get()));
                      Transform2d robotToIntake =
                          new Transform2d(
                              -(DriveConstants.robotWidth + intakingOffset.get()),
                              0.0,
                              Rotation2d.kPi);

                      Logger.recordOutput(
                          "IntakeCommands/SLAPredictedRobot", new Pose2d[] {predictedRobot});

                      // Find nearest coral
                      var coralTranslation = getNearestCoral();
                      // Log targeted coral
                      Logger.recordOutput(
                          "IntakeCommands/TargetedCoral",
                          coralTranslation
                              .map(
                                  coral ->
                                      new Translation3d[] {
                                        new Translation3d(
                                            coral.getX(),
                                            coral.getY(),
                                            FieldConstants.coralDiameter)
                                      })
                              .orElseGet(() -> new Translation3d[] {}));
                      if (coralTranslation.isEmpty() || hasCoral.getAsBoolean()) {
                        // Output joystick speeds
                        Logger.recordOutput("IntakeCommands/SLA/WantedSpeeds", wantedSpeeds);
                        driveController.reset();
                        drive.runVelocity(wantedSpeeds);
                        robotToCoralError.value = -1.0;
                        return;
                      }

                      // Nudge wanted speeds in direction of coral
                      Pose2d intakePose = robot.plus(robotToIntake);
                      Translation2d intakeToCoralError =
                          new Pose2d(coralTranslation.get(), intakePose.getRotation())
                              .relativeTo(intakePose)
                              .getTranslation();
                      final double driveError = intakeToCoralError.getNorm();
                      robotToCoralError.value = driveError;
                      boolean shouldDrive = true;
                      if (Math.abs(intakeToCoralError.getY()) >= coralMaxYDistance.get()
                          || Math.abs(intakeToCoralError.getX()) >= coralMaxXDistance.get()) {
                        driveController.reset();
                        shouldDrive = false;
                      }

                      Translation2d driveVelocity =
                          new Pose2d(
                                  new Translation2d(
                                      shouldDrive
                                          ? driveController.calculate(driveError, 0.0)
                                          : 0.0,
                                      intakeToCoralError.getAngle()),
                                  intakeToCoralError.getAngle())
                              .transformBy(robotToIntake.inverse())
                              .getTranslation();
                      wantedSpeeds =
                          new ChassisSpeeds(
                              wantedSpeeds.vxMetersPerSecond
                                  + (intakeToCoralError.getX() <= 0.1
                                          ? 0.0
                                          : driveVelocity.getX()
                                              * MathUtil.clamp(
                                                  (driveError - coralMinXDistance.get())
                                                      / (coralMinXDistance.get() + 1.0),
                                                  0.0,
                                                  1.0))
                                      * linearVelocity.getNorm(),
                              wantedSpeeds.vyMetersPerSecond
                                  + driveVelocity.getY()
                                      * MathUtil.clamp(
                                          linearVelocity.getNorm() + driveMinOutput.get(),
                                          0.0,
                                          1.0),
                              wantedSpeeds.omegaRadiansPerSecond
                                  + (Math.abs(intakeToCoralError.getAngle().getDegrees())
                                                  <= thetaTolerance.get()
                                              || Math.abs(intakeToCoralError.getY()) <= 0.1
                                          ? 0.0
                                          : Math.signum(intakeToCoralError.getY()))
                                      * (1.0 - Math.abs(omega)));
                      Logger.recordOutput("IntakeCommands/SLA/WantedSpeeds", wantedSpeeds);
                      drive.runVelocity(wantedSpeeds);
                    })
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().coralIntaking = true,
                        () -> Leds.getInstance().coralIntaking = false),
                    Commands.sequence(
                        new SuppliedWaitCommand(
                                () ->
                                    robotToCoralError.value < 0.0
                                        ? 9999.0
                                        : (MathUtil.clamp(
                                                    (robotToCoralError.value - 0.1) / 1.4, 0.0, 1.0)
                                                * 0.35)
                                            + 0.05)
                            .andThen(
                                new SuppliedWaitCommand(
                                        () ->
                                            (MathUtil.clamp(
                                                        (robotToCoralError.value - 0.1) / 1.4,
                                                        0.0,
                                                        1.0)
                                                    * 0.35)
                                                + 0.05)
                                    .deadlineFor(controllerRumble)))),
            disableIntakeAutoAlign)
        .alongWith(intake.intake())
        .finallyDo(
            () -> {
              Logger.recordOutput("IntakeCommands/SLA/PredictedRobot", new Pose2d[] {});
              Logger.recordOutput("IntakeCommands/SLA/TargetedCoral", new Translation3d[] {});
            });
  }

  public static Command teleopAutoIntake(
      Drive drive,
      Intake intake,
      BooleanSupplier hasCoral,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableAutoCoralIntake) {
    return autoIntake(
        drive,
        intake,
        IntakeCommands::getNearestCoral,
        hasCoral,
        driverX,
        driverY,
        driverOmega,
        joystickDrive,
        robotRelative,
        disableAutoCoralIntake);
  }

  public static Command autoIntake(
      Drive drive,
      Intake intake,
      Supplier<Optional<Translation2d>> targetedCoral,
      BooleanSupplier hasCoral) {
    return autoIntake(
        drive,
        intake,
        targetedCoral,
        hasCoral,
        () -> 0.0,
        () -> 0.0,
        () -> 0.0,
        Commands.none(),
        () -> false,
        () -> false);
  }

  private static Command autoIntake(
      Drive drive,
      Intake intake,
      Supplier<Optional<Translation2d>> targetedCoral,
      BooleanSupplier hasCoral,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableAutoCoralIntake) {
    return Commands.either(
            joystickDrive,
            new DriveToPose(
                    drive,
                    () -> {
                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                      if (hasCoral.getAsBoolean()) {
                        return robot;
                      }

                      return targetedCoral
                          .get()
                          .map(
                              coral -> {
                                Logger.recordOutput(
                                    "IntakeCommands/TargetedCoral",
                                    new Translation3d[] {
                                      new Translation3d(
                                          coral.getX(), coral.getY(), FieldConstants.coralDiameter)
                                    });
                                return new Pose2d(
                                        coral, robot.getTranslation().minus(coral).getAngle())
                                    .transformBy(
                                        new Transform2d(
                                            DriveConstants.robotWidth / 2.0 + intakingOffset.get(),
                                            0.0,
                                            Rotation2d.kZero));
                              })
                          .orElseGet(
                              () -> {
                                Logger.recordOutput(
                                    "IntakeCommands/TargetedCoral", new Translation3d[] {});
                                return RobotState.getInstance().getEstimatedPose();
                              });
                    },
                    RobotState.getInstance()::getEstimatedPose,
                    () ->
                        DriveCommands.getLinearVelocityFromJoysticks(
                                driverX.getAsDouble(), driverY.getAsDouble())
                            .times(
                                AllianceFlipUtil.shouldFlip() && !robotRelative.getAsBoolean()
                                    ? -1.0
                                    : 1.0)
                            .rotateBy(
                                robotRelative.getAsBoolean()
                                    ? RobotState.getInstance().getRotation()
                                    : Rotation2d.kZero),
                    () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()))
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoScoring = true,
                        () -> Leds.getInstance().autoScoring = false)),
            disableAutoCoralIntake)
        .alongWith(intake.intake())
        .finallyDo(
            () -> Logger.recordOutput("IntakeCommands/TargetedCoral", new Translation3d[] {}));
  }

  private static Optional<Translation2d> getNearestCoral() {
    Pose2d robot = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotVelocity = RobotState.getInstance().getRobotVelocity();
    Pose2d predictedRobot = robot.exp(robotVelocity.toTwist2d(lookAheadSecs.get()));
    Pose2d predictedIntakePose =
        predictedRobot.transformBy(
            new Transform2d(
                -(DriveConstants.robotWidth + intakingOffset.get()), 0.0, Rotation2d.kPi));
    Logger.recordOutput("IntakeCommands/PredictedRobot", predictedRobot);

    return RobotState.getInstance().getCoralTranslations().stream()
        .filter(
            coral ->
                coral.getDistance(predictedIntakePose.getTranslation()) <= coralMaxDistance.get()
                    && Math.abs(
                            predictedIntakePose.getRotation().getDegrees()
                                - (coral
                                    .minus(predictedIntakePose.getTranslation())
                                    .getAngle()
                                    .getDegrees()))
                        <= coralMaxAngleDeg.get())
        .min(
            Comparator.comparingDouble(
                coral ->
                    coral.getDistance(predictedIntakePose.getTranslation())
                        + Math.abs(
                            coral
                                    .minus(predictedIntakePose.getTranslation())
                                    .getAngle()
                                    .minus(predictedIntakePose.getRotation())
                                    .getRadians()
                                * angleDifferenceWeight.get())));
  }

  public static Command algaeIntakeWithAiming(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverTheta,
      BooleanSupplier robotRelative,
      BooleanSupplier disableIntakeAutoAlign) {
    @SuppressWarnings("resource")
    var controller = new PIDController(algaeKp.get(), 0, algaeKd.get());
    var tunableId = new Object();
    return Commands.run(
            () -> {
              // Get translation velocity
              Translation2d linearVelocity =
                  DriveCommands.getLinearVelocityFromJoysticks(
                          driverX.getAsDouble(), driverY.getAsDouble())
                      .times(DriveConstants.maxLinearSpeed);
              double driverOmega = DriveCommands.getOmegaFromJoysticks(driverTheta.getAsDouble());

              // Update controller
              if (algaeKp.hasChanged(tunableId.hashCode())
                  || algaeKd.hasChanged(tunableId.hashCode())) {
                controller.setPID(algaeKp.get(), 0.0, algaeKd.get());
              }

              // Get observations
              var leftObservation = RobotState.getInstance().getLeftAlgaeObservation();
              var rightObservation = RobotState.getInstance().getRightAlgaeObservation();

              // Calculate omega velocity
              double autoOmega = 0.0;
              var algaeTranslations = new Translation3d[] {};
              if (leftObservation.isEmpty() && rightObservation.isPresent()) {
                autoOmega = algaeSearchSpeed.get();
              } else if (leftObservation.isPresent() && rightObservation.isEmpty()) {
                autoOmega = -algaeSearchSpeed.get();
              } else if (leftObservation.isPresent() && rightObservation.isPresent()) {
                var leftVector =
                    VisionConstants.cameras[0]
                        .pose()
                        .get()
                        .toPose2d()
                        .transformBy(new Transform2d(0.0, 0.0, leftObservation.get()));
                var rightVector =
                    VisionConstants.cameras[1]
                        .pose()
                        .get()
                        .toPose2d()
                        .transformBy(new Transform2d(0.0, 0.0, rightObservation.get()));

                double mLeft = leftVector.getRotation().getTan();
                double mRight = rightVector.getRotation().getTan();

                double algaeX = (rightVector.getY() - leftVector.getY()) / (mLeft - mRight);
                double algaeY = mLeft * algaeX + leftVector.getY();
                double algaeAngle = Math.atan2(algaeY, algaeX + leftVector.getX());
                autoOmega = controller.calculate(-algaeAngle);

                Logger.recordOutput("IntakeCommands/AlgaeX", algaeX);
                Logger.recordOutput("IntakeCommands/AlgaeY", algaeY);
                Logger.recordOutput("IntakeCommands/AlgaeAngle", algaeAngle);

                Translation2d algaeTranslation =
                    RobotState.getInstance()
                        .getEstimatedPose()
                        .transformBy(GeomUtil.toTransform2d(algaeX, algaeY))
                        .getTranslation();
                algaeTranslations =
                    new Translation3d[] {
                      new Translation3d(
                          algaeTranslation.getX(),
                          algaeTranslation.getY(),
                          FieldConstants.algaeDiameter / 2.0)
                    };
              }
              Logger.recordOutput("IntakeCommands/TargetedAlgae", algaeTranslations);

              // Merge driver and auto omega
              final double thetaS = MathUtil.clamp(Math.abs(driverOmega) * 3.0, 0.0, 1.0);
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                      disableIntakeAutoAlign.getAsBoolean()
                          ? driverOmega * DriveConstants.maxAngularSpeed
                          : MathUtil.interpolate(
                              autoOmega, driverOmega * DriveConstants.maxAngularSpeed, thetaS));

              // Apply speeds
              drive.runVelocity(
                  robotRelative.getAsBoolean()
                      ? speeds
                      : ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          DriverStation.getAlliance().isPresent()
                                  && DriverStation.getAlliance().get() == Alliance.Red
                              ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                              : RobotState.getInstance().getRotation()));
            },
            drive)
        .alongWith(superstructure.runGoal(SuperstructureState.ALGAE_GROUND_INTAKE))
        .until(superstructure::hasAlgae)
        .finallyDo(
            () -> Logger.recordOutput("IntakeCommands/TargetedAlgae", new Translation3d[] {}));
  }
}
