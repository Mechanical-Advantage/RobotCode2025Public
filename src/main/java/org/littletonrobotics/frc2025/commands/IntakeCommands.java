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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {
  public static LoggedTunableNumber funnelVolts = new LoggedTunableNumber("Funnel/IntakeVolts", 8);
  public static LoggedTunableNumber outtakeVolts =
      new LoggedTunableNumber("Funnel/OuttakeVolts", -10);
  private static final LoggedTunableNumber algaeSearchSpeed =
      new LoggedTunableNumber("IntakeCommands/Algae/SearchSpeed", 2.0);
  private static final LoggedTunableNumber algaeKp =
      new LoggedTunableNumber("IntakeCommands/Algae/kP", 4.0);
  private static final LoggedTunableNumber algaeKd =
      new LoggedTunableNumber("IntakeCommands/Algae/kD", 0.0);

  private IntakeCommands() {}

  public static Command intake(Superstructure superstructure, RollerSystem funnel) {
    return superstructure
        .runGoal(
            () ->
                superstructure.hasAlgae()
                    ? SuperstructureState.ALGAE_CORAL_INTAKE
                    : SuperstructureState.CORAL_INTAKE)
        .alongWith(
            funnel
                .runRoller(outtakeVolts)
                .until(superstructure::atGoal)
                .andThen(funnel.runRoller(funnelVolts)));
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
