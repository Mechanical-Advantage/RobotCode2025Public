// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Comparator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToCoral extends DriveToPose {
  private static final LoggedTunableNumber lookAheadSecs =
      new LoggedTunableNumber("DriveToCoral/LookAheadSecs", 0.3);
  private static final LoggedTunableNumber angleDifferenceWeight =
      new LoggedTunableNumber("DriveToCoral/AngleDifferenceWeight", 0.3);
  private static final LoggedTunableNumber coralMaxDistance =
      new LoggedTunableNumber("DriveToCoral/CoralMaxDistance", 1.4);
  private static final LoggedTunableNumber coralMaxAngleDeg =
      new LoggedTunableNumber("DriveToCoral/CoralMaxAngleDegrees", 70.0);

  public DriveToCoral(
      Drive drive, DoubleSupplier driverX, DoubleSupplier driverY, DoubleSupplier driverOmega) {
    super(
        drive,
        () -> {
          RobotState instance = RobotState.getInstance();
          Pose2d robot = instance.getEstimatedPose();

          ChassisSpeeds robotVelocity = instance.getRobotVelocity();

          Pose2d predictedRobot =
              instance.getEstimatedPose().exp(robotVelocity.toTwist2d(lookAheadSecs.get()));
          Logger.recordOutput("DriveToCoral/LookAheadPose", predictedRobot);

          return instance.getCoralTranslations().stream()
              .min(
                  Comparator.comparingDouble(
                      coral ->
                          coral.getDistance(predictedRobot.getTranslation())
                              + Math.abs(
                                  coral
                                          .minus(robot.getTranslation())
                                          .getAngle()
                                          .minus(robot.getRotation())
                                          .getRadians()
                                      * angleDifferenceWeight.get())))
              .filter(
                  coral ->
                      coral.getDistance(predictedRobot.getTranslation()) <= coralMaxDistance.get()
                          && Math.abs(
                                  predictedRobot.getRotation().rotateBy(Rotation2d.kPi).getDegrees()
                                      - (coral
                                          .minus(predictedRobot.getTranslation())
                                          .getAngle()
                                          .getDegrees()))
                              <= coralMaxAngleDeg.get())
              .map(
                  coral -> {
                    Logger.recordOutput("DriveToCoral/TargetedCoral", new Translation2d[] {coral});
                    return new Pose2d(coral, robot.getTranslation().minus(coral).getAngle())
                        .transformBy(
                            new Transform2d(
                                DriveConstants.robotWidth / 2.0, 0.0, Rotation2d.kZero));
                  })
              .orElseGet(
                  () -> {
                    Logger.recordOutput("DriveToCoral/TargetedCoral", new Translation2d[] {});
                    return RobotState.getInstance().getEstimatedPose();
                  });
        },
        RobotState.getInstance()::getEstimatedPose,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
  }
}
