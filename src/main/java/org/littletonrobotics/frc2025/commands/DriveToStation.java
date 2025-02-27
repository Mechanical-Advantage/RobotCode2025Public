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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToStation extends DriveToPose {
  private static final LoggedTunableNumber stationAlignDistance =
      new LoggedTunableNumber(
          "DriveToStation/StationAlignDistance",
          DriveConstants.robotWidth / 2.0 + Units.inchesToMeters(2.0));
  private static final LoggedTunableNumber horizontalMaxOffset =
      new LoggedTunableNumber(
          "DriveToStation/HorizontalMaxOffset",
          FieldConstants.CoralStation.stationLength / 2 - Units.inchesToMeters(16));

  public DriveToStation(Drive drive, boolean sideIntaking) {
    this(drive, () -> 0, () -> 0, () -> 0, sideIntaking);
  }

  public DriveToStation(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      boolean sideIntaking) {
    this(
        drive,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () ->
            Math.copySign(
                Math.pow(
                    MathUtil.applyDeadband(driverOmega.getAsDouble(), DriveCommands.DEADBAND), 2.0),
                driverOmega.getAsDouble()),
        sideIntaking);
  }

  public DriveToStation(
      Drive drive, Supplier<Translation2d> linearFF, DoubleSupplier theta, boolean sideIntaking) {
    super(
        drive,
        () -> {
          Pose2d curPose = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());

          List<Pose2d> finalPoses = new ArrayList<>();
          for (Pose2d stationCenter :
              new Pose2d[] {
                FieldConstants.CoralStation.leftCenterFace,
                FieldConstants.CoralStation.rightCenterFace
              }) {
            Transform2d offset = new Transform2d(stationCenter, curPose);
            offset =
                new Transform2d(
                    stationAlignDistance.get(),
                    MathUtil.clamp(
                        offset.getY(), -horizontalMaxOffset.get(), horizontalMaxOffset.get()),
                    new Rotation2d());

            Rotation2d rotationOffset = curPose.getRotation().minus(stationCenter.getRotation());
            if (Math.abs(rotationOffset.getDegrees()) > 45 && sideIntaking) {
              finalPoses.add(
                  new Pose2d(
                      stationCenter.transformBy(offset).getTranslation(),
                      stationCenter
                          .getRotation()
                          .rotateBy(
                              Rotation2d.fromDegrees(
                                  Math.copySign(90, rotationOffset.getDegrees())))));
            } else {
              finalPoses.add(stationCenter.transformBy(offset));
            }
          }
          Logger.recordOutput(
              "DriveToStation/LeftClosestPose", AllianceFlipUtil.apply(finalPoses.get(0)));
          Logger.recordOutput(
              "DriveToStation/RightClosestPose", AllianceFlipUtil.apply(finalPoses.get(1)));
          return AllianceFlipUtil.apply(curPose.nearest(finalPoses));
        },
        RobotState.getInstance()::getEstimatedPose,
        linearFF,
        theta);
  }
}
