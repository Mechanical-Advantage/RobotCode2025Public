// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import static org.littletonrobotics.frc2025.FieldConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.commands.AutoTracker.IntakingLocation;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.MirrorUtil;

@RequiredArgsConstructor
public class AutoConstants {
  private static final LoggedTunableNumber stationIntakeRadius =
      new LoggedTunableNumber("Auto/StationIntakeRadius", 4.0);
  private static final LoggedTunableNumber stationIntakeXOffset =
      new LoggedTunableNumber("Auto/StationIntakeXOffset", -1.0);
  private static final LoggedTunableNumber stationIntakeYOffset =
      new LoggedTunableNumber("Auto/StationIntakeYOffset", -0.5);
  private static final LoggedTunableNumber stationIntakeAimXOffset =
      new LoggedTunableNumber("Auto/StationIntakeAimXOffset", 0.0);
  private static final LoggedTunableNumber stationIntakeAimYOffset =
      new LoggedTunableNumber("Auto/StationIntakeAimYOffset", -0.4);
  private static final LoggedTunableNumber maxTurnIceCreamIntake =
      new LoggedTunableNumber("Auto/MaxTurnIceCreamIntake", 100.0);
  private static final LoggedTunableNumber iceCreamIntakeOffset =
      new LoggedTunableNumber("Auto/IceCreamIntakeOffset", 0.05);

  static final Pose2d closeCageStart =
      new Pose2d(
          startingLineX - DriveConstants.robotWidth / 2.0,
          fieldWidth
              - Barge.closeCage.getY()
              - Barge.cageWidth / 2.0
              - DriveConstants.robotWidth / 2.0,
          Rotation2d.kCCW_Pi_2);
  static final Pose2d farCageStart =
      new Pose2d(
          startingLineX - DriveConstants.robotWidth / 2.0,
          fieldWidth
              - Barge.farCage.getY()
              + Barge.cageWidth / 2.0
              + DriveConstants.robotWidth / 2.0,
          Rotation2d.kCCW_Pi_2);

  public static Pose2d getIntakePose(Pose2d robot, IntakingLocation intakingLocation) {
    Pose2d transformedRobot = MirrorUtil.apply(AllianceFlipUtil.apply(robot));
    // Station intake
    if (intakingLocation == IntakingLocation.STATION) {
      Translation2d circleCenter =
          CoralStation.rightCenterFace
              .transformBy(
                  GeomUtil.toTransform2d(stationIntakeXOffset.get(), stationIntakeYOffset.get()))
              .getTranslation();
      Translation2d aimCenter =
          CoralStation.rightCenterFace
              .transformBy(
                  GeomUtil.toTransform2d(
                      stationIntakeAimXOffset.get(), stationIntakeAimYOffset.get()))
              .getTranslation();
      return AllianceFlipUtil.apply(
          MirrorUtil.apply(
              new Pose2d(
                  new Pose2d(
                          circleCenter,
                          transformedRobot.getTranslation().minus(circleCenter).getAngle())
                      .transformBy(GeomUtil.toTransform2d(stationIntakeRadius.get(), 0.0))
                      .getTranslation(),
                  transformedRobot.getTranslation().minus(aimCenter).getAngle())));
    }
    // Ice cream intake
    Translation2d iceCreamTranslation =
        StagingPositions.iceCreams[intakingLocation.getIceCreamIndex()];
    // Clamp angle to minimize turning
    Rotation2d angleOffset =
        transformedRobot
            .getTranslation()
            .minus(iceCreamTranslation)
            .getAngle()
            .minus(transformedRobot.getRotation());
    Rotation2d clampedAngleOffset =
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                MathUtil.inputModulus(angleOffset.getDegrees(), -180.0, 180.0),
                -maxTurnIceCreamIntake.get(),
                maxTurnIceCreamIntake.get()));
    Rotation2d goalAngle = transformedRobot.getRotation().plus(clampedAngleOffset);
    return AllianceFlipUtil.apply(
        MirrorUtil.apply(
            new Pose2d(iceCreamTranslation, goalAngle)
                .transformBy(
                    GeomUtil.toTransform2d(
                        DriveConstants.robotWidth / 2.0 + iceCreamIntakeOffset.get(), 0.0))));
  }
}
