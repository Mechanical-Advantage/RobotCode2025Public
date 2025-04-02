// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.util.swerve.ModuleLimits;

public class DriveConstants {
  public static final double odometryFrequency = 250;
  public static final double trackWidthX =
      Constants.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(22.75);
  public static final double trackWidthY =
      Constants.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(22.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(2.000);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(16)
        .turnMotorId(15)
        .encoderChannel(41)
        .encoderOffset(Rotation2d.fromRadians(2.5356702423749646))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(10)
        .turnMotorId(11)
        .encoderChannel(42)
        .encoderOffset(Rotation2d.fromRadians(-2.932971266437346))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(18)
        .turnMotorId(19)
        .encoderChannel(43)
        .encoderOffset(Rotation2d.fromRadians(0.6458059116998549))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(13)
        .turnMotorId(14)
        .encoderChannel(44)
        .encoderOffset(Rotation2d.fromRadians(-2.5187964537082226))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-0.009115335014721037))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(0.8427416931125384))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(-1.0620197413817225))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(-2.600063124240756))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = Constants.getRobot() == RobotType.DEVBOT ? 3 : 30;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
