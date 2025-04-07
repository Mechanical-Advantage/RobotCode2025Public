// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import lombok.Builder;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double objDetectConfidenceThreshold = 0.8;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 2200;
  private static double monoGain = 17.5;
  private static double monoDenoise = 1.0;
  //   private static int colorExposure = 4500;
  //   private static double colorGain = 5.0;

  public static LoggedTunableNumber[] pitchAdjustments =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
            };
        case COMPBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
            };
        default -> new LoggedTunableNumber[] {};
      };
  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.254,
                              0.2032,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(-20.0))))
                  .id("40265450")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.254,
                              -0.2032,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(20.0))))
                  .id("40265453")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build()
            };
        case COMPBOT ->
            new CameraConfig[] {
              // Front Left
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              0.2286,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(-20.0))))
                  .id("40530395")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build(),
              // Front Right
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              -0.2286,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(20.0))))
                  .id("40552081")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build(),
              // Back Up
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(3),
                              Units.inchesToMeters(10),
                              Units.inchesToMeters(25),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-40.0 + pitchAdjustments[2].get()),
                                  Units.degreesToRadians(198.394))))
                  .id("40265453")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build(),
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Supplier<Pose3d> pose,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}

  private VisionConstants() {}
}
