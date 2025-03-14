// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public GyroIOData data =
        new GyroIOData(false, Rotation2d.kZero, 0, Rotation2d.kZero, 0, Rotation2d.kZero, 0);
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public record GyroIOData(
      boolean connected,
      Rotation2d yawPosition,
      double yawVelocityRadPerSec,
      Rotation2d pitchPosition,
      double pitchVelocityRadPerSec,
      Rotation2d rollPosition,
      double rollVelocityRadPerSec) {}

  public default void updateInputs(GyroIOInputs inputs) {}
}
