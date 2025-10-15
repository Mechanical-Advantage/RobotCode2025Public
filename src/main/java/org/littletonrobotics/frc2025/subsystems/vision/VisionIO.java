// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean ntConnected = false;
  }

  @AutoLog
  class AprilTagVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  @AutoLog
  class ObjDetectVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  default void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {}

  default void setRecording(boolean active) {}
}
