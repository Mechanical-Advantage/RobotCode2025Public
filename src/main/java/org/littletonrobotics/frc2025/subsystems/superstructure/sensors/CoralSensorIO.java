// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  class CoralSensorIOInputs {
    public CoralSensorIOData data = new CoralSensorIOData(0, false);
  }

  record CoralSensorIOData(double distanceMeters, boolean valid) {}

  default void updateInputs(CoralSensorIOInputs inputs) {}
}
