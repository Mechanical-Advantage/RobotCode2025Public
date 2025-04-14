// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
  @AutoLog
  class SlamIOInputs {
    public SlamIOData data = new SlamIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record SlamIOData(
      boolean motorConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius) {}

  default void updateInputs(SlamIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
