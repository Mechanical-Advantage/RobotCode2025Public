// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util.gslam;

import org.littletonrobotics.junction.AutoLog;

public interface GenericSlamElevatorIO {
  @AutoLog
  class GenericSlamElevatorIOInputs {
    public GenericSlamElevatorIOData data =
        new GenericSlamElevatorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record GenericSlamElevatorIOData(
      boolean motorConnected,
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius) {}

  /** Update the inputs. */
  default void updateInputs(GenericSlamElevatorIOInputs inputs) {}

  /** Run slam elevator at amps */
  default void runCurrent(double amps) {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}
}
