// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.chariot;

import org.littletonrobotics.frc2025.util.gslam.GenericSlamElevatorIOKrakenFOC;

public class ChariotIOTalonFX extends GenericSlamElevatorIOKrakenFOC implements ChariotIO {
  public ChariotIOTalonFX() {
    super(8, "*", 60, false, 4.0);
  }
}
