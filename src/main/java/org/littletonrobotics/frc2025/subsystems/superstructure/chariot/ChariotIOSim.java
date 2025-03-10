// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.chariot;

import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.util.gslam.GenericSlamElevatorIOSim;

public class ChariotIOSim extends GenericSlamElevatorIOSim implements ChariotIO {
  public ChariotIOSim() {
    super(SuperstructureConstants.chariotMaxExtension, 4.0, Chariot.drumRadius);
  }
}
