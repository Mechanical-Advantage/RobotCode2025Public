// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface ReefControlsIO {
  @AutoLog
  class ReefControlsIOInputs {
    public int[] selectedLevel = new int[] {}; // 0 = L2, 1 = L3, 2 = L4
    public int[] level1State = new int[] {}; // Count
    public int[] level2State = new int[] {}; // Bitfield
    public int[] level3State = new int[] {}; // Bitfield
    public int[] level4State = new int[] {}; // Bitfield
    public int[] algaeState = new int[] {}; // Bitfield
    public boolean[] coopState = new boolean[] {}; // Boolean
  }

  default void updateInputs(ReefControlsIOInputs inputs) {}

  default void setSelectedLevel(int value) {}

  default void setLevel1State(int value) {}

  default void setLevel2State(int value) {}

  default void setLevel3State(int value) {}

  default void setLevel4State(int value) {}

  default void setAlgaeState(int value) {}

  default void setCoopState(boolean value) {}

  default void setElims(boolean isElims) {}
}
