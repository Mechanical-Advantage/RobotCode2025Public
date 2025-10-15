// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
  @Builder.Default
  private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);

  @Builder.Default private final DoubleSupplier tunnelVolts = () -> 0.0;
  @Builder.Default private final Dispenser.GripperGoal gripperGoal = Dispenser.GripperGoal.IDLE;
  @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;
  @Builder.Default private final Height height = Height.BOTTOM;

  /** What height is the carriage above? */
  @RequiredArgsConstructor
  @Getter
  public enum Height {
    BOTTOM(0),
    FIRST_STAGE(SuperstructureConstants.stage1ToStage2Height),
    SECOND_STAGE(SuperstructureConstants.stage2ToStage3Height);

    private final double position;

    public boolean lowerThan(Height other) {
      return position <= other.position;
    }

    public boolean upperThan(Height other) {
      return position > other.position;
    }
  }
}
