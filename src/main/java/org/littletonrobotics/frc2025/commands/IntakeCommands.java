// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class IntakeCommands {
  public static LoggedTunableNumber funnelVolts = new LoggedTunableNumber("Funnel/IntakeVolts", 8);
  public static LoggedTunableNumber outtakeVolts =
      new LoggedTunableNumber("Funnel/OuttakeVolts", -10);

  private IntakeCommands() {}

  public static Command intake(Superstructure superstructure, RollerSystem funnel) {
    return superstructure
        .runGoal(SuperstructureState.INTAKE)
        .alongWith(
            Commands.waitUntil(superstructure::atGoal).andThen(funnel.runRoller(funnelVolts)));
  }
}
