// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final Alert tempFault;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(name + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.data.connected() && !Robot.isJITing());
    tempFault.set(inputs.data.tempFault());

    // Record cycle time
    LoggedTracer.record(name);
  }

  @AutoLogOutput
  public Command runRoller(DoubleSupplier inputVolts) {
    return runEnd(() -> io.runVolts(inputVolts.getAsDouble()), io::stop);
  }
}
