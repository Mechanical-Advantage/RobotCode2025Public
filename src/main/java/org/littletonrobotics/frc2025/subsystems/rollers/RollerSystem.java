// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import lombok.Setter;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class RollerSystem {
  private final String name;
  private final String inputsName;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final Alert tempFault;

  @Setter private double volts = 0.0;
  private boolean brakeModeEnabled = true;

  public RollerSystem(String name, String inputsName, RollerSystemIO io) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(name + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(!inputs.data.connected() && !Robot.isJITing());
    tempFault.set(inputs.data.tempFault());

    // Run roller
    io.runVolts(volts);

    // Record cycle time
    LoggedTracer.record(name);

    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public double getTorqueCurrent() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getVelocity() {
    return inputs.data.velocityRadsPerSec();
  }

  public void stop() {
    volts = 0.0;
  }
}
