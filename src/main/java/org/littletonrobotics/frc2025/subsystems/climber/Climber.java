// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private static final LoggedTunableNumber deployCurrent =
      new LoggedTunableNumber("Climber/DeployCurrent", 30);
  private static final LoggedTunableNumber deployAngle =
      new LoggedTunableNumber("Climber/DeployAngle", 130);
  private static final LoggedTunableNumber climbCurrent =
      new LoggedTunableNumber("Climber/ClimbCurrent", 65);
  private static final LoggedTunableNumber climbCurrentRampRate =
      new LoggedTunableNumber("Climber/ClimbCurrentRampRate", 80);
  static final LoggedTunableNumber climbStopAngle =
      new LoggedTunableNumber("Climber/ClimbStopAngle", 225);

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  @Setter private BooleanSupplier coastOverride = () -> false;

  @AutoLogOutput(key = "Climber/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
    climberIO.setBrakeMode(true);
  }

  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      climberIO.runTorqueCurrent(0.0);
    }

    // Set brake mode
    boolean coast = coastOverride.getAsBoolean() && DriverStation.isDisabled();
    setBrakeMode(!coast);
    Logger.recordOutput("Climber/CoastOverride", !coast);

    // Record cycle time
    LoggedTracer.record("Climber");
  }

  public Command deploy() {
    return run(() -> climberIO.runTorqueCurrent(deployCurrent.get()))
        .until(() -> climberInputs.data.positionRads() >= Units.degreesToRadians(deployAngle.get()))
        .finallyDo(() -> climberIO.runTorqueCurrent(0.0));
  }

  public Command climb() {
    Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(
                () -> {
                  boolean stopped =
                      climberInputs.data.positionRads()
                          >= Units.degreesToRadians(climbStopAngle.get());
                  if (stopped) {
                    timer.restart();
                  }
                  climberIO.runTorqueCurrent(
                      stopped
                          ? 0.0
                          : Math.min(climbCurrentRampRate.get() * timer.get(), climbCurrent.get()));
                }))
        .finallyDo(() -> climberIO.runTorqueCurrent(0.0));
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    climberIO.setBrakeMode(enabled);
  }
}
