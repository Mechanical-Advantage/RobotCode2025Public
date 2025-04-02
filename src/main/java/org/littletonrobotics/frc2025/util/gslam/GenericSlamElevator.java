// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util.gslam;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class GenericSlamElevator<G extends GenericSlamElevator.SlamElevatorGoal> {

  public interface SlamElevatorGoal {
    DoubleSupplier getSlammingCurrent();

    boolean isStopAtGoal();

    SlamElevatorState getState();
  }

  public enum SlamElevatorState {
    IDLING,
    RETRACTING,
    EXTENDING
  }

  private final GenericSlamElevatorIO io;
  protected final GenericSlamElevatorIOInputsAutoLogged inputs =
      new GenericSlamElevatorIOInputsAutoLogged();

  private final String name;
  private final double staticTimeSecs;
  private final double minVelocityThresh;

  protected abstract G getGoal();

  private G lastGoal = null;

  protected boolean slammed = false;
  private final Timer staticTimer = new Timer();

  private boolean brakeModeEnabled = false;
  private BooleanSupplier coastModeSupplier = () -> false;

  private final Alert disconnected;

  /**
   * Creates a new GenericSlamElevator
   *
   * @param name Name of elevator.
   * @param io IO implementation of elevator.
   * @param staticTimeSecs Time that it takes for elevator to stop running after hitting the end of
   *     the elevator.
   * @param minVelocityThresh Minimum velocity threshold for elevator to start stopping at in
   *     rads/sec of the last sprocket.
   */
  public GenericSlamElevator(
      String name, GenericSlamElevatorIO io, double staticTimeSecs, double minVelocityThresh) {
    this.name = name;
    this.io = io;
    this.staticTimeSecs = staticTimeSecs;
    this.minVelocityThresh = minVelocityThresh;
    setBrakeMode(true);

    disconnected = new Alert(name + " disconnected!", Alert.AlertType.kWarning);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    coastModeSupplier = coastOverride;
  }

  private void setBrakeMode(boolean enable) {
    if (brakeModeEnabled == enable) return;
    brakeModeEnabled = enable;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Ensure brake mode is enabled
    if (DriverStation.isEnabled()) {
      setBrakeMode(true);
    }

    // Reset if changing goals
    if (lastGoal != null && getGoal() != lastGoal) {
      slammed = false;
      staticTimer.stop();
      staticTimer.reset();
    }
    // Set last goal
    lastGoal = getGoal();

    // Set alert
    disconnected.set(!inputs.data.motorConnected() && !Robot.isJITing());

    // Check if at goal.
    if (!slammed) {
      // Start static timer if within min velocity threshold.
      if (Math.abs(inputs.data.velocityRadsPerSec()) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.stop();
        staticTimer.reset();
      }
      // If we are finished with timer, finish goal.
      // Also assume we are at the goal if auto was started
      slammed = staticTimer.hasElapsed(staticTimeSecs) || DriverStation.isAutonomousEnabled();
    } else {
      staticTimer.stop();
      staticTimer.reset();
    }

    // Run to goal.
    if (!slammed) {
      io.runCurrent(getGoal().getSlammingCurrent().getAsDouble());
    } else {
      if (getGoal().isStopAtGoal()) {
        io.stop();
      } else {
        io.runCurrent(getGoal().getSlammingCurrent().getAsDouble());
      }
    }

    if (DriverStation.isDisabled()) {
      // Reset
      io.stop();
      lastGoal = null;
      staticTimer.stop();
      staticTimer.reset();
      if (Math.abs(inputs.data.velocityRadsPerSec()) > minVelocityThresh) {
        // If we don't move when disabled, assume we are still at goal
        slammed = false;
      }
    }

    // Update coast mode
    setBrakeMode(!coastModeSupplier.getAsBoolean());

    Logger.recordOutput("Chariot/Goal", getGoal().toString());
    Logger.recordOutput("Chariot/BrakeModeEnabled", brakeModeEnabled);
  }

  @AutoLogOutput(key = "Chariot/Slammed")
  public boolean slammed() {
    return slammed;
  }

  @AutoLogOutput(key = "Chariot/Extended")
  public boolean extended() {
    return getGoal().getState() == SlamElevatorState.EXTENDING && slammed;
  }

  @AutoLogOutput(key = "Chariot/Retracted")
  public boolean retracted() {
    return getGoal().getState() == SlamElevatorState.RETRACTING && slammed;
  }
}
