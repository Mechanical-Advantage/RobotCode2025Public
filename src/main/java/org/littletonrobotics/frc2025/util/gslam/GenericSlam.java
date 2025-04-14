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

public abstract class GenericSlam<G extends GenericSlam.SlamGoal> {

  public interface SlamGoal {
    DoubleSupplier getSlammingCurrent();

    boolean isStopAtGoal();

    SlamState getState();
  }

  public enum SlamState {
    IDLING,
    RETRACTING,
    EXTENDING
  }

  private final GenericSlamIO io;
  protected final GenericSlamIOInputsAutoLogged inputs = new GenericSlamIOInputsAutoLogged();

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
  public GenericSlam(
      String name, GenericSlamIO io, double staticTimeSecs, double minVelocityThresh) {
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
      io.runTorqueCurrent(getGoal().getSlammingCurrent().getAsDouble());
    } else {
      if (getGoal().isStopAtGoal()) {
        io.stop();
      } else {
        io.runTorqueCurrent(getGoal().getSlammingCurrent().getAsDouble());
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

    Logger.recordOutput(name + "/Goal", getGoal().toString());
    Logger.recordOutput(name + "/BrakeModeEnabled", brakeModeEnabled);
  }

  @AutoLogOutput(key = "{name}/Slammed")
  public boolean slammed() {
    return slammed;
  }

  @AutoLogOutput(key = "{name}/Extended")
  public boolean extended() {
    return getGoal().getState() == SlamState.EXTENDING && slammed;
  }

  @AutoLogOutput(key = "{name}/Retracted")
  public boolean retracted() {
    return getGoal().getState() == SlamState.RETRACTING && slammed;
  }
}
