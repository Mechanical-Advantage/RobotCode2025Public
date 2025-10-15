// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Slam {
  protected static final double maxAngle = Units.degreesToRadians(140.0);
  private static final Translation3d intakeOrigin3d = new Translation3d(-0.285750, 0.0, 0.285750);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Slam/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Slam/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Slam/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Slam/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Slam/kA");
  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Slam/MaxVelocityMetersPerSec", 8.0);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Slam/MaxAccelerationMetersPerSec2", 80.0);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Slam/HomingVolts", -3.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Slam/HomingTimeSecs", 0.4);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Slam/HomingVelocityThresh", 0.1);

  static {
    switch (Constants.getRobot()) {
      default -> {
        kP.initDefault(1800);
        kD.initDefault(100);
        kS.initDefault(0);
        kG.initDefault(20);
        kA.initDefault(0);
      }
      case SIMBOT -> {
        kP.initDefault(1000.0);
        kD.initDefault(0);
        kS.initDefault(0);
        kG.initDefault(0);
        kA.initDefault(0);
      }
    }
  }

  @RequiredArgsConstructor
  public enum Goal {
    DEPLOY(new LoggedTunableNumber("Slam/DeployedDegrees", 5.0)),
    RETRACT(new LoggedTunableNumber("Slam/RetractedDegrees", 100.0)),
    REVERSE(new LoggedTunableNumber("Slam/ReverseDegrees", 18)),
    L1(new LoggedTunableNumber("Slam/L1Degrees", 90)),
    L1_EJECT(new LoggedTunableNumber("Slam/L1EjectDegrees", 90.0)),
    CLIMB(new LoggedTunableNumber("Slam/ClimbDegrees", 70.0));

    private final DoubleSupplier positionDeg;

    double getAngleRad() {
      return Units.degreesToRadians(positionDeg.getAsDouble());
    }
  }

  private final SlamIO io;
  private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Slam motor disconnected!", Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Intake/Slam/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  @Getter private Goal goal = Goal.RETRACT;
  private boolean stopProfile = false;

  @AutoLogOutput(key = "Intake/Slam/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput(key = "Intake/Slam/Homed")
  @Getter
  private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());
  private final Command homingCommand;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Intake/Slam/Profile/AtGoal")
  private boolean atGoal = false;

  @Getter
  @Accessors(fluent = true)
  private boolean wantsToDeploy = false;

  public Slam(SlamIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));

    homingCommand = homingSequence();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Slam", inputs);

    motorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected()) && !Robot.isJITing());

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    }

    // Tell intake to deploy or not when disabled
    wantsToDeploy = !homed || (getMeasuredAngleRad() < maxAngle / 2.0);

    // Home on enable
    if (DriverStation.isEnabled() && !homed && !homingCommand.isScheduled()) {
      homingCommand.schedule();
    }

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && brakeModeEnabled
            && (homed || Constants.getRobot() == Constants.RobotType.SIMBOT)
            && DriverStation.isEnabled();
    Logger.recordOutput("Intake/Slam/RunningProfile", shouldRunProfile);

    if (shouldRunProfile) {
      // Clamp goal
      var goalState = new State(MathUtil.clamp(goal.getAngleRad(), 0.0, maxAngle), 0.0);
      double previousVelocity = setpoint.velocity;
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0 || setpoint.position > maxAngle) {
        setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, maxAngle), 0.0);
      }

      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Run
      double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
      io.runPosition(
          setpoint.position + homedPosition,
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * Math.cos(setpoint.position)
              + kA.get() * accel);

      // Log state
      Logger.recordOutput("Intake/Slam/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Intake/Slam/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Intake/Slam/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Intake/Slam/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      // Reset setpoint
      setpoint = new State(getMeasuredAngleRad(), 0.0);

      // Clear logs
      Logger.recordOutput("Intake/Slam/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Intake/Slam/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Intake/Slam/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Intake/Slam/Profile/GoalVelocityMetersPerSec", 0.0);
    }

    // Log state
    Logger.recordOutput(
        "Intake/Slam/MeasuredVelocityMetersPerSec", inputs.data.velocityRadPerSec());

    // Visualize mechanism
    visualizeSlam("Measured", getMeasuredAngleRad());
    visualizeSlam("Setpoint", setpoint.position);
    visualizeSlam("Goal", goal.getAngleRad());

    // Record cycle time
    LoggedTracer.record("Slam");
  }

  void setGoal(Goal goal) {
    if (goal == this.goal) return;
    this.goal = goal;
    atGoal = false;
  }

  void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  /** Set current position of slam to home. */
  public void setHome() {
    homedPosition = inputs.data.positionRad();
    homed = true;
  }

  private void visualizeSlam(String key, double position) {
    Logger.recordOutput(
        "Mechanism3d/" + key + "/Intake",
        new Pose3d(intakeOrigin3d, new Rotation3d(0.0, position, 0.0)));
  }

  private Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (!brakeModeEnabled) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.data.velocityRadPerSec()) <= homingVelocityThresh.get()
                          && Math.abs(inputs.data.appliedVolts()) >= homingVolts.get() * 0.7);
            })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  void overrideHoming() {
    homed = false;
  }

  /** Get position of slam with maxAngle at home */
  @AutoLogOutput(key = "Intake/Slam/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.data.positionRad() - homedPosition;
  }
}
