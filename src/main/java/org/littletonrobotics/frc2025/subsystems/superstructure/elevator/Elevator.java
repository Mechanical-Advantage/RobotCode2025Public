// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public static final double drumRadius = Units.inchesToMeters(1.0);
  private static final LoggedTunableNumber characterizationRampRate =
      new LoggedTunableNumber("Elevator/CharacterizationRampRate", 0.5);
  private static final LoggedTunableNumber characterizationUpVelocityThresh =
      new LoggedTunableNumber("Elevator/CharacterizationUpVelocityThresh", 0.1);
  private static final LoggedTunableNumber characterizationDownStartAmps =
      new LoggedTunableNumber("Elevator/CharacterizationDownStartAmps", 0.0);
  private static final LoggedTunableNumber characterizationDownVelocityThresh =
      new LoggedTunableNumber("Elevator/CharacterizationDownVelocityThresh", -0.1);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber[] kS = {
    new LoggedTunableNumber("Elevator/kS/Stage1"),
    new LoggedTunableNumber("Elevator/kS/Stage2"),
    new LoggedTunableNumber("Elevator/kS/Stage3")
  };
  private static final LoggedTunableNumber[] kG = {
    new LoggedTunableNumber("Elevator/kG/Stage1"),
    new LoggedTunableNumber("Elevator/kG/Stage2"),
    new LoggedTunableNumber("Elevator/kG/Stage3")
  };
  private static final LoggedTunableNumber[] kA = {
    new LoggedTunableNumber("Elevator/kA/Stage1"),
    new LoggedTunableNumber("Elevator/kA/Stage2"),
    new LoggedTunableNumber("Elevator/kA/Stage3")
  };
  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 3.5);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 14.0);
  private static final LoggedTunableNumber algaeMaxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/AlgaeMaxVelocityMetersPerSec", 3.5);
  private static final LoggedTunableNumber algaeMaxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/AlgaeMaxAccelerationMetersPerSec2", 14.0);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingVelocityThresh", 5.0);
  private static final LoggedTunableNumber stowStopCheckHeight =
      new LoggedTunableNumber("Elevator/StowStopCheckHeight", .08);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", 0.5);

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        kP.initDefault(1500);
        kD.initDefault(30);
        for (int stage = 0; stage < 3; stage++) {
          kS[stage].initDefault(0);
          kG[stage].initDefault(40);
          kA[stage].initDefault(80);
        }
      }
      case SIMBOT -> {
        kP.initDefault(5000);
        kD.initDefault(2000);
        for (int stage = 0; stage < 3; stage++) {
          kS[stage].initDefault(5);
          kG[stage].initDefault(50);
          kA[stage].initDefault(0);
        }
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  private TrapezoidProfile algaeProfile;
  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;
  @Setter private boolean hasAlgae = false;
  @Setter private boolean forceFastConstraints = false;

  @AutoLogOutput(key = "Elevator/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput @Getter private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private boolean stowed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.data.motorConnected() && !Robot.isJITing());
    followerDisconnectedAlert.set(!inputs.data.followerConnected() && !Robot.isJITing());

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
    if (algaeMaxVelocityMetersPerSec.hasChanged(hashCode())
        || algaeMaxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      algaeProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  algaeMaxVelocityMetersPerSec.get(), algaeMaxAccelerationMetersPerSec2.get()));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && (homed || Constants.getRobot() == Constants.RobotType.SIMBOT)
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);
    // Check if out of tolerance
    boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.get().position, 0.0, SuperstructureConstants.elevatorMaxTravel),
              goal.get().velocity);
      double previousVelocity = setpoint.velocity;
      setpoint =
          (hasAlgae && !forceFastConstraints ? algaeProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0
          || setpoint.position > SuperstructureConstants.elevatorMaxTravel) {
        setpoint =
            new State(
                MathUtil.clamp(setpoint.position, 0.0, SuperstructureConstants.elevatorMaxTravel),
                0.0);
      }

      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Run
      if (stowed && atGoal && getPositionMeters() < stowStopCheckHeight.get()) {
        io.stop();
      } else {
        double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
        io.runPosition(
            setpoint.position / drumRadius + homedPosition,
            kS[getStage()].get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
                + kG[getStage()].get()
                + kA[getStage()].get() * accel);
      }

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }

    if (isEStopped) {
      io.stop();
    }

    // Set extension in robot state
    RobotState.getInstance()
        .setElevatorExtensionPercent(
            getPositionMeters() / SuperstructureConstants.elevatorMaxTravel);

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec", inputs.data.velocityRadPerSec() * drumRadius);

    // Record cycle time
    LoggedTracer.record("Elevator");
  }

  public void setGoal(DoubleSupplier goal) {
    setGoal(() -> new State(goal.getAsDouble(), 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    atGoal = false;
    this.goal = goal;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public Command upStaticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              stopProfile = true;
              state.characterizationOutput = characterizationRampRate.get() * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputUp", state.characterizationOutput);
            })
        .until(() -> inputs.data.velocityRadPerSec() >= characterizationUpVelocityThresh.get())
        .andThen(io::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputUp", state.characterizationOutput);
            });
  }

  public Command downStaticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput =
                  characterizationDownStartAmps.get()
                      - characterizationRampRate.get() * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputDown", state.characterizationOutput);
            })
        .until(() -> inputs.data.velocityRadPerSec() <= characterizationDownVelocityThresh.get())
        .andThen(io::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputDown", state.characterizationOutput);
            });
  }

  /** Set current position of elevator to home. */
  public void setHome() {
    homedPosition = inputs.data.positionRad();
    homed = true;
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.data.velocityRadPerSec()) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  /** Get position of elevator in meters with 0 at home */
  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.data.positionRad() - homedPosition) * drumRadius;
  }

  @AutoLogOutput(key = "Elevator/MeasuredStage")
  public int getStage() {
    if (getPositionMeters() <= SuperstructureConstants.stage1ToStage2Height) {
      return 0;
    } else if (getPositionMeters() <= SuperstructureConstants.stage2ToStage3Height) {
      return 1;
    } else {
      return 2;
    }
  }

  public double getGoalMeters() {
    return goal.get().position;
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
