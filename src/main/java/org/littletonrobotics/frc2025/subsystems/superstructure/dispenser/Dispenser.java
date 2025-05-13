// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIO;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Dispenser {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-87.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(13.7);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Dispenser/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Dispenser/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Dispenser/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Dispenser/kG");
  private static final LoggedTunableNumber tunnelkP =
      new LoggedTunableNumber("Dispenser/TunnelPID/kP", 2.5);
  private static final LoggedTunableNumber tunnelkD =
      new LoggedTunableNumber("Dispenser/TunnelPID/kD", 0.0);
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/MaxVelocityDegreesPerSec", 1500.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/MaxAccelerationDegreesPerSec2", 2500.0);
  private static final LoggedTunableNumber algaeMaxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/AlgaeMaxVelocityDegreesPerSec", 800.0);
  private static final LoggedTunableNumber algaeMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/AlgaeMaxAccelerationDegreesPerSec2", 1500.0);
  private static final LoggedTunableNumber slowMaxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/SlowMaxVelocityDegreesPerSec", 800.0);
  private static final LoggedTunableNumber slowMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/SlowMaxAccelerationDegreesPerSec2", 1500.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationRampRate", 0.2);
  private static final LoggedTunableNumber algaeCurrentThresh =
      new LoggedTunableNumber("Dispenser/AlgaeCurrentThreshold", 20.0);
  private static final LoggedTunableNumber coralProxThreshold =
      new LoggedTunableNumber("Dispenser/CoralProxThresh", 0.15);
  public static final LoggedTunableNumber gripperHoldVolts =
      new LoggedTunableNumber("Dispenser/GripperHoldVolts", 1.0);
  public static final LoggedTunableNumber gripperIntakeVolts =
      new LoggedTunableNumber("Dispenser/GripperIntakeVolts", 9.0);
  public static final LoggedTunableNumber gripperEjectVolts =
      new LoggedTunableNumber("Dispenser/GripperEjectVolts", -12.0);
  public static final LoggedTunableNumber gripperIdleReverseTime =
      new LoggedTunableNumber("Dispenser/GripperIdleReverseTime", 0.5);
  public static final LoggedTunableNumber gripperHardstopVolts =
      new LoggedTunableNumber("Dispenser/GripperHardstopVolts", -12.0);
  public static final LoggedTunableNumber gripperReverseHardstopVolts =
      new LoggedTunableNumber("Dispenser/GripperReverseHardstopVolts", 12.0);
  public static final LoggedTunableNumber gripperCurrentLimit =
      new LoggedTunableNumber("Dispenser/GripperCurrentLimit", 50.0);
  public static final LoggedTunableNumber[] tunnelDispenseVolts = {
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L1", 1.5),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L2", 2.6),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L3", 2.6),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L4", 5.0)
  };
  public static final LoggedTunableNumber[] tunnelDispenseVoltsAlgae = {
    new LoggedTunableNumber("Dispenser/TunnelDispenseVoltsAlgae/L2", 3.0),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVoltsAlgae/L3", 3.0),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVoltsAlgae/L4", 5.0)
  };
  public static final LoggedTunableNumber tunnelIntakeVolts =
      new LoggedTunableNumber("Dispenser/TunnelIntakeVolts", 1.5);
  public static final LoggedTunableNumber tunnelL1ReverseVolts =
      new LoggedTunableNumber("Dispenser/TunnelL1ReverseVolts", -3.0);
  public static final LoggedTunableNumber tunnelIndexReverseVolts =
      new LoggedTunableNumber("Dispenser/TunnelIndexReverseVolts", -2.0);
  public static final LoggedTunableNumber tunnelPreIndexTimeout =
      new LoggedTunableNumber("Dispenser/TunnelPreIndexTimeout", 0.4);
  public static final LoggedTunableNumber tunnelIndexTimeout =
      new LoggedTunableNumber("Dispenser/TunnelIndexTimeout", 0.2);
  public static final LoggedTunableNumber tunnelPositionMaxVolts =
      new LoggedTunableNumber("Dispenser/TunnelPositionMaxVolts", 1.0);
  public static final LoggedTunableNumber tunnelPreIndexOffsetRads =
      new LoggedTunableNumber("Dispenser/TunnelPreIndexOffsetRads", 2.5);
  public static final LoggedTunableNumber tunnelHardstopIndexOffsetRads =
      new LoggedTunableNumber("Dispenser/TunnelHardstopIndexOffsetRads", -0.7);
  public static final LoggedTunableNumber tunnelIndexOffsetRads =
      new LoggedTunableNumber("Dispenser/TunnelIndexOffsetRads", 1.3);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Dispenser/Tolerance", 0.4);
  public static final LoggedTunableNumber simIntakingTime =
      new LoggedTunableNumber("Dispenser/SimIntakingTime", 0.5);
  private static final LoggedTunableNumber coralEjectDebounceTime =
      new LoggedTunableNumber("Dispenser/CoralEjectDebounceTime", 0.2);
  private static final LoggedTunableNumber coralHardstopTime =
      new LoggedTunableNumber("Dispenser/CoralHardstopTime", 0.3);
  private static final LoggedTunableNumber algaeDebounceTime =
      new LoggedTunableNumber("Dispenser/AlgaeDebounceTime", 0.6);
  public static final LoggedTunableNumber gripperHardstopTime =
      new LoggedTunableNumber("Dispenser/GripperHardstopTime", 0.5);

  static {
    switch (Constants.getRobot()) {
      case SIMBOT -> {
        kP.initDefault(4000);
        kD.initDefault(1000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
      default -> {
        kP.initDefault(6500);
        kD.initDefault(100);
        kS.initDefault(0);
        kG.initDefault(0);
      }
    }
  }

  public enum GripperGoal {
    IDLE,
    GRIP,
    EJECT,
    HARDSTOP,
    REVERSE_HARDSTOP
  }

  // Hardware
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final RollerSystemIO tunnelIO;
  private final PIDController tunnelPositionController;
  private final RollerSystemIOInputsAutoLogged tunnelInputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput(key = "Dispenser/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  private TrapezoidProfile algaeProfile;
  private TrapezoidProfile slowProfile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @AutoLogOutput private double appliedGripperVolts = 0.0;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;
  @Setter private boolean isIntaking = false;
  @Setter private boolean forceFastConstraints = false;
  @Setter private boolean forceEjectForward = false;
  @Setter private boolean coralIndexed = false;

  @Accessors(fluent = true)
  @Setter
  private boolean forceSlowConstraints = false;

  @Getter
  @AutoLogOutput(key = "Dispenser/Profile/AtGoal")
  private boolean atGoal = false;

  @AutoLogOutput @Setter private double tunnelVolts = 0.0;
  @AutoLogOutput private GripperGoal gripperGoal = GripperGoal.IDLE;
  private Timer gripperGoalTimer = new Timer();

  @AutoLogOutput
  @Accessors(fluent = true)
  @Getter
  private boolean hasCoral = false;

  @AutoLogOutput
  @Accessors(fluent = true)
  @Getter
  private boolean rawHasCoral = false;

  private boolean lastHasCoral = hasCoral;

  @AutoLogOutput
  @Accessors(fluent = true)
  @Getter()
  private boolean hasAlgae = false;

  @Getter private boolean doNotStopIntaking = false;

  private Debouncer coralEjectedDebouncer =
      new Debouncer(coralEjectDebounceTime.get(), DebounceType.kRising);
  private Debouncer coralHardstopDebouncer = new Debouncer(coralHardstopTime.get());
  private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime.get(), DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Setter @Getter @AutoLogOutput private double coralThresholdOffset = 0.0;

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Dispenser pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Dispenser pivot encoder disconnected!", Alert.AlertType.kWarning);
  private final Alert tunnelDisconnectedAlert =
      new Alert("Dispenser tunnel disconnected!", Alert.AlertType.kWarning);
  private final Alert gripperDisconnectedAlert =
      new Alert("Dispenser gripper disconnected!", Alert.AlertType.kWarning);

  private boolean lastAlgaeButtonPressed = false;
  private boolean lastCoralButtonPressed = false;

  public Dispenser(
      PivotIO pivotIO,
      RollerSystemIO tunnelIO,
      RollerSystemIO gripperIO,
      CoralSensorIO coralSensorIO) {
    this.pivotIO = pivotIO;
    this.tunnelIO = tunnelIO;
    this.gripperIO = gripperIO;
    this.coralSensorIO = coralSensorIO;
    tunnelPositionController = new PIDController(tunnelkP.get(), 0.0, tunnelkD.get());

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    gripperGoalTimer.start();
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Dispenser/Pivot", pivotInputs);
    tunnelIO.updateInputs(tunnelInputs);
    Logger.processInputs("Dispenser/Tunnel", tunnelInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Dispenser/Gripper", gripperInputs);
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Dispenser/CoralSensor", coralSensorInputs);

    pivotMotorDisconnectedAlert.set(
        !pivotInputs.data.motorConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    pivotEncoderDisconnectedAlert.set(
        !pivotInputs.data.encoderConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    tunnelDisconnectedAlert.set(!tunnelInputs.data.connected() && !Robot.isJITing());
    gripperDisconnectedAlert.set(
        !gripperInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivotIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (tunnelkP.hasChanged(hashCode()) || tunnelkD.hasChanged(hashCode())) {
      tunnelPositionController.setPID(tunnelkP.get(), 0.0, tunnelkD.get());
    }
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }
    if (algaeMaxVelocityDegPerSec.hasChanged(hashCode())
        || algaeMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      algaeProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(algaeMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(algaeMaxAccelerationDegPerSec2.get())));
    }
    if (slowMaxVelocityDegPerSec.hasChanged(hashCode())
        || slowMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      slowProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(slowMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(slowMaxAccelerationDegPerSec2.get())));
    }
    if (gripperCurrentLimit.hasChanged(hashCode())) {
      gripperIO.setCurrentLimit(gripperCurrentLimit.get());
    }
    if (coralEjectDebounceTime.hasChanged(hashCode())) {
      coralEjectedDebouncer.setDebounceTime(coralEjectDebounceTime.get());
    }
    if (coralHardstopTime.hasChanged(hashCode())) {
      coralHardstopDebouncer.setDebounceTime(coralHardstopTime.get());
    }
    if (algaeDebounceTime.hasChanged(hashCode())) {
      algaeDebouncer.setDebounceTime(algaeDebounceTime.get());
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Dispenser/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(pivotInputs.data.internalPosition().getRadians() - setpoint.position)
            > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.getAsDouble(), minAngle.getRadians(), maxAngle.getRadians()),
              0.0);
      setpoint =
          (forceSlowConstraints
                  ? slowProfile
                  : hasAlgae && !forceFastConstraints ? algaeProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(setpoint.position),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * pivotInputs.data.internalPosition().getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log state
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRad", setpoint.position);
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRadPerSec", setpoint.velocity);
      Logger.recordOutput("Dispenser/Profile/GoalAngleRad", goalState.position);
    } else {
      // Reset setpoint
      setpoint = new State(pivotInputs.data.internalPosition().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRad", 0.0);
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRadPerSec", 0.0);
      Logger.recordOutput("Dispenser/Profile/GoalAngleRad", 0.0);
    }
    if (isEStopped) {
      pivotIO.stop();
    }

    // Check algae & coral states
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (Math.abs(appliedGripperVolts) >= 0.5 || DriverStation.isDisabled()) {
        hasAlgae =
            algaeDebouncer.calculate(
                gripperInputs.data.torqueCurrentAmps() >= algaeCurrentThresh.get());
      } else {
        algaeDebouncer.calculate(hasAlgae);
      }
      if (isIntaking) {
        if (tunnelVolts > 0.0) {
          rawHasCoral =
              coralSensorInputs.data.valid()
                  && coralSensorInputs.data.distanceMeters() < coralProxThreshold.get();
          if (gripperGoal == GripperGoal.HARDSTOP) {
            hasCoral = coralHardstopDebouncer.calculate(rawHasCoral);
          } else {
            hasCoral = rawHasCoral;
          }
        } else {
          coralHardstopDebouncer.calculate(hasCoral);
        }
        coralEjectedDebouncer.calculate(false);
      } else if (coralEjectedDebouncer.calculate(Math.abs(tunnelVolts) >= 0.5)) {
        hasCoral = false;
      }
    } else if (DriverStation.isAutonomous()) {
      if (!isIntaking && tunnelInputs.data.velocityRadsPerSec() >= 50.0) {
        hasCoral = false;
      }
    } else {
      boolean algaeButtonPressed = DriverStation.getStickButtonPressed(2, 1);
      boolean coralButtonPressed = DriverStation.getStickButtonPressed(2, 2);
      if (algaeButtonPressed && !lastAlgaeButtonPressed) {
        hasAlgae = !hasAlgae;
      }
      if (coralButtonPressed && !lastCoralButtonPressed) {
        hasCoral = !hasCoral;
      }
      lastAlgaeButtonPressed = algaeButtonPressed;
      lastCoralButtonPressed = coralButtonPressed;
    }

    // Run tunnel and gripper
    if (forceEjectForward) {
      tunnelIO.runVolts(tunnelDispenseVolts[3].get());
      appliedGripperVolts = gripperEjectVolts.get();
      gripperIO.runVolts(appliedGripperVolts);
    } else if (!isEStopped) {
      double intakeVolts = tunnelVolts;
      if (hasCoral) {
        if (!lastHasCoral) {
          tunnelPositionController.setSetpoint(
              tunnelInputs.data.positionRads()
                  + (gripperGoal == GripperGoal.HARDSTOP
                      ? tunnelHardstopIndexOffsetRads.get()
                      : tunnelPreIndexOffsetRads.get()));
        }
        if ((isIntaking && tunnelVolts >= 0.0) || EqualsUtil.epsilonEquals(tunnelVolts, 0.0)) {
          intakeVolts =
              MathUtil.clamp(
                  tunnelPositionController.calculate(tunnelInputs.data.positionRads()),
                  -tunnelPositionMaxVolts.get(),
                  tunnelPositionMaxVolts.get());
        }
      }
      tunnelIO.runVolts(intakeVolts);

      switch (gripperGoal) {
        case IDLE ->
            appliedGripperVolts =
                gripperGoalTimer.hasElapsed(gripperIdleReverseTime.get())
                    ? 0.0
                    : gripperReverseHardstopVolts.get();
        case GRIP -> {
          if (hasAlgae) {
            appliedGripperVolts = gripperHoldVolts.get();
          } else {
            appliedGripperVolts = gripperIntakeVolts.get();
          }
        }
        case EJECT -> appliedGripperVolts = gripperEjectVolts.get();
        case HARDSTOP ->
            appliedGripperVolts =
                gripperGoalTimer.hasElapsed(gripperHardstopTime.get())
                    ? 0.0
                    : gripperHardstopVolts.get();
        case REVERSE_HARDSTOP -> appliedGripperVolts = gripperReverseHardstopVolts.get();
      }
      gripperIO.runVolts(appliedGripperVolts);
    } else {
      tunnelIO.stop();
      gripperIO.stop();
      appliedGripperVolts = 0.0;
    }
    lastHasCoral = hasCoral;

    // Display hasCoral & hasAlgae
    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Has Algae?", hasAlgae);

    // Display coral threshold offset
    SmartDashboard.putString("Coral Threshold Offset", String.format("%.1f", coralThresholdOffset));

    // Log state
    Logger.recordOutput("Dispenser/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Dispenser/DisabledOverride", disabledOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Dispenser");
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    this.goal =
        () -> MathUtil.inputModulus(goal.get().getRadians(), -3.0 * Math.PI / 2.0, Math.PI / 2.0);
    atGoal = false;
  }

  public void setGripperGoal(GripperGoal goal) {
    if (goal == gripperGoal) return;
    gripperGoal = goal;
    gripperGoalTimer.restart();
  }

  public double getGoal() {
    return goal.getAsDouble();
  }

  @AutoLogOutput(key = "Dispenser/MeasuredAngle")
  public Rotation2d getPivotAngle() {
    return pivotInputs.data.internalPosition();
  }

  public void triggerReindex() {
    tunnelPositionController.setSetpoint(
        tunnelInputs.data.positionRads() + tunnelIndexOffsetRads.get());
  }

  public void resetHasCoral(boolean value) {
    hasCoral = value;
    lastHasCoral = value;
    tunnelPositionController.setSetpoint(tunnelInputs.data.positionRads());
  }

  public void resetHasAlgae(boolean value) {
    hasAlgae = value;
    algaeDebouncer = new Debouncer(algaeDebounceTime.get(), DebounceType.kRising);
    algaeDebouncer.calculate(value);
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    pivotIO.setBrakeMode(enabled);
  }

  public Command staticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = staticCharacterizationRampRate.get() * timer.get();
              pivotIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Dispenser/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () ->
                pivotInputs.data.velocityRadPerSec() >= staticCharacterizationVelocityThresh.get())
        .andThen(pivotIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Dispenser/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
