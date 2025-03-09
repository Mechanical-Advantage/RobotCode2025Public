// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import edu.wpi.first.math.MathUtil;
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
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Dispenser {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(20.5);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Dispenser/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Dispenser/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Dispenser/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Dispenser/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/MaxVelocityDegreesPerSec", 1500.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/MaxAccelerationDegreesPerSec2", 2500.0);
  private static final LoggedTunableNumber algaeMaxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/AlgaeMaxVelocityDegreesPerSec", 300.0);
  private static final LoggedTunableNumber algaeMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/AlgaeMaxAccelerationDegreesPerSec2", 400.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationRampRate", 0.2);
  private static final LoggedTunableNumber algaeVelocityThresh =
      new LoggedTunableNumber("Dispenser/AlgaeVelocityThreshold", 45.0);
  private static final LoggedTunableNumber coralProxThreshold =
      new LoggedTunableNumber("Dispenser/CoralProxThresh", 0.06);
  public static final LoggedTunableNumber gripperHoldVolts =
      new LoggedTunableNumber("Dispenser/GripperHoldVolts", 5.0);
  public static final LoggedTunableNumber gripperIntakeVolts =
      new LoggedTunableNumber("Dispenser/GripperIntakeVolts", 5.0);
  public static final LoggedTunableNumber gripperEjectVolts =
      new LoggedTunableNumber("Dispenser/GripperEjectVolts", -12.0);
  public static final LoggedTunableNumber gripperL1EjectVolts =
      new LoggedTunableNumber("Dispenser/GripperL1EjectVolts", -5.0);
  public static final LoggedTunableNumber gripperCurrentLimit =
      new LoggedTunableNumber("Dispenser/GripperCurrentLimit", 50.0);
  public static final LoggedTunableNumber tunnelDispenseVolts =
      new LoggedTunableNumber("Dispenser/TunnelDispenseVolts", 6.0);
  public static final LoggedTunableNumber tunnelL1DispenseVolts =
      new LoggedTunableNumber("Dispenser/TunnelL1DispenseVolts", 3.5);
  public static final LoggedTunableNumber tunnelIntakeVolts =
      new LoggedTunableNumber("Dispenser/TunnelIntakeVolts", 3.0);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Dispenser/Tolerance", 0.4);
  public static final LoggedTunableNumber intakeReverseVolts =
      new LoggedTunableNumber("Dispenser/IntakeReverseVolts", -1.8);
  public static final LoggedTunableNumber intakeReverseTime =
      new LoggedTunableNumber("Dispenser/IntakeReverseTime", 0.1);
  public static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Dispenser/HomingTimeSecs", 0.2);
  public static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Dispenser/HomingVolts", 3.0);
  public static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Dispenser/HomingVelocityThreshold", 0.1);

  static {
    switch (Constants.getRobot()) {
      case SIMBOT -> {
        kP.initDefault(4000);
        kD.initDefault(1000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
      default -> {
        kP.initDefault(12000.0);
        kD.initDefault(120.0);
        kS.initDefault(0);
        kG.initDefault(0);
      }
    }
  }

  public enum GripperGoal {
    IDLE,
    GRIP,
    EJECT,
    L1_EJECT
  }

  // Hardware
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final RollerSystemIO tunnelIO;
  private final RollerSystemIOInputsAutoLogged tunnelInputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  @AutoLogOutput(key = "Dispenser/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  private TrapezoidProfile algaeProfile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;
  @Setter private boolean isIntaking = false;
  private final Timer intakingReverseTimer = new Timer();

  @Getter
  @AutoLogOutput(key = "Dispenser/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private double tunnelVolts = 0.0;
  @AutoLogOutput @Setter private GripperGoal gripperGoal = GripperGoal.IDLE;

  @Setter private boolean hasCoral = false;
  private boolean hasAlgae = false;
  @Getter private boolean doNotStopIntaking = false;

  private static final double coralDebounceTime = 0.1;
  private static final double algaeDebounceTime = 0.4;
  private Debouncer coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
  private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private Debouncer homingDebouncer = new Debouncer(0.25);

  @Setter @Getter @AutoLogOutput private double coralThresholdOffset = 0.0;
  @AutoLogOutput private Rotation2d homingOffset = Rotation2d.kZero;

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Dispenser pivot motor disconnected!", Alert.AlertType.kWarning);
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

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    intakingReverseTimer.start();
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
    tunnelDisconnectedAlert.set(!tunnelInputs.data.connected() && !Robot.isJITing());
    gripperDisconnectedAlert.set(
        !gripperInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivotIO.setPID(kP.get(), 0.0, kD.get());
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
    if (gripperCurrentLimit.hasChanged(hashCode())) {
      gripperIO.setCurrentLimit(gripperCurrentLimit.get());
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
        Math.abs(getPivotAngle().getRadians() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.getAsDouble(), minAngle.getRadians(), maxAngle.getRadians()),
              0.0);
      setpoint =
          (hasAlgae() ? algaeProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(
              setpoint.position - maxAngle.getRadians() + homingOffset.getRadians()),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
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
      setpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRad", 0.0);
      Logger.recordOutput("Dispenser/Profile/SetpointAngleRadPerSec", 0.0);
      Logger.recordOutput("Dispenser/Profile/GoalAngleRad", 0.0);
    }

    // Run tunnel and gripper
    Leds.getInstance().coralGrabbed = false;
    if (!isEStopped) {
      double intakeVolts = tunnelVolts;
      if (isIntaking && !hasCoral) {
        intakingReverseTimer.restart();
      } else if (intakingReverseTimer.get() < intakeReverseTime.get()) {
        intakeVolts = intakeReverseVolts.get();
      } else if (isIntaking) {
        Leds.getInstance().coralGrabbed = true;
        intakeVolts = 0.0;
      }
      tunnelIO.runVolts(intakeVolts);

      switch (gripperGoal) {
        case IDLE -> gripperIO.stop();
        case GRIP -> {
          if (hasAlgae) {
            gripperIO.runVolts(gripperHoldVolts.get());
          } else {
            gripperIO.runVolts(gripperIntakeVolts.get());
          }
        }
        case EJECT -> gripperIO.runVolts(gripperEjectVolts.get());
        case L1_EJECT -> gripperIO.runVolts(gripperL1EjectVolts.get());
      }
    } else {
      pivotIO.stop();
      tunnelIO.stop();
      gripperIO.stop();
    }

    // Check algae & coral states
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (Math.abs(gripperInputs.data.torqueCurrentAmps()) >= 5.0) {
        hasAlgae =
            algaeDebouncer.calculate(
                Math.abs(gripperInputs.data.velocityRadsPerSec()) <= algaeVelocityThresh.get());
      } else {
        algaeDebouncer.calculate(hasAlgae);
      }
      if (tunnelInputs.data.torqueCurrentAmps() >= 5.0
          && tunnelInputs.data.velocityRadsPerSec() > 0.0) {
        hasCoral =
            (Constants.getRobot() != RobotType.DEVBOT)
                ? coralDebouncer.calculate(
                    coralSensorInputs.data.valid()
                        && coralSensorInputs.data.distanceMeters() < coralProxThreshold.get())
                : coralDebouncer.calculate(pivotInputs.data.velocityRadPerSec() < 1.0);
      } else {
        coralDebouncer.calculate(hasCoral);
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

    // Display hasCoral & hasAlgae
    SmartDashboard.putBoolean("Has Coral?", hasCoral());
    SmartDashboard.putBoolean("Has Algae?", hasAlgae());

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

  public double getGoal() {
    return goal.getAsDouble();
  }

  @AutoLogOutput
  public boolean hasAlgae() {
    return hasAlgae && !disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return hasCoral || disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput(key = "Dispenser/MeasuredAngle")
  public Rotation2d getPivotAngle() {
    return pivotInputs.data.position().plus(maxAngle).minus(homingOffset);
  }

  public void resetHasCoral() {
    hasCoral = false;
    coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
    coralDebouncer.calculate(false);
  }

  public void resetHasAlgae() {
    hasAlgae = false;
    algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
    algaeDebouncer.calculate(false);
  }

  public void setOverrides(
      BooleanSupplier coastOverride,
      BooleanSupplier disabledOverride,
      BooleanSupplier disableGamePieceDetectionOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
    this.disableGamePieceDetectionOverride = disableGamePieceDetectionOverride;
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

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homingDebouncer = new Debouncer(homingTimeSecs.get(), DebounceType.kRising);
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              pivotIO.runVolts(homingVolts.get());
            })
        .raceWith(
            Commands.runOnce(() -> {})
                .andThen(
                    Commands.waitUntil(
                        () ->
                            homingDebouncer.calculate(
                                Math.abs(pivotInputs.data.velocityRadPerSec())
                                    <= homingVelocityThresh.get()))))
        .andThen(
            () -> {
              homingOffset = pivotInputs.data.position();
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
