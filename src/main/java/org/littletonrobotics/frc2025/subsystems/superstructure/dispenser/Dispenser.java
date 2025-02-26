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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Dispenser {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-55.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(20.0);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Dispenser/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Dispenser/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Dispenser/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Dispenser/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/MaxVelocityDegreesPerSec", 2000.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/MaxAccelerationDegreesPerSec2", 8000.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationRampRate", 0.2);
  private static final LoggedTunableNumber algaeIntakeVelocityThresh =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeVelocityThreshold", 1.0);
  public static final LoggedTunableNumber gripperIntakeCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeCurrent", 30.0);
  public static final LoggedTunableNumber gripperDispenseCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeDispenseCurrent", -30.0);
  public static final LoggedTunableNumber tunnelDispenseVolts =
      new LoggedTunableNumber("Dispenser/TunnelDispenseVolts", 6.0);
  public static final LoggedTunableNumber tunnelIntakeVolts =
      new LoggedTunableNumber("Dispenser/TunnelIntakeVolts", 3.0);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Dispenser/Tolerance", .1);

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

  // Hardware
  private final DispenserIO pivotIO;
  private final DispenserIOInputsAutoLogged pivotInputs = new DispenserIOInputsAutoLogged();
  private final RollerSystemIO tunnelIO;
  private final RollerSystemIOInputsAutoLogged tunnelInputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  @AutoLogOutput(key = "Dispenser/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @Getter
  @AutoLogOutput(key = "Dispenser/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private double tunnelVolts = 0.0;
  @Setter private double gripperCurrent = 0.0;

  private boolean hasCoral = false;
  private boolean hasAlgae = false;

  private Debouncer coralDebouncer = new Debouncer(0.1);
  private Debouncer algaeDebouncer = new Debouncer(0.1);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Dispenser pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Dispenser encoder disconnected!", Alert.AlertType.kWarning);
  private final Alert tunnelDisconnectedAlert =
      new Alert("Dispenser tunnel disconnected!", Alert.AlertType.kWarning);
  private final Alert gripperDisconnectedAlert =
      new Alert("Dispenser gripper disconnected!", Alert.AlertType.kWarning);

  public Dispenser(DispenserIO pivotIO, RollerSystemIO tunnelIO, RollerSystemIO gripperIO) {
    this.pivotIO = pivotIO;
    this.tunnelIO = tunnelIO;
    this.gripperIO = gripperIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));

    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      new Trigger(() -> DriverStation.getStickButtonPressed(2, 1))
          .onTrue(Commands.runOnce(() -> hasAlgae = !hasAlgae));
    }
    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      new Trigger(() -> DriverStation.getStickButtonPressed(2, 2))
          .onTrue(Commands.runOnce(() -> hasCoral = !hasCoral));
    }
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Dispenser/Pivot", pivotInputs);
    tunnelIO.updateInputs(tunnelInputs);
    Logger.processInputs("Dispenser/Tunnel", tunnelInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Dispenser/Gripper", gripperInputs);

    pivotMotorDisconnectedAlert.set(
        !pivotInputs.data.motorConnected() && Constants.getRobot() == RobotType.COMPBOT);
    pivotEncoderDisconnectedAlert.set(
        !pivotInputs.data.encoderConnected() && Constants.getRobot() == RobotType.COMPBOT);
    tunnelDisconnectedAlert.set(!tunnelInputs.data.connected());
    gripperDisconnectedAlert.set(
        !gripperInputs.data.connected() && Constants.getRobot() == RobotType.COMPBOT);

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
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(setpoint.position),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log state
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", setpoint.position);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", goalState.position);
    } else {
      // Reset setpoint
      setpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", 0.0);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", 0.0);
    }

    // Run tunnel and gripper
    if (!isEStopped) {
      tunnelIO.runVolts(tunnelVolts);
      gripperIO.runTorqueCurrent(gripperCurrent);
    } else {
      pivotIO.stop();
      tunnelIO.stop();
      gripperIO.stop();
    }

    // Check algae
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (Math.abs(gripperInputs.data.torqueCurrentAmps()) >= 5.0) {
        hasAlgae =
            algaeDebouncer.calculate(
                Math.abs(gripperInputs.data.velocityRadsPerSec())
                    >= algaeIntakeVelocityThresh.get());
      } else {
        algaeDebouncer.calculate(hasAlgae);
      }
      if (Math.abs(tunnelInputs.data.torqueCurrentAmps()) >= 5.0) {
        hasCoral =
            coralDebouncer.calculate(Math.abs(tunnelInputs.data.velocityRadsPerSec()) <= 3.0);
      } else {
        coralDebouncer.calculate(hasCoral);
      }
    }

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

  public Rotation2d getPivotAngle() {
    return pivotInputs.data.internalPosition();
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

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
