// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private static final LoggedTunableNumber deployVolts =
      new LoggedTunableNumber("Climber/DeployVolts", -8);
  private static final LoggedTunableNumber deployAngle =
      new LoggedTunableNumber("Climber/DeployAngle", -77);
  private static final LoggedTunableNumber climbCurrent =
      new LoggedTunableNumber("Climber/ClimbCurrent", 100);
  private static final LoggedTunableNumber climbCurrentRampRate =
      new LoggedTunableNumber("Climber/ClimbCurrentRampRate", 120);
  private static final LoggedTunableNumber climbStopAngle =
      new LoggedTunableNumber("Climber/ClimbStopAngle", 35);
  private static final LoggedTunableNumber autoClimbCurrent =
      new LoggedTunableNumber("Climber/AutoClimbCurrent", 33);
  private static final LoggedTunableNumber gripVolts =
      new LoggedTunableNumber("Climber/GripVolts", 12.0);

  private static final Translation3d climberOrigin3d =
      new Translation3d(-0.0508005, 0.342900, 0.32);

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();

  @AutoLogOutput private double climbStopOffsetDegrees = 0.0;
  @Getter @AutoLogOutput private ClimbState climbState = ClimbState.START;

  private final Timer climbTimer = new Timer();
  private boolean stopPull = false;
  private final Debouncer autoClimbDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  @Setter private BooleanSupplier coastOverride = () -> false;

  private final Alert climberDisconnected =
      new Alert("Climber motor disconnected!", Alert.AlertType.kWarning);
  private final Alert climberGripperDisconnected =
      new Alert("Climber gripper motor disconnected!", Alert.AlertType.kWarning);
  private final Alert climberGripperTempFault =
      new Alert("Climber gripper motor too hot! 🥵", Alert.AlertType.kWarning);

  public Climber(ClimberIO climberIO, RollerSystemIO gripperIO) {
    this.climberIO = climberIO;
    this.gripperIO = gripperIO;
  }

  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber/Climber", climberInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Climber/Gripper", gripperInputs);

    climberDisconnected.set(
        !motorConnectedDebouncer.calculate(climberInputs.data.motorConnected())
            && !Robot.isJITing());
    climberGripperDisconnected.set(!gripperInputs.data.connected() && !Robot.isJITing());
    climberGripperTempFault.set(gripperInputs.data.tempFault());

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      climberIO.runVolts(0.0);
      climbState = ClimbState.START;

      if (coastOverride.getAsBoolean()) {
        climberIO.coast();
      }
    }

    Logger.recordOutput("Climber/GripperReady", false);

    // Handle state
    if (climbState != ClimbState.PULL) {
      stopPull = false;
      Leds.getInstance().superClimbed = false;
      climbTimer.restart();
    }
    if (climbState != ClimbState.READY) {
      autoClimbDebouncer.calculate(false);
    }
    switch (climbState) {
      case START -> {
        Leds.getInstance().ready = false;
        if (DriverStation.isEnabled()) {
          Leds.getInstance().superClimbed = false;
        }
      }
      case READY -> {
        Leds.getInstance().superClimbed = false;
        double position = climberInputs.data.positionRads();
        boolean gripperReady = false;
        if (position >= Units.degreesToRadians(deployAngle.get())) {
          climberIO.runVolts(deployVolts.get());
        } else {
          gripperReady = true;
          climberIO.stop();
        }

        if (gripperReady) {
          gripperIO.runVolts(gripVolts.get());
          Leds.getInstance().ready = true;
          if (autoClimbDebouncer.calculate(
              gripperInputs.data.supplyCurrentAmps() > autoClimbCurrent.get())) {
            climbState = ClimbState.PULL;
          }
        } else {
          Leds.getInstance().ready = false;
          gripperIO.stop();
        }
        Logger.recordOutput("Climber/GripperReady", gripperReady);
      }
      case PULL -> {
        gripperIO.stop();
        if (climberInputs.data.positionRads()
            >= Units.degreesToRadians(climbStopAngle.get() + climbStopOffsetDegrees)) {
          stopPull = true;
          Leds.getInstance().superClimbed = true;
        }
        climberIO.runTorqueCurrent(
            stopPull
                ? 0.0
                : Math.min(climbCurrentRampRate.get() * climbTimer.get(), climbCurrent.get()));
      }
    }

    // Visualize climber position
    Logger.recordOutput(
        "Mechanism3d/Measured/Climber",
        new Pose3d(
            climberOrigin3d,
            new Rotation3d(
                climberInputs.data.positionRads() - Units.degreesToRadians(35.0), 0.0, 0.0)));

    // Record cycle time
    LoggedTracer.record("Climber");
  }

  public Command readyClimb() {
    return runOnce(
        () -> {
          if (climbState == ClimbState.READY) {
            climbState = ClimbState.PULL;
          } else {
            climbState = ClimbState.READY;
          }
        });
  }

  public void adjustClimbOffset(double offset) {
    climbStopOffsetDegrees += offset;
  }

  public enum ClimbState {
    START,
    READY,
    PULL
  }
}
