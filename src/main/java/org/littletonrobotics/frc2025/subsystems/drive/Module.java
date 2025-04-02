// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Robot;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber drivekT =
      new LoggedTunableNumber("Drive/Module/DrivekT");
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        drivekS.initDefault(5.0);
        drivekV.initDefault(0);
        // Multiplied by desired wheelTorqueNm
        drivekT.initDefault(ModuleIOComp.driveReduction / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
        drivekP.initDefault(35.0);
        drivekD.initDefault(0);
        turnkP.initDefault(4000.0);
        turnkD.initDefault(50.0);
      }
      default -> {
        drivekS.initDefault(0.014);
        drivekV.initDefault(0.134);
        drivekT.initDefault(0);
        drivekP.initDefault(0.1);
        drivekD.initDefault(0);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0);
      }
    }
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SimpleMotorFeedforward ffModel;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void periodic() {
    // Update tunable numbers
    if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
    }
    if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
      io.setDrivePID(drivekP.get(), 0, drivekD.get());
    }
    if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
      io.setTurnPID(turnkP.get(), 0, turnkD.get());
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryDrivePositionsRad.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.data.driveConnected() && !Robot.isJITing());
    turnDisconnectedAlert.set(!inputs.data.turnConnected() && !Robot.isJITing());
    turnEncoderDisconnectedAlert.set(!inputs.data.turnEncoderConnected() && !Robot.isJITing());

    // Record cycle time
    LoggedTracer.record("Drive/Module" + index);
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleState state) {
    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    io.runDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec));
    io.runTurnPosition(state.angle);
  }

  /**
   * Runs the module with the specified setpoint state and a setpoint wheel force used for
   * torque-based feedforward.
   */
  public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    io.runDriveVelocity(
        speedRadPerSec, ffModel.calculate(speedRadPerSec) + wheelTorqueNm * drivekT.get());
    io.runTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.runDriveOpenLoop(output);
    io.runTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.data.turnPosition();
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.data.drivePositionRad() * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.data.driveVelocityRadPerSec() * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.data.drivePositionRad();
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.data.driveVelocityRadPerSec());
  }

  /* Sets brake mode to {@code enabled} */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
