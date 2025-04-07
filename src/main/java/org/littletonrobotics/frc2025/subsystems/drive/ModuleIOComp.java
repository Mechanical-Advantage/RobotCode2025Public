// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants.ModuleConfig;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

public class ModuleIOComp implements ModuleIO {
  private static final double driveCurrentLimitAmps = 80;
  private static final double turnCurrentLimitAmps = 40;
  public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double turnReduction = (150.0 / 7.0);

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final Rotation2d encoderOffset;
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOComp(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveMotorId(), "*");
    turnTalon = new TalonFX(config.turnMotorId(), "*");
    encoder = new CANcoder(config.encoderChannel(), "*");
    encoderOffset = config.encoderOffset();

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    driveConfig.Feedback.SensorToMechanismRatio = driveReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveCurrentLimitAmps;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    turnConfig.Feedback.FeedbackRemoteSensorID = config.encoderChannel();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = turnReduction;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnCurrentLimitAmps;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
        config.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        config.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition().clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalon.getSupplyCurrent();
    driveTorqueCurrentAmps = driveTalon.getTorqueCurrent();

    // Create turn status signals
    turnAbsolutePosition = encoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition().clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalon.getSupplyCurrent();
    turnTorqueCurrentAmps = turnTalon.getTorqueCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, encoder));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        true,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnPosition,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps);
  }

  @Override
  public void updateInputs(ModuleIO.ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.data =
        new ModuleIOData(
            driveConnectedDebounce.calculate(
                BaseStatusSignal.isAllGood(
                    drivePosition,
                    driveVelocity,
                    driveAppliedVolts,
                    driveSupplyCurrentAmps,
                    driveTorqueCurrentAmps)),
            Units.rotationsToRadians(drivePosition.getValueAsDouble()),
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
            driveAppliedVolts.getValueAsDouble(),
            driveSupplyCurrentAmps.getValueAsDouble(),
            driveTorqueCurrentAmps.getValueAsDouble(),
            turnConnectedDebounce.calculate(
                BaseStatusSignal.isAllGood(
                    turnPosition,
                    turnVelocity,
                    turnAppliedVolts,
                    turnSupplyCurrentAmps,
                    turnTorqueCurrentAmps)),
            turnEncoderConnectedDebounce.calculate(
                BaseStatusSignal.isAllGood(turnAbsolutePosition)),
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(encoderOffset),
            Rotation2d.fromRotations(turnPosition.getValueAsDouble()),
            Units.rotationsToRadians(turnVelocity.getValueAsDouble()),
            turnAppliedVolts.getValueAsDouble(),
            turnSupplyCurrentAmps.getValueAsDouble(),
            turnTorqueCurrentAmps.getValueAsDouble());

    // Update odometry inputs
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveTalon.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforward));
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveConfig) {
            driveConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
          }
        });
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnConfig) {
            turnConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
          }
        });
  }
}
