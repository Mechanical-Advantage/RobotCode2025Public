// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import static edu.wpi.first.units.Units.*;
import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
  private static final double reduction = (80.0 / 10.0) * (82.0 / 20.0);
  private static final Rotation2d encoderOffset = Rotation2d.fromRadians(2.6860003595877577);
  private static final int encoderId = 50;

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> internalPosition;
  private final StatusSignal<Angle> encoderAbsolutePosition;
  private final StatusSignal<Angle> encoderRelativePosition;
  private final StatusSignal<AngularVelocity> internalVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    talon = new TalonFX(5);
    encoder = new CANcoder(encoderId);

    // Configure encoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        MathUtil.inputModulus(
                Dispenser.maxAngle.getRotations() + Dispenser.minAngle.getRotations(), 0, 1)
            / 2.0;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = reduction;
    config.Feedback.FeedbackRemoteSensorID = encoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Get and set status signals
    internalPosition = talon.getPosition();
    encoderAbsolutePosition = encoder.getAbsolutePosition();
    encoderRelativePosition = encoder.getPosition();
    internalVelocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, internalVelocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, internalPosition, encoderAbsolutePosition, encoderRelativePosition);
    talon.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        internalPosition,
        encoderAbsolutePosition,
        encoderRelativePosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.data =
        new PivotIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    internalPosition,
                    internalVelocity,
                    appliedVolts,
                    supplyCurrentAmps,
                    torqueCurrentAmps,
                    temp)),
            encoderConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(encoderAbsolutePosition, encoderAbsolutePosition)),
            Rotation2d.fromRotations(internalPosition.getValueAsDouble()),
            Rotation2d.fromRotations(encoderAbsolutePosition.getValueAsDouble())
                .minus(encoderOffset),
            encoderRelativePosition.getValue().in(Radians),
            internalVelocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValue().in(Volts),
            supplyCurrentAmps.getValue().in(Amps),
            torqueCurrentAmps.getValue().in(Amps),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(config));
            })
        .start();
  }
}
