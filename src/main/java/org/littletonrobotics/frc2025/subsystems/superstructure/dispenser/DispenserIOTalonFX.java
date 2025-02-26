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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

public class DispenserIOTalonFX implements DispenserIO {
  public static final double reduction = (72.0 / 12.0) * (44.0 / 19.0) * (74.0 / 24.0);
  private static final Rotation2d offset = new Rotation2d();
  private static final int encoderId = 45;

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration Config = new TalonFXConfiguration();

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

  public DispenserIOTalonFX() {
    talon = new TalonFX(5);
    encoder = new CANcoder(encoderId);

    // Configure  motor
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    //    Config.Feedback.RotorToSensorRatio = reduction;
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //    Config.Feedback.FeedbackRemoteSensorID = encoderId;
    //    Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    Config.Feedback.SensorToMechanismRatio = reduction;
    Config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    Config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    Config.CurrentLimits.StatorCurrentLimit = 40.0;
    Config.CurrentLimits.StatorCurrentLimitEnable = true;
    Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config, 0.25));

    // Configure encoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = offset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

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
        50.0,
        internalPosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, encoderAbsolutePosition, encoderRelativePosition);
    ParentDevice.optimizeBusUtilizationForAll(talon, encoder);

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        internalPosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp,
        encoderAbsolutePosition,
        encoderRelativePosition);

    // Reset to zero (fallback until CANcoder is available)
    tryUntilOk(5, () -> talon.setPosition(0.0));
  }

  @Override
  public void updateInputs(DispenserIOInputs inputs) {
    inputs.data =
        new DispenserIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    internalPosition,
                    internalVelocity,
                    appliedVolts,
                    supplyCurrentAmps,
                    torqueCurrentAmps,
                    temp)),
            encoderConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(encoderAbsolutePosition, encoderRelativePosition)),
            Rotation2d.fromRotations(internalPosition.getValueAsDouble()),
            Rotation2d.fromRotations(encoderAbsolutePosition.getValueAsDouble()).minus(offset),
            encoderRelativePosition.getValue().in(Radians) - offset.getRadians(),
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
    Config.Slot0.kP = kP;
    Config.Slot0.kI = kI;
    Config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              Config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
            })
        .start();
  }
}
