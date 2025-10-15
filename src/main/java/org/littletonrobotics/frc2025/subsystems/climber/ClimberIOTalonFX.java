// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
  public static final double reduction = (74.0 / 16.0) / (72.0 / 18.0) * 125.0;

  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final VoltageOut voltsRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final CoastOut coastRequest = new CoastOut();

  public ClimberIOTalonFX() {
    talon = new TalonFX(1);

    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = reduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
    talon.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.data =
        new ClimberIOData(
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrentAmps, temp),
            position.getValue().in(Radians),
            velocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValueAsDouble(),
            torqueCurrentAmps.getValueAsDouble(),
            supplyCurrentAmps.getValueAsDouble(),
            temp.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltsRequest.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double current) {
    talon.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void coast() {
    talon.setControl(coastRequest);
  }
}
