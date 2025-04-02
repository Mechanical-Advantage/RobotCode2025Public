// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util.gslam;

import static edu.wpi.first.units.Units.*;
import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

public class GenericSlamElevatorIOKrakenFOC implements GenericSlamElevatorIO {
  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Control
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  public GenericSlamElevatorIOKrakenFOC(
      int id, String bus, int currentLimitAmps, boolean invert, double reduction) {
    talon = new TalonFX(id, bus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    talon.optimizeBusUtilization(0, 1.0);

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        new CANBus(bus).isNetworkFD(),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temp);
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    inputs.data =
        new GenericSlamElevatorIOData(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp),
            position.getValue().in(Radians),
            velocity.getValue().in(RadiansPerSecond),
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runCurrent(double amps) {
    talon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
