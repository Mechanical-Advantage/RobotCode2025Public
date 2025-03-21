// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.rollers;

import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.frc2025.util.PhoenixUtil;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class RollerSystemIOTalonFX implements RollerSystemIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> tempFault;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC torqueCurrentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final double reduction;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public RollerSystemIOTalonFX(
      int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    talon = new TalonFX(id, bus);

    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.VelocityFilterTimeConstant = 0.1;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    tempFault = talon.getFault_DeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        new CANBus(bus).isNetworkFD(),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        tempFault);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.data =
        new RollerSystemIOData(
            Units.rotationsToRadians(position.getValueAsDouble()) / reduction,
            Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction,
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            tempCelsius.getValueAsDouble(),
            tempFault.getValue(),
            connectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    position,
                    velocity,
                    appliedVoltage,
                    supplyCurrent,
                    torqueCurrent,
                    tempCelsius,
                    tempFault)));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double amps) {
    talon.setControl(torqueCurrentOut.withOutput(amps));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setCurrentLimit(double currentLimit) {
    new Thread(
        () -> {
          config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(currentLimit));
          tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        });
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () ->
                tryUntilOk(
                    5,
                    () ->
                        talon.setNeutralMode(
                            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)))
        .start();
  }
}
