// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.rollers;

import static org.littletonrobotics.frc2025.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class RollerSystemIOSpark implements RollerSystemIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;
  private final SparkBaseConfig config;

  private final Debouncer connectedDebouncer = new Debouncer(.5);
  private int currentLimit = 60;
  private boolean brakeModeEnabled = true;

  public RollerSystemIOSpark(int deviceId, boolean isFlex) {
    spark =
        isFlex
            ? new SparkFlex(deviceId, SparkLowLevel.MotorType.kBrushless)
            : new SparkMax(deviceId, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();

    config = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
    config
        .idleMode(
            brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(currentLimit, 50)
        .voltageCompensation(12.0);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    sparkStickyFault = false;
    inputs.data =
        new RollerSystemIOData(
            ifOkOrDefault(spark, encoder::getPosition, inputs.data.positionRads()),
            ifOkOrDefault(spark, encoder::getVelocity, inputs.data.velocityRadsPerSec()),
            ifOkOrDefault(
                spark,
                new DoubleSupplier[] {spark::getBusVoltage, spark::getAppliedOutput},
                x -> x[0] * x[1],
                inputs.data.appliedVoltage()),
            0.0,
            ifOkOrDefault(spark, spark::getOutputCurrent, inputs.data.torqueCurrentAmps()),
            ifOkOrDefault(spark, spark::getMotorTemperature, inputs.data.tempCelsius()),
            false,
            connectedDebouncer.calculate(!sparkStickyFault));
  }

  @Override
  public void runVolts(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    new Thread(
            () ->
                tryUntilOk(
                    spark,
                    5,
                    () ->
                        spark.configure(
                            config.idleMode(
                                brakeModeEnabled
                                    ? SparkBaseConfig.IdleMode.kBrake
                                    : SparkBaseConfig.IdleMode.kCoast),
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters)))
        .start();
  }
}
