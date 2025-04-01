// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIOSpark.h"

#include <thread>

#include "org/littletonrobotics/frc2025/util/SparkUtil.h"

RollerSystemIOSpark::RollerSystemIOSpark(int deviceId, bool isFlex)
    : spark(isFlex
                ? static_cast<rev::CANSparkBase *>(new rev::CANSparkFlex(
                      deviceId, rev::CANSparkLowLevel::MotorType::kBrushless))
                : static_cast<rev::CANSparkBase *>(new rev::CANSparkMax(
                      deviceId, rev::CANSparkLowLevel::MotorType::kBrushless))),
      encoder(spark->GetEncoder()),
      config(isFlex ? rev::SparkFlexConfig() : rev::SparkMaxConfig()) {
  config.SetIdleMode(brakeModeEnabled ? rev::SparkBaseConfig::IdleMode::kBrake
                                      : rev::SparkBaseConfig::IdleMode::kCoast);
  config.SetSmartCurrentLimit(currentLimit, 50);
  config.SetVoltageCompensation(12.0);
  config.GetEncoderConfig().SetUVWMeasurementPeriod(10);
  config.GetEncoderConfig().SetUVWAverageDepth(2);
  config.GetSignalConfig().SetPrimaryEncoderPositionAlwaysOn(true);
  config.GetSignalConfig().SetPrimaryEncoderPositionPeriodMs(20);
  config.GetSignalConfig().SetPrimaryEncoderVelocityAlwaysOn(true);
  config.GetSignalConfig().SetPrimaryEncoderVelocityPeriodMs(20);
  config.GetSignalConfig().SetAppliedOutputPeriodMs(20);
  config.GetSignalConfig().SetBusVoltagePeriodMs(20);
  config.GetSignalConfig().SetOutputCurrentPeriodMs(20);

  SparkUtil::TryUntilOk(spark, 5, [&]() {
    return spark->Configure(config,
                            rev::CANSparkBase::ResetMode::kResetSafeParameters,
                            rev::CANSparkBase::PersistMode::kPersistParameters);
  });
  SparkUtil::TryUntilOk(spark, 5, [&]() { return encoder->SetPosition(0.0); });
}

void RollerSystemIOSpark::UpdateInputs(RollerSystemIOInputs &inputs) {
  SparkUtil::sparkStickyFault = false;

  inputs.data.positionRads = SparkUtil::IfOkOrDefault(
      spark, [&]() { return encoder->GetPosition(); },
      inputs.data.positionRads);
  inputs.data.velocityRadsPerSec = SparkUtil::IfOkOrDefault(
      spark, [&]() { return encoder->GetVelocity(); },
      inputs.data.velocityRadsPerSec);

  inputs.data.appliedVoltage = SparkUtil::IfOkOrDefault(
      spark,
      [&]() { return spark->GetBusVoltage() * spark->GetAppliedOutput(); },
      inputs.data.appliedVoltage);

  inputs.data.torqueCurrentAmps = SparkUtil::IfOkOrDefault(
      spark, [&]() { return spark->GetOutputCurrent(); },
      inputs.data.torqueCurrentAmps);
  inputs.data.tempCelsius = SparkUtil::IfOkOrDefault(
      spark, [&]() { return spark->GetMotorTemperature(); },
      inputs.data.tempCelsius);
  inputs.data.tempFault = false;
  inputs.data.connected =
      connectedDebouncer.Calculate(!SparkUtil::sparkStickyFault);
}

void RollerSystemIOSpark::RunVolts(double volts) { spark->SetVoltage(volts); }

void RollerSystemIOSpark::Stop() { spark->StopMotor(); }

void RollerSystemIOSpark::SetBrakeMode(bool enabled) {
  if (brakeModeEnabled == enabled)
    return;
  brakeModeEnabled = enabled;
  std::thread([this, enabled]() {
    SparkUtil::TryUntilOk(spark, 5, [&]() {
      rev::SparkBaseConfig tempConfig = config;
      tempConfig.SetIdleMode(enabled ? rev::SparkBaseConfig::IdleMode::kBrake
                                     : rev::SparkBaseConfig::IdleMode::kCoast);
      return spark->Configure(
          tempConfig, rev::CANSparkBase::ResetMode::kNoResetSafeParameters,
          rev::CANSparkBase::PersistMode::kNoPersistParameters);
    });
  }).detach();
}