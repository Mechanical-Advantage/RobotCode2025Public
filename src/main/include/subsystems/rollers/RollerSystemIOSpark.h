// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include "frc/filter/Debouncer.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"
#include "org/littletonrobotics/frc2025/util/SparkUtil.h"
#include "rev/CANSparkBase.h"
#include "rev/CANSparkFlex.h"
#include "rev/CANSparkMax.h"
#include "rev/RelativeEncoder.h"
#include "rev/SparkBaseConfig.h"
#include "rev/SparkFlexConfig.h"
#include "rev/SparkMaxConfig.h"

class RollerSystemIOSpark : public RollerSystemIO {
public:
  RollerSystemIOSpark(int deviceId, bool isFlex);
  ~RollerSystemIOSpark() override = default;

  void UpdateInputs(RollerSystemIOInputs &inputs) override;
  void RunVolts(double volts) override;
  void Stop() override;
  void SetBrakeMode(bool enabled) override;

private:
  rev::CANSparkBase *spark;
  rev::RelativeEncoder *encoder;
  rev::SparkBaseConfig config;

  frc::Debouncer connectedDebouncer = frc::Debouncer(0.5);
  int currentLimit = 60;
  bool brakeModeEnabled = true;
};