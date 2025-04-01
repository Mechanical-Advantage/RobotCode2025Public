// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <string>

#include "frc/Alert.h"
#include "frc2/command/Command.h"
#include "frc2/command/SubsystemBase.h"
#include "org/littletonrobotics/frc2025/subsystems/rollers/RollerSystemIO.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/junction/AutoLog.h"

class RollerSystem : public frc2::SubsystemBase {
public:
  RollerSystem(const std::string &name, RollerSystemIO &io);
  ~RollerSystem() override = default;

  void Periodic() override;

  frc2::CommandPtr RunRoller(std::function<double()> inputVolts);

private:
  const std::string name;
  RollerSystemIO &io;
  RollerSystemIO::RollerSystemIOInputs inputs;
  frc::Alert disconnected;
  frc::Alert tempFault;

  AUTO_LOG_OUTPUT(inputs);
};