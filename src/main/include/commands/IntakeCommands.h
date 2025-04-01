// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "subsystems/rollers/rollersystem.h"
#include "subsystems/superstructure/superstructure.h"
#include "util/loggedtunablenumber.h"

namespace org::littletonrobotics::frc2025::commands {

class IntakeCommands {
public:
  static LoggedTunableNumber funnelVolts;
  static LoggedTunableNumber outtakeVolts;

  static frc2::CommandPtr
  Intake(subsystems::superstructure::Superstructure &superstructure,
         subsystems::rollers::RollerSystem &funnel);

private:
  IntakeCommands() = delete;
};

} // namespace org::littletonrobotics::frc2025::commands