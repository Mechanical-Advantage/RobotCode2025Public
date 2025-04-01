// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "intakecommands.h"

#include <frc2/command/Commands.h>

#include "subsystems/superstructure/superstructurestate.h"

namespace org::littletonrobotics::frc2025::commands {

LoggedTunableNumber IntakeCommands::funnelVolts{"Funnel/IntakeVolts", 8};
LoggedTunableNumber IntakeCommands::outtakeVolts{"Funnel/OuttakeVolts", -10};

frc2::CommandPtr IntakeCommands::Intake(
    subsystems::superstructure::Superstructure &superstructure,
    subsystems::rollers::RollerSystem &funnel) {
  return superstructure
      .RunGoal(subsystems::superstructure::SuperstructureState::kIntake)
      .AlongWith(frc2::Commands::WaitUntil([&]() {
                   return superstructure.AtGoal();
                 }).Then(funnel.RunRoller(funnelVolts)));
}

} // namespace org::littletonrobotics::frc2025::commands