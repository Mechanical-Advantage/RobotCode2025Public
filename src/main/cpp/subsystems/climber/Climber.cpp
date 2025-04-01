// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Climber.h"

#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc2/command/Commands.h"
#include "units/angle.h"
#include "units/time.h"

#include "junction/Logger.h"
#include "util/LoggedTracer.h"
#include "util/LoggedTunableNumber.h"

LoggedTunableNumber Climber::deployCurrent("Climber/DeployCurrent", 30);
LoggedTunableNumber Climber::deployAngle("Climber/DeployAngle", 130);
LoggedTunableNumber Climber::undeployAngle("Climber/UndeployAngle", 140);
LoggedTunableNumber Climber::climbCurrent("Climber/ClimbCurrent", 65);
LoggedTunableNumber
    Climber::climbCurrentRampRate("Climber/ClimbCurrentRampRate", 80);
LoggedTunableNumber Climber::climbStopAngle("Climber/ClimbStopAngle", 220);

Climber::Climber(ClimberIO &climberIO) : climberIO(climberIO) {
  climberIO.SetBrakeMode(true);
}

void Climber::Periodic() {
  climberIO.UpdateInputs(climberInputs);
  Logger::ProcessInputs("Climber", climberInputs);

  // Stop when disabled
  if (frc::DriverStation::IsDisabled()) {
    climberIO.RunTorqueCurrent(0.0);
  }

  // Set brake mode
  bool coast = coastOverride() && frc::DriverStation::IsDisabled();
  SetBrakeMode(!coast);
  Logger::RecordOutput("Climber/CoastOverride", !coast);

  // Record cycle time
  LoggedTracer::Record("Climber");
}

frc2::CommandPtr Climber::Deploy() {
  return frc2::CommandPtr(new frc2::FunctionalCommand(
      [this]() { climberIO.RunTorqueCurrent(deployCurrent.Get()); },
      [this]() {
        return climberInputs.data.positionRads >=
               units::degree_t(deployAngle.Get());
      },
      [this](bool interrupted) { climberIO.RunTorqueCurrent(0.0); }, {this}));
}

frc2::CommandPtr Climber::Undeploy() {
  return frc2::CommandPtr(new frc2::FunctionalCommand(
      [this]() { climberIO.RunTorqueCurrent(-deployCurrent.Get()); },
      [this]() {
        return climberInputs.data.positionRads <=
               units::degree_t(undeployAngle.Get());
      },
      [this](bool interrupted) { climberIO.RunTorqueCurrent(0.0); }, {this}));
}

frc2::CommandPtr Climber::Climb() {
  frc::Timer timer;
  return frc2::CommandPtr(new frc2::SequentialCommandGroup(
      frc2::CommandPtr(
          new frc2::InstantCommand([&timer]() { timer.Restart(); })),
      frc2::CommandPtr(new frc2::FunctionalCommand(
          [this, &timer]() {
            bool stopped = climberInputs.data.positionRads >=
                           units::degree_t(climbStopAngle.Get());
            if (stopped) {
              timer.Restart();
            }
            climberIO.RunTorqueCurrent(
                stopped ? 0.0
                        : std::min(climbCurrentRampRate.Get() * timer.Get(),
                                   climbCurrent.Get()));
          },
          []() { return false; },
          [this](bool interrupted) { climberIO.RunTorqueCurrent(0.0); },
          {this}))));
}

void Climber::SetBrakeMode(bool enabled) {
  if (brakeModeEnabled == enabled)
    return;
  brakeModeEnabled = enabled;
  climberIO.SetBrakeMode(enabled);
}

void Climber::SetCoastOverride(std::function<bool()> coastOverride) {
  this->coastOverride = coastOverride;
}