// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/superstructure/chariot/Chariot.h"

#include "frc/units/length.h"
#include "frc2/command/Commands.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/SuperstructureConstants.h"
#include "org/littletonrobotics/junction/Logger.h"

frc::units::meter_t Chariot::drumRadius = frc::units::inch_t(0.5);

LoggedTunableNumber Chariot::occupiedVolts =
    LoggedTunableNumber("Chariot/OccupiedVolts", 5.0);
LoggedTunableNumber Chariot::floorIntakeVolts =
    LoggedTunableNumber("Chariot/FloorIntakeVolts", 8.0);
LoggedTunableNumber Chariot::halfOutPositionInches =
    LoggedTunableNumber("Chariot/StupidDeployPositionInches", 5.0);

Chariot::Chariot(ChariotIO &chariotIO, RollerSystemIO &rollerIO)
    : GenericSlamElevator("Chariot/Chariot", chariotIO, 0.2, 0.5),
      rollerIO(&rollerIO) {
  frc2::Trigger([this]() {
    return GetGoal() ==
               Goal{LoggedTunableNumber("Chariot/RetractCurrent", -30.0).Get(),
                    true, SlamElevatorState::RETRACTING} &&
           slammed;
  }).OnTrue(frc2::cmd::RunOnce([this]() {
    home = inputs.data.positionRads * drumRadius.value();
  }));
  frc2::Trigger([this]() {
    return GetGoal() ==
               Goal{LoggedTunableNumber("Chariot/DeployCurrent", 30.0).Get(),
                    true, SlamElevatorState::EXTENDING} &&
           slammed;
  }).OnTrue(frc2::cmd::RunOnce([this]() {
    home = inputs.data.positionRads * drumRadius.value() -
           SuperstructureConstants::chariotMaxExtension.value();
  }));
}

void Chariot::Periodic() {
  GenericSlamElevator::Periodic();
  rollerIO->UpdateInputs(rollerInputs);
  Logger::ProcessInputs("Chariot/Roller", rollerInputs);

  if (Constants::GetRobot() == Constants::RobotType::DEVBOT) {
    slammed = true;
  }

  position = inputs.data.positionRads * drumRadius.value() - home;
  if (GetGoal() ==
          Goal{LoggedTunableNumber("Chariot/StupidDeployCurrent", 10.0).Get(),
               true, SlamElevatorState::EXTENDING} &&
      position >= frc::units::inch_t(halfOutPositionInches.Get()).value()) {
    slammed = true;
  }

  rollerIO->RunVolts(intakeVolts);

  LoggedTracer::Record("Chariot");
}

Chariot::Goal Chariot::GetGoal() const { return goal; }

void Chariot::SetGoal(Goal goal) { this->goal = goal; }

double Chariot::GetPosition() const { return position; }

void Chariot::SetIntakeVolts(double intakeVolts) {
  this->intakeVolts = intakeVolts;
}

void Chariot::SetCoastOverride(std::function<bool()> coastOverride) {
  this->coastOverride = coastOverride;
}