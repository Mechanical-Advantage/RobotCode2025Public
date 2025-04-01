// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/Module.h"

#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/units/units.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/Robot.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/junction/Logger.h"

LoggedTunableNumber Module::drivekS("Drive/Module/DrivekS");
LoggedTunableNumber Module::drivekV("Drive/Module/DrivekV");
LoggedTunableNumber Module::drivekT("Drive/Module/DrivekT");
LoggedTunableNumber Module::drivekP("Drive/Module/DrivekP");
LoggedTunableNumber Module::drivekD("Drive/Module/DrivekD");
LoggedTunableNumber Module::turnkP("Drive/Module/TurnkP");
LoggedTunableNumber Module::turnkD("Drive/Module/TurnkD");

Module::Module(ModuleIO *io, int index)
    : io(io), index(index), ffModel(drivekS.Get(), drivekV.Get()),
      driveDisconnectedAlert("Disconnected drive motor on module " +
                                 std::to_string(index) + ".",
                             frc::Alert::AlertType::kError),
      turnDisconnectedAlert("Disconnected turn motor on module " +
                                std::to_string(index) + ".",
                            frc::Alert::AlertType::kError),
      turnEncoderDisconnectedAlert("Disconnected turn encoder on module " +
                                       std::to_string(index) + ".",
                                   frc::Alert::AlertType::kError) {
  switch (org::littletonrobotics::frc2025::Constants::GetRobot()) {
  case org::littletonrobotics::frc2025::Constants::RobotType::COMPBOT:
  case org::littletonrobotics::frc2025::Constants::RobotType::DEVBOT:
    drivekS.InitDefault(5.0);
    drivekV.InitDefault(0);
    drivekT.InitDefault(ModuleIOComp::driveReduction /
                        frc::DCMotor::KrakenX60Foc(1).KtNMPerAmp());
    drivekP.InitDefault(35.0);
    drivekD.InitDefault(0);
    turnkP.InitDefault(4000.0);
    turnkD.InitDefault(50.0);
    break;
  default:
    drivekS.InitDefault(0.014);
    drivekV.InitDefault(0.134);
    drivekT.InitDefault(0);
    drivekP.InitDefault(0.1);
    drivekD.InitDefault(0);
    turnkP.InitDefault(10.0);
    turnkD.InitDefault(0);
    break;
  }
}

void Module::UpdateInputs() {
  io->UpdateInputs(inputs);
  Logger::ProcessInputs("Drive/Module" + std::to_string(index), inputs);
}

void Module::Periodic() {
  // Update tunable numbers
  if (drivekS.HasChanged(this->hashCode()) ||
      drivekV.HasChanged(this->hashCode())) {
    ffModel = frc::SimpleMotorFeedforward(drivekS.Get(), drivekV.Get());
  }
  if (drivekP.HasChanged(this->hashCode()) ||
      drivekD.HasChanged(this->hashCode())) {
    io->SetDrivePID(drivekP.Get(), 0, drivekD.Get());
  }
  if (turnkP.HasChanged(this->hashCode()) ||
      turnkD.HasChanged(this->hashCode())) {
    io->SetTurnPID(turnkP.Get(), 0, turnkD.Get());
  }

  // Calculate positions for odometry
  int sampleCount = inputs.odometryDrivePositionsRad
                        .size(); // All signals are sampled together
  odometryPositions = new frc::SwerveModulePosition[sampleCount];
  for (int i = 0; i < sampleCount; i++) {
    double positionMeters =
        inputs.odometryDrivePositionsRad[i] * DriveConstants::wheelRadius;
    frc::Rotation2d angle = inputs.odometryTurnPositions[i];
    odometryPositions[i] = frc::SwerveModulePosition(positionMeters, angle);
  }

  // Update alerts
  driveDisconnectedAlert.Set(
      !inputs.data.driveConnected &&
      !org::littletonrobotics::frc2025::Robot::IsJITing());
  turnDisconnectedAlert.Set(
      !inputs.data.turnConnected &&
      !org::littletonrobotics::frc2025::Robot::IsJITing());
  turnEncoderDisconnectedAlert.Set(
      !inputs.data.turnEncoderConnected &&
      !org::littletonrobotics::frc2025::Robot::IsJITing());

  // Record cycle time
  LoggedTracer::Record("Drive/Module" + std::to_string(index));
}

void Module::RunSetpoint(frc::SwerveModuleState state) {
  // Apply setpoints
  double speedRadPerSec =
      state.speedMetersPerSecond / DriveConstants::wheelRadius;
  io->RunDriveVelocity(speedRadPerSec, ffModel.Calculate(speedRadPerSec));
  io->RunTurnPosition(state.angle);
}

void Module::RunSetpoint(frc::SwerveModuleState state, double wheelTorqueNm) {
  // Apply setpoints
  double speedRadPerSec =
      state.speedMetersPerSecond / DriveConstants::wheelRadius;
  io->RunDriveVelocity(speedRadPerSec, ffModel.Calculate(speedRadPerSec) +
                                           wheelTorqueNm * drivekT.Get());
  io->RunTurnPosition(state.angle);
}

void Module::RunCharacterization(double output) {
  io->RunDriveOpenLoop(output);
  io->RunTurnPosition(frc::Rotation2d());
}

void Module::Stop() {
  io->RunDriveOpenLoop(0.0);
  io->RunTurnOpenLoop(0.0);
}

frc::Rotation2d Module::GetAngle() { return inputs.data.turnPosition; }

double Module::GetPositionMeters() {
  return inputs.data.drivePositionRad * DriveConstants::wheelRadius;
}

double Module::GetVelocityMetersPerSec() {
  return inputs.data.driveVelocityRadPerSec * DriveConstants::wheelRadius;
}

frc::SwerveModulePosition Module::GetPosition() {
  return frc::SwerveModulePosition(GetPositionMeters(), GetAngle());
}

frc::SwerveModuleState Module::GetState() {
  return frc::SwerveModuleState(GetVelocityMetersPerSec(), GetAngle());
}

frc::SwerveModulePosition *Module::GetOdometryPositions() {
  return odometryPositions;
}

double Module::GetWheelRadiusCharacterizationPosition() {
  return inputs.data.drivePositionRad;
}

double Module::GetFFCharacterizationVelocity() {
  return frc::units::radians_to_rotations(inputs.data.driveVelocityRadPerSec);
}

void Module::SetBrakeMode(bool enabled) { io->SetBrakeMode(enabled); }