// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/Alert.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/units/units.h"

#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/util/LoggedTunableNumber.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIOInputsAutoLogged.h"

class Module {
public:
  Module(ModuleIO *io, int index);

  void UpdateInputs();
  void Periodic();

  void RunSetpoint(frc::SwerveModuleState state);
  void RunSetpoint(frc::SwerveModuleState state, double wheelTorqueNm);
  void RunCharacterization(double output);
  void Stop();

  frc::Rotation2d GetAngle();
  double GetPositionMeters();
  double GetVelocityMetersPerSec();
  frc::SwerveModulePosition GetPosition();
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition *GetOdometryPositions();
  double GetWheelRadiusCharacterizationPosition();
  double GetFFCharacterizationVelocity();

  void SetBrakeMode(bool enabled);

private:
  static LoggedTunableNumber drivekS;
  static LoggedTunableNumber drivekV;
  static LoggedTunableNumber drivekT;
  static LoggedTunableNumber drivekP;
  static LoggedTunableNumber drivekD;
  static LoggedTunableNumber turnkP;
  static LoggedTunableNumber turnkD;

  ModuleIO *io;
  ModuleIOInputsAutoLogged inputs;
  int index;

  frc::SimpleMotorFeedforward ffModel;

  frc::Alert driveDisconnectedAlert;
  frc::Alert turnDisconnectedAlert;
  frc::Alert turnEncoderDisconnectedAlert;
  frc::SwerveModulePosition *odometryPositions;
};