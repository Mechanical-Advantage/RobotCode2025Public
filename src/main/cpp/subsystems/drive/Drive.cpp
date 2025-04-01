// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/Drive.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/math/VecBuilder.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/Robot.h"
#include "org/littletonrobotics/frc2025/RobotState.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/junction/Logger.h"

std::mutex Drive::odometryLock;
LoggedTunableNumber Drive::coastWaitTime("Drive/CoastWaitTimeSeconds", 0.5);
LoggedTunableNumber
    Drive::coastMetersPerSecondThreshold("Drive/CoastMetersPerSecThreshold",
                                         0.05);

Drive::Drive(GyroIO *gyroIO, ModuleIO *flModuleIO, ModuleIO *frModuleIO,
             ModuleIO *blModuleIO, ModuleIO *brModuleIO)
    : gyroIO(gyroIO), modules({Module(flModuleIO, 0), Module(frModuleIO, 1),
                               Module(blModuleIO, 2), Module(brModuleIO, 3)}),
      swerveSetpointGenerator(kinematics, DriveConstants::moduleTranslations) {
  lastMovementTimer.Start();
  SetBrakeMode(true);

  // Start odometry thread
  PhoenixOdometryThread::GetInstance().Start();
}

void Drive::Periodic() {
  std::lock_guard<std::mutex> lock(
      odometryLock); // Prevents odometry updates while reading data
  gyroIO->UpdateInputs(gyroInputs);
  Logger::ProcessInputs("Drive/Gyro", gyroInputs);
  for (auto &module : modules) {
    module.UpdateInputs();
  }
  LoggedTracer::Record("Drive/Inputs");

  // Call periodic on modules
  for (auto &module : modules) {
    module.Periodic();
  }

  // Stop moving when disabled
  if (frc::DriverStation::IsDisabled()) {
    for (auto &module : modules) {
      module.Stop();
    }
  }

  // Log empty setpoint states when disabled
  if (frc::DriverStation::IsDisabled()) {
    Logger::RecordOutput("Drive/SwerveStates/Setpoints",
                         std::vector<frc::SwerveModuleState>{});
    Logger::RecordOutput("Drive/SwerveStates/SetpointsUnoptimized",
                         std::vector<frc::SwerveModuleState>{});
  }

  // Send odometry updates to robot state
  std::vector<double> sampleTimestamps;
  if (org::littletonrobotics::frc2025::Constants::GetMode() ==
      org::littletonrobotics::frc2025::Constants::Mode::SIM) {
    sampleTimestamps.push_back(frc::Timer::GetFPGATimestamp());
  } else {
    sampleTimestamps.assign(gyroInputs.odometryYawTimestamps.begin(),
                            gyroInputs.odometryYawTimestamps.end());
  }
  int sampleCount = sampleTimestamps.size();
  for (int i = 0; i < sampleCount; i++) {
    std::array<frc::SwerveModulePosition, 4> wheelPositions;
    for (int j = 0; j < 4; j++) {
      wheelPositions[j] = modules[j].GetOdometryPositions()[i];
    }
    std::optional<frc::Rotation2d> gyroRotation;
    if (gyroInputs.data.connected() &&
        org::littletonrobotics::frc2025::Constants::GetMode() !=
            org::littletonrobotics::frc2025::Constants::Mode::SIM) {
      gyroRotation = gyroInputs.odometryYawPositions[i];
    }
    RobotState::GetInstance().AddOdometryObservation(
        RobotState::OdometryObservation{wheelPositions, gyroRotation,
                                        sampleTimestamps[i]});
  }

  RobotState::GetInstance().AddDriveSpeeds(GetChassisSpeeds());

  // Update brake mode
  // Reset movement timer if velocity above threshold
  if (std::any_of(modules.begin(), modules.end(), [&](const Module &module) {
        return std::abs(module.GetVelocityMetersPerSec()) >
               coastMetersPerSecondThreshold.Get();
      })) {
    lastMovementTimer.Reset();
  }

  if (frc::DriverStation::IsEnabled()) {
    coastRequest = CoastRequest::AUTOMATIC;
  }

  switch (coastRequest) {
  case CoastRequest::AUTOMATIC:
    if (frc::DriverStation::IsEnabled()) {
      SetBrakeMode(true);
    } else if (lastMovementTimer.HasElapsed(coastWaitTime.Get())) {
      SetBrakeMode(false);
    }
    break;
  case CoastRequest::ALWAYS_BREAK:
    SetBrakeMode(true);
    break;
  case CoastRequest::ALWAYS_COAST:
    SetBrakeMode(false);
    break;
  }

  // Update current setpoint if not in velocity mode
  if (!velocityMode) {
    currentSetpoint = SwerveSetpoint(GetChassisSpeeds(), GetModuleStates());
  }

  // Update gyro alert
  gyroDisconnectedAlert.Set(
      !gyroInputs.data.connected() &&
      org::littletonrobotics::frc2025::Constants::GetMode() !=
          org::littletonrobotics::frc2025::Constants::Mode::SIM &&
      !Robot::IsJITing());

  // Record cycle time
  LoggedTracer::Record("Drive/Periodic");
}

void Drive::SetBrakeMode(bool enabled) {
  if (brakeModeEnabled != enabled) {
    for (auto &module : modules) {
      module.SetBrakeMode(enabled);
    }
  }
  brakeModeEnabled = enabled;
}

void Drive::RunVelocity(frc::ChassisSpeeds speeds) {
  velocityMode = true;
  // Calculate module setpoints
  frc::ChassisSpeeds discreteSpeeds = frc::ChassisSpeeds::Discretize(
      speeds, org::littletonrobotics::frc2025::Constants::loopPeriodSecs);
  std::array<frc::SwerveModuleState, 4> setpointStatesUnoptimized =
      kinematics.ToSwerveModuleStates(discreteSpeeds);
  currentSetpoint = swerveSetpointGenerator.GenerateSetpoint(
      DriveConstants::moduleLimitsFree, currentSetpoint, discreteSpeeds,
      org::littletonrobotics::frc2025::Constants::loopPeriodSecs);
  std::array<frc::SwerveModuleState, 4> setpointStates =
      currentSetpoint.moduleStates();

  // Log unoptimized setpoints and setpoint speeds
  Logger::RecordOutput(
      "Drive/SwerveStates/SetpointsUnoptimized",
      std::vector<frc::SwerveModuleState>(setpointStatesUnoptimized.begin(),
                                          setpointStatesUnoptimized.end()));
  Logger::RecordOutput("Drive/SwerveStates/Setpoints",
                       std::vector<frc::SwerveModuleState>(
                           setpointStates.begin(), setpointStates.end()));
  Logger::RecordOutput("Drive/SwerveChassisSpeeds/Setpoints",
                       currentSetpoint.chassisSpeeds());

  // Send setpoints to modules
  for (int i = 0; i < 4; i++) {
    modules[i].RunSetpoint(setpointStates[i]);
  }
}

void Drive::RunVelocity(
    frc::ChassisSpeeds speeds,
    const std::vector<frc::Vector<frc::numbers::N2>> &moduleForces) {
  velocityMode = true;
  // Calculate module setpoints
  frc::ChassisSpeeds discreteSpeeds = frc::ChassisSpeeds::Discretize(
      speeds, org::littletonrobotics::frc2025::Constants::loopPeriodSecs);
  std::array<frc::SwerveModuleState, 4> setpointStatesUnoptimized =
      kinematics.ToSwerveModuleStates(discreteSpeeds);
  currentSetpoint = swerveSetpointGenerator.GenerateSetpoint(
      DriveConstants::moduleLimitsFree, currentSetpoint, discreteSpeeds,
      org::littletonrobotics::frc2025::Constants::loopPeriodSecs);
  std::array<frc::SwerveModuleState, 4> setpointStates =
      currentSetpoint.moduleStates();

  // Log unoptimized setpoints and setpoint speeds
  Logger::RecordOutput(
      "Drive/SwerveStates/SetpointsUnoptimized",
      std::vector<frc::SwerveModuleState>(setpointStatesUnoptimized.begin(),
                                          setpointStatesUnoptimized.end()));
  Logger::RecordOutput("Drive/SwerveStates/Setpoints",
                       std::vector<frc::SwerveModuleState>(
                           setpointStates.begin(), setpointStates.end()));
  Logger::RecordOutput("Drive/SwerveChassisSpeeds/Setpoints",
                       currentSetpoint.chassisSpeeds());

  // Save module forces to swerve states for logging
  std::array<frc::SwerveModuleState, 4> wheelForces;
  // Send setpoints to modules
  std::array<frc::SwerveModuleState, 4> moduleStates = GetModuleStates();
  for (int i = 0; i < 4; i++) {
    // Optimize state
    frc::Rotation2d wheelAngle = moduleStates[i].angle;
    setpointStates[i].Optimize(wheelAngle);
    setpointStates[i].CosineScale(wheelAngle);

    // Calculate wheel torque in direction
    frc::Vector<frc::numbers::N2> wheelForce = moduleForces[i];
    frc::Vector<frc::numbers::N2> wheelDirection =
        frc::math::VecBuilder<frc::numbers::N2>::fill(wheelAngle.Cos(),
                                                      wheelAngle.Sin());
    double wheelTorqueNm =
        wheelForce.Dot(wheelDirection) * DriveConstants::wheelRadius;
    modules[i].RunSetpoint(setpointStates[i], wheelTorqueNm);

    // Save to array for logging
    wheelForces[i] =
        frc::SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
  }
  Logger::RecordOutput("Drive/SwerveStates/ModuleForces",
                       std::vector<frc::SwerveModuleState>(wheelForces.begin(),
                                                           wheelForces.end()));
}

void Drive::RunCharacterization(double output) {
  velocityMode = false;
  for (int i = 0; i < 4; i++) {
    modules[i].RunCharacterization(output);
  }
}

void Drive::Stop() { RunVelocity(frc::ChassisSpeeds()); }

void Drive::StopWithX() {
  std::array<frc::Rotation2d, 4> headings;
  for (int i = 0; i < 4; i++) {
    headings[i] = DriveConstants::moduleTranslations[i].Angle();
  }
  kinematics.ResetHeadings(headings);
  Stop();
}

std::array<frc::SwerveModuleState, 4> Drive::GetModuleStates() {
  std::array<frc::SwerveModuleState, 4> states;
  for (int i = 0; i < 4; i++) {
    states[i] = modules[i].GetState();
  }
  return states;
}

frc::ChassisSpeeds Drive::GetChassisSpeeds() {
  return kinematics.ToChassisSpeeds(GetModuleStates());
}

double *Drive::GetWheelRadiusCharacterizationPositions() {
  static double values[4];
  for (int i = 0; i < 4; i++) {
    values[i] = modules[i].GetWheelRadiusCharacterizationPosition();
  }
  return values;
}

double Drive::GetFFCharacterizationVelocity() {
  double output = 0.0;
  for (int i = 0; i < 4; i++) {
    output += modules[i].GetFFCharacterizationVelocity() / 4.0;
  }
  return output;
}

frc::Rotation2d Drive::GetGyroRotation() {
  return gyroInputs.data.yawPosition();
}

double Drive::GetMaxLinearSpeedMetersPerSec() {
  return DriveConstants::maxLinearSpeed;
}

double Drive::GetMaxAngularSpeedRadPerSec() {
  return GetMaxLinearSpeedMetersPerSec() / DriveConstants::driveBaseRadius;
}
