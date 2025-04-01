// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <vector>

#include "frc/Alert.h"
#include "frc/Timer.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/numbers/Numbers.h"
#include "frc2/command/SubsystemBase.h"

#include "org/littletonrobotics/frc2025/util/LoggedTunableNumber.h"
#include "org/littletonrobotics/frc2025/util/swerve/SwerveSetpoint.h"
#include "org/littletonrobotics/frc2025/util/swerve/SwerveSetpointGenerator.h"
#include "org/littletonrobotics/junction/AutoLogOutput.h"

#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/RobotState.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/GyroIOInputsAutoLogged.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/Module.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/ModuleIO.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.h"

class Drive : public frc2::SubsystemBase {
public:
  enum class CoastRequest { AUTOMATIC, ALWAYS_BREAK, ALWAYS_COAST };

  Drive(GyroIO *gyroIO, ModuleIO *flModuleIO, ModuleIO *frModuleIO,
        ModuleIO *blModuleIO, ModuleIO *brModuleIO);

  void Periodic() override;

  void RunVelocity(frc::ChassisSpeeds speeds);
  void
  RunVelocity(frc::ChassisSpeeds speeds,
              const std::vector<frc::Vector<frc::numbers::N2>> &moduleForces);
  void RunCharacterization(double output);
  void Stop();
  void StopWithX();

  double *GetWheelRadiusCharacterizationPositions();
  double GetFFCharacterizationVelocity();
  frc::Rotation2d GetGyroRotation();
  double GetMaxLinearSpeedMetersPerSec();
  double GetMaxAngularSpeedRadPerSec();

  static std::mutex odometryLock;

private:
  GyroIO *gyroIO;
  GyroIOInputsAutoLogged gyroInputs;
  std::array<Module, 4> modules;
  frc::Alert gyroDisconnectedAlert{
      "Disconnected gyro, using kinematics as fallback.",
      frc::Alert::AlertType::kError};

  static LoggedTunableNumber coastWaitTime;
  static LoggedTunableNumber coastMetersPerSecondThreshold;

  frc::Timer lastMovementTimer;

  frc::SwerveDriveKinematics kinematics{DriveConstants::moduleTranslations};

  bool velocityMode = false;
  bool brakeModeEnabled = true;

  SwerveSetpoint currentSetpoint{frc::ChassisSpeeds(),
                                 std::array<frc::SwerveModuleState, 4>{}};
  SwerveSetpointGenerator swerveSetpointGenerator{
      kinematics, DriveConstants::moduleTranslations};

  CoastRequest coastRequest = CoastRequest::AUTOMATIC;
  void setCoastRequest(CoastRequest request) { coastRequest = request; }

  void SetBrakeMode(bool enabled);

  std::array<frc::SwerveModuleState, 4> GetModuleStates();
  frc::ChassisSpeeds GetChassisSpeeds();
};