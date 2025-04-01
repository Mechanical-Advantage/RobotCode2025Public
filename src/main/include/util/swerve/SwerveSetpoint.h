// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>

struct SwerveSetpoint {
  frc::kinematics::ChassisSpeeds chassisSpeeds;
  std::array<frc::kinematics::SwerveModuleState, 4>
      moduleStates; // Assuming 4 modules, adjust if needed.
};