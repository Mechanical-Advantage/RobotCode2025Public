// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "util/EqualsUtil.h"
#include "util/GeomUtil.h"
#include "util/swerve/ModuleLimits.h"
#include "util/swerve/SwerveSetpoint.h"

/**
 * "Inspired" by FRC team 254. See the license file in the root directory of
 * this project.
 *
 * <p>Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver,
 * or from a path follower), and outputs a new setpoint that respects all of the
 * kinematic constraints on module rotation speed and wheel
 * velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any
 * intermediate state that is kinematically infeasible (and can result in wheel
 * slip or robot heading drift as a result).
 */
class SwerveSetpointGenerator {
public:
  SwerveSetpointGenerator(
      frc::kinematics::SwerveDriveKinematics kinematics,
      std::array<frc::geometry::Translation2d, 4> moduleLocations);

  SwerveSetpoint GenerateSetpoint(const ModuleLimits &limits,
                                  const SwerveSetpoint &prevSetpoint,
                                  frc::kinematics::ChassisSpeeds desiredState,
                                  double dt);

private:
  bool FlipHeading(frc::geometry::Rotation2d prevToGoal);
  double UnwrapAngle(double ref, double angle);

  using Function2d = std::function<double(double, double)>;

  double FindRoot(Function2d func, double x_0, double y_0, double f_0,
                  double x_1, double y_1, double f_1, int iterations_left);
  double FindSteeringMaxS(double x_0, double y_0, double f_0, double x_1,
                          double y_1, double f_1, double max_deviation,
                          int max_iterations);
  double FindDriveMaxS(double x_0, double y_0, double f_0, double x_1,
                       double y_1, double f_1, double max_vel_step,
                       int max_iterations);

  frc::kinematics::SwerveDriveKinematics kinematics;
  std::array<frc::geometry::Translation2d, 4> moduleLocations;
};