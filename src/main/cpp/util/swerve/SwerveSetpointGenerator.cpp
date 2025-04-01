// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/swerve/SwerveSetpointGenerator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "util/EqualsUtil.h"
#include "util/GeomUtil.h"

SwerveSetpointGenerator::SwerveSetpointGenerator(
    frc::kinematics::SwerveDriveKinematics kinematics,
    std::array<frc::geometry::Translation2d, 4> moduleLocations)
    : kinematics(kinematics), moduleLocations(moduleLocations) {}

bool SwerveSetpointGenerator::FlipHeading(
    frc::geometry::Rotation2d prevToGoal) {
  return std::abs(prevToGoal.Radians()) > M_PI / 2.0;
}

double SwerveSetpointGenerator::UnwrapAngle(double ref, double angle) {
  double diff = angle - ref;
  if (diff > M_PI) {
    return angle - 2.0 * M_PI;
  } else if (diff < -M_PI) {
    return angle + 2.0 * M_PI;
  } else {
    return angle;
  }
}

double SwerveSetpointGenerator::FindRoot(Function2d func, double x_0,
                                         double y_0, double f_0, double x_1,
                                         double y_1, double f_1,
                                         int iterations_left) {
  if (iterations_left < 0 || EqualsUtil::EpsilonEquals(f_0, f_1)) {
    return 1.0;
  }
  double s_guess = std::max(0.0, std::min(1.0, -f_0 / (f_1 - f_0)));
  double x_guess = (x_1 - x_0) * s_guess + x_0;
  double y_guess = (y_1 - y_0) * s_guess + y_0;
  double f_guess = func(x_guess, y_guess);
  if (std::signbit(f_0) == std::signbit(f_guess)) {
    // 0 and guess on same side of root, so use upper bracket.
    return s_guess + (1.0 - s_guess) * FindRoot(func, x_guess, y_guess, f_guess,
                                                x_1, y_1, f_1,
                                                iterations_left - 1);
  } else {
    // Use lower bracket.
    return s_guess * FindRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess,
                              iterations_left - 1);
  }
}

double SwerveSetpointGenerator::FindSteeringMaxS(double x_0, double y_0,
                                                 double f_0, double x_1,
                                                 double y_1, double f_1,
                                                 double max_deviation,
                                                 int max_iterations) {
  f_1 = UnwrapAngle(f_0, f_1);
  double diff = f_1 - f_0;
  if (std::abs(diff) <= max_deviation) {
    // Can go all the way to s=1.
    return 1.0;
  }
  double offset = f_0 + std::copysign(max_deviation, diff);
  Function2d func = [this, f_0, offset](double x, double y) {
    return UnwrapAngle(f_0, std::atan2(y, x)) - offset;
  };
  return FindRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset,
                  max_iterations);
}

double SwerveSetpointGenerator::FindDriveMaxS(double x_0, double y_0,
                                              double f_0, double x_1,
                                              double y_1, double f_1,
                                              double max_vel_step,
                                              int max_iterations) {
  double diff = f_1 - f_0;
  if (std::abs(diff) <= max_vel_step) {
    // Can go all the way to s=1.
    return 1.0;
  }
  double offset = f_0 + std::copysign(max_vel_step, diff);
  Function2d func = [](double x, double y) {
    return std::hypot(x, y) - offset;
  };
  return FindRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset,
                  max_iterations);
}

SwerveSetpoint SwerveSetpointGenerator::GenerateSetpoint(
    const ModuleLimits &limits, const SwerveSetpoint &prevSetpoint,
    frc::kinematics::ChassisSpeeds desiredState, double dt) {
  std::array<frc::geometry::Translation2d, 4> modules = moduleLocations;

  std::array<frc::kinematics::SwerveModuleState, 4> desiredModuleState =
      kinematics.ToSwerveModuleStates(desiredState);
  // Make sure desiredState respects velocity limits.
  if (limits.maxDriveVelocity > 0.0) {
    frc::kinematics::SwerveDriveKinematics::DesaturateWheelSpeeds(
        &desiredModuleState, limits.maxDriveVelocity);
    desiredState = kinematics.ToChassisSpeeds(desiredModuleState);
  }

  // Special case: desiredState is a complete stop. In this case, module angle
  // is arbitrary, so just use the previous angle.
  bool need_to_steer = true;
  if (desiredState.ToTwist2d().EpsilonEquals(frc::geometry::Twist2d())) {
    need_to_steer = false;
    for (int i = 0; i < modules.size(); ++i) {
      desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle;
      desiredModuleState[i].speedMetersPerSecond = 0.0;
    }
  }

  // For each module, compute local Vx and Vy vectors.
  std::array<double, 4> prev_vx;
  std::array<double, 4> prev_vy;
  std::array<frc::geometry::Rotation2d, 4> prev_heading;
  std::array<double, 4> desired_vx;
  std::array<double, 4> desired_vy;
  std::array<frc::geometry::Rotation2d, 4> desired_heading;
  bool all_modules_should_flip = true;
  for (int i = 0; i < modules.size(); ++i) {
    prev_vx[i] = prevSetpoint.moduleStates[i].angle.Cos() *
                 prevSetpoint.moduleStates[i].speedMetersPerSecond;
    prev_vy[i] = prevSetpoint.moduleStates[i].angle.Sin() *
                 prevSetpoint.moduleStates[i].speedMetersPerSecond;
    prev_heading[i] = prevSetpoint.moduleStates[i].angle;
    if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0.0) {
      prev_heading[i] = prev_heading[i] + frc::geometry::Rotation2d(M_PI);
    }
    desired_vx[i] = desiredModuleState[i].angle.Cos() *
                    desiredModuleState[i].speedMetersPerSecond;
    desired_vy[i] = desiredModuleState[i].angle.Sin() *
                    desiredModuleState[i].speedMetersPerSecond;
    desired_heading[i] = desiredModuleState[i].angle;
    if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
      desired_heading[i] = desired_heading[i] + frc::geometry::Rotation2d(M_PI);
    }
    if (all_modules_should_flip) {
      double required_rotation_rad = std::abs(
          (prev_heading[i].UnaryMinus() + desired_heading[i]).Radians());
      if (required_rotation_rad < M_PI / 2.0) {
        all_modules_should_flip = false;
      }
    }
  }
  if (all_modules_should_flip &&
      !prevSetpoint.chassisSpeeds.ToTwist2d().EpsilonEquals(
          frc::geometry::Twist2d()) &&
      !desiredState.ToTwist2d().EpsilonEquals(frc::geometry::Twist2d())) {
    // It will (likely) be faster to stop therobot, rotate the modules in place
    // to the complement of the desired angle, and accelerate again.
    return GenerateSetpoint(limits, prevSetpoint,
                            frc::kinematics::ChassisSpeeds(), dt);
  }

  // Compute the deltas between start and goal. We can then interpolate from the
  // start state to the goal state; then find the amount we can move from start
  // towards goal in this cycle such that no kinematic limit is exceeded.
  double dx = desiredState.vxMetersPerSecond -
              prevSetpoint.chassisSpeeds.vxMetersPerSecond;
  double dy = desiredState.vyMetersPerSecond -
              prevSetpoint.chassisSpeeds.vyMetersPerSecond;
  double dtheta = desiredState.omegaRadiansPerSecond -
                  prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

  // 's' interpolates between start and goal. At 0, we are at prevState and at
  // 1, we are at desiredState.
  double min_s = 1.0;

  // In cases where an individual module is stopped, we want to remember the
  // right steering angle to command (since inverse kinematics doesn't care
  // about angle, we can be opportunistically lazy).
  std::array<std::optional<frc::geometry::Rotation2d>, 4> overrideSteering;
  // Enforce steering velocity limits. We do this by taking the derivative of
  // steering angle at the current angle, and then backing out the maximum
  // interpolant between start and goal states. We remember the minimum across
  // all modules, since that is the active constraint.
  const double max_theta_step = dt * limits.maxSteeringVelocity;
  for (int i = 0; i < modules.size(); ++i) {
    if (!need_to_steer) {
      overrideSteering[i] = prevSetpoint.moduleStates[i].angle;
      continue;
    }
    overrideSteering[i] = std::nullopt;
    if (EqualsUtil::EpsilonEquals(
            prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
      // If module is stopped, we know that we will need to move straight to the
      // final steering angle, so limit based purely on rotation in place.
      if (EqualsUtil::EpsilonEquals(desiredModuleState[i].speedMetersPerSecond,
                                    0.0)) {
        // Goal angle doesn't matter. Just leave module at its current angle.
        overrideSteering[i] = prevSetpoint.moduleStates[i].angle;
        continue;
      }

      frc::geometry::Rotation2d necessaryRotation =
          prevSetpoint.moduleStates[i].angle.UnaryMinus() +
          desiredModuleState[i].angle;
      if (FlipHeading(necessaryRotation)) {
        necessaryRotation = necessaryRotation + frc::geometry::Rotation2d(M_PI);
      }
      // Radians() bounds to +/- Pi.
      const double numStepsNeeded =
          std::abs(necessaryRotation.Radians()) / max_theta_step;

      if (numStepsNeeded <= 1.0) {
        // Steer directly to goal angle.
        overrideSteering[i] = desiredModuleState[i].angle;
        // Don't limit the global min_s;
        continue;
      } else {
        // Adjust steering by max_theta_step.
        overrideSteering[i] = prevSetpoint.moduleStates[i].angle +
                              frc::geometry::Rotation2d(std::copysign(
                                  max_theta_step, necessaryRotation.Radians()));
        min_s = 0.0;
        continue;
      }
    }
    if (min_s == 0.0) {
      // s can't get any lower. Save some CPU.
      continue;
    }

    const int kMaxIterations = 8;
    double s = FindSteeringMaxS(prev_vx[i], prev_vy[i],
                                prev_heading[i].Radians(), desired_vx[i],
                                desired_vy[i], desired_heading[i].Radians(),
                                max_theta_step, kMaxIterations);
    min_s = std::min(min_s, s);
  }

  // Enforce drive wheel acceleration limits.
  const double max_vel_step = dt * limits.maxDriveAcceleration;
  for (int i = 0; i < modules.size(); ++i) {
    if (min_s == 0.0) {
      // No need to carry on.
      break;
    }
    double vx_min_s = (min_s == 1.0)
                          ? desired_vx[i]
                          : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
    double vy_min_s = (min_s == 1.0)
                          ? desired_vy[i]
                          : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
    // Find the max s for this drive wheel. Search on the interval between 0 and
    // min_s, because we already know we can't go faster than that.
    const int kMaxIterations = 10;
    double s =
        min_s * FindDriveMaxS(prev_vx[i], prev_vy[i],
                              std::hypot(prev_vx[i], prev_vy[i]), vx_min_s,
                              vy_min_s, std::hypot(vx_min_s, vy_min_s),
                              max_vel_step, kMaxIterations);
    min_s = std::min(min_s, s);
  }

  frc::kinematics::ChassisSpeeds retSpeeds(
      prevSetpoint.chassisSpeeds.vxMetersPerSecond + min_s * dx,
      prevSetpoint.chassisSpeeds.vyMetersPerSecond + min_s * dy,
      prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
  std::array<frc::kinematics::SwerveModuleState, 4> retStates =
      kinematics.ToSwerveModuleStates(retSpeeds);
  for (int i = 0; i < modules.size(); ++i) {
    if (overrideSteering[i].has_value()) {
      frc::geometry::Rotation2d override = overrideSteering[i].value();
      if (FlipHeading(retStates[i].angle.UnaryMinus() + override)) {
        retStates[i].speedMetersPerSecond *= -1.0;
      }
      retStates[i].angle = override;
    }
    frc::geometry::Rotation2d deltaRotation =
        prevSetpoint.moduleStates[i].angle.UnaryMinus() + retStates[i].angle;
    if (FlipHeading(deltaRotation)) {
      retStates[i].angle = retStates[i].angle + frc::geometry::Rotation2d(M_PI);
      retStates[i].speedMetersPerSecond *= -1.0;
    }
  }
  return SwerveSetpoint{retSpeeds, retStates};
}
