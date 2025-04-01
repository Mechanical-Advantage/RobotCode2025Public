// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <list>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/units/units.h>
#include <frc2/command/Command.h>

#include "subsystems/drive/Drive.h"

namespace org::littletonrobotics::frc2025::commands {

class DriveCommands {
public:
  static constexpr double DEADBAND = 0.1;

  static frc::Translation2d GetLinearVelocityFromJoysticks(double x, double y);

  static double GetOmegaFromJoysticks(double driverOmega);

  /**
   * Field or robot relative drive command using two joysticks (controlling
   * linear and angular velocities).
   */
  static frc2::command::Command *
  JoystickDrive(Drive &drive, std::function<double()> xSupplier,
                std::function<double()> ySupplier,
                std::function<double()> omegaSupplier,
                std::function<bool()> robotRelative);

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control. Possible use cases include snapping to an angle, aiming at
   * a vision target, or controlling absolute rotation with a joystick.
   */
  static frc2::command::Command *
  JoystickDriveAtAngle(Drive &drive, std::function<double()> xSupplier,
                       std::function<double()> ySupplier,
                       std::function<frc::Rotation2d()> rotationSupplier);

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  static frc2::command::Command *FeedforwardCharacterization(Drive &drive);

  /** Measures the robot's wheel radius by spinning in a circle. */
  static frc2::command::Command *WheelRadiusCharacterization(Drive &drive);

private:
  static constexpr double ANGLE_KP = 5.0;
  static constexpr double ANGLE_KD = 0.4;
  static constexpr double ANGLE_MAX_VELOCITY = 8.0;
  static constexpr double ANGLE_MAX_ACCELERATION = 20.0;
  static constexpr double FF_START_DELAY = 2.0;             // Secs
  static constexpr double FF_RAMP_RATE = 0.1;               // Volts/Sec
  static constexpr double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  static constexpr double WHEEL_RADIUS_RAMP_RATE = 0.05;    // Rad/Sec^2

  struct WheelRadiusCharacterizationState {
    std::array<double, 4> positions = {0.0, 0.0, 0.0, 0.0};
    frc::Rotation2d lastAngle = frc::Rotation2d::Degrees(0);
    double gyroDelta = 0.0;
  };

  DriveCommands() = delete;
};

} // namespace org::littletonrobotics::frc2025::commands