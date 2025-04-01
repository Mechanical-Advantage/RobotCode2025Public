// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/DriveCommands.h"

#include <cmath>
#include <functional>
#include <list>
#include <sstream>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/units/units.h>
#include <frc2/command/Commands.h>

#include "RobotState.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "util/Util.h"
#include "wpi/math/MathUtil.h"
#include "wpi/math/numbers.h"

using namespace org::littletonrobotics::frc2025::commands;

frc::Translation2d DriveCommands::GetLinearVelocityFromJoysticks(double x,
                                                                 double y) {
  // Apply deadband
  double linearMagnitude =
      wpi::math::numbers::hypot(wpi::math::numbers::apply_deadband(
          wpi::math::numbers::hypot(x, y), DriveCommands::DEADBAND));
  frc::Rotation2d linearDirection(std::atan2(y, x));

  // Square magnitude for more precise control
  linearMagnitude = linearMagnitude * linearMagnitude;

  // Return new linear velocity
  return frc::Pose2d(frc::Translation2d{}, linearDirection)
      .TransformBy(
          frc::Transform2d(linearMagnitude, 0.0, frc::Rotation2d::Degrees(0)))
      .Translation();
}

double DriveCommands::GetOmegaFromJoysticks(double driverOmega) {
  double omega =
      wpi::math::numbers::apply_deadband(driverOmega, DriveCommands::DEADBAND);
  return omega * omega * util::Sign(omega);
}

/**
 * Field or robot relative drive command using two joysticks (controlling linear
 * and angular velocities).
 */
frc2::command::Command *
DriveCommands::JoystickDrive(Drive &drive, std::function<double()> xSupplier,
                             std::function<double()> ySupplier,
                             std::function<double()> omegaSupplier,
                             std::function<bool()> robotRelative) {
  return frc2::command::Commands::Run(
      [&]() {
        // Get linear velocity
        frc::Translation2d linearVelocity =
            GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

        // Calculate angular velocity
        double omega = GetOmegaFromJoysticks(omegaSupplier());

        frc::kinematics::ChassisSpeeds speeds(
            linearVelocity.X() * drive.GetMaxLinearSpeedMetersPerSec(),
            linearVelocity.Y() * drive.GetMaxLinearSpeedMetersPerSec(),
            omega * drive.GetMaxAngularSpeedRadPerSec());

        drive.RunVelocity(
            robotRelative()
                ? speeds
                : frc::kinematics::ChassisSpeeds::FromFieldRelativeSpeeds(
                      speeds,
                      frc::DriverStation::GetAlliance().has_value() &&
                              frc::DriverStation::GetAlliance().value() ==
                                  frc::DriverStation::Alliance::kRed
                          ? RobotState::GetInstance().GetRotation() +
                                frc::Rotation2d::Degrees(180)
                          : RobotState::GetInstance().GetRotation()));
      },
      {&drive});
}

/**
 * Field relative drive command using joystick for linear control and PID for
 * angular control. Possible use cases include snapping to an angle, aiming at
 * a vision target, or controlling absolute rotation with a joystick.
 */
frc2::command::Command *DriveCommands::JoystickDriveAtAngle(
    Drive &drive, std::function<double()> xSupplier,
    std::function<double()> ySupplier,
    std::function<frc::Rotation2d()> rotationSupplier) {
  // Create PID controller
  frc::ProfiledPIDController angleController(
      DriveCommands::ANGLE_KP, 0.0, DriveCommands::ANGLE_KD,
      frc::TrapezoidProfile<units::radian>::Constraints(
          DriveCommands::ANGLE_MAX_VELOCITY,
          DriveCommands::ANGLE_MAX_ACCELERATION));
  angleController.EnableContinuousInput(-wpi::math::numbers::pi,
                                        wpi::math::numbers::pi);

  // Construct command
  return frc2::command::Commands::Run(
             [&]() {
               // Get linear velocity
               frc::Translation2d linearVelocity =
                   GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

               // Calculate angular speed
               frc::Rotation2d rotation =
                   RobotState::GetInstance().GetRotation();
               double omega = angleController.Calculate(
                   rotation.Radians(), rotationSupplier().Radians());

               // Convert to field relative speeds & send command
               frc::kinematics::ChassisSpeeds speeds(
                   linearVelocity.X() * drive.GetMaxLinearSpeedMetersPerSec(),
                   linearVelocity.Y() * drive.GetMaxLinearSpeedMetersPerSec(),
                   omega);
               bool isFlipped = frc::DriverStation::GetAlliance().has_value() &&
                                frc::DriverStation::GetAlliance().value() ==
                                    frc::DriverStation::Alliance::kRed;
               drive.RunVelocity(
                   frc::kinematics::ChassisSpeeds::FromFieldRelativeSpeeds(
                       speeds, isFlipped
                                   ? rotation + frc::Rotation2d::Degrees(180)
                                   : rotation));
             },
             {&drive})

      // Reset PID controller when command starts
      .BeforeStarting([&]() {
        angleController.Reset(
            RobotState::GetInstance().GetRotation().Radians());
      });
}

/**
 * Measures the velocity feedforward constants for the drive motors.
 *
 * <p>This command should only be used in voltage control mode.
 */
frc2::command::Command *
DriveCommands::FeedforwardCharacterization(Drive &drive) {
  std::list<double> velocitySamples;
  std::list<double> voltageSamples;
  frc::Timer timer;

  return frc2::command::Commands::Sequence(
      // Reset data
      frc2::command::Commands::RunOnce([&]() {
        velocitySamples.clear();
        voltageSamples.clear();
      }),

      // Allow modules to orient
      frc2::command::Commands::Run([&]() { drive.RunCharacterization(0.0); },
                                   {&drive})
          .WithTimeout(DriveCommands::FF_START_DELAY),

      // Start timer
      frc2::command::Commands::RunOnce([&]() { timer.Restart(); }),

      // Accelerate and gather data
      frc2::command::Commands::Run(
          [&]() {
            double voltage = timer.Get() * DriveCommands::FF_RAMP_RATE;
            drive.RunCharacterization(voltage);
            velocitySamples.push_back(drive.GetFFCharacterizationVelocity());
            voltageSamples.push_back(voltage);
          },
          {&drive})

          // When cancelled, calculate and print results
          .FinallyDo([&](bool interrupted) {
            int n = velocitySamples.size();
            double sumX = 0.0;
            double sumY = 0.0;
            double sumXY = 0.0;
            double sumX2 = 0.0;
            for (int i = 0; i < n; i++) {
              auto itX = std::next(velocitySamples.begin(), i);
              auto itY = std::next(voltageSamples.begin(), i);
              sumX += *itX;
              sumY += *itY;
              sumXY += *itX * *itY;
              sumX2 += *itX * *itX;
            }
            double kS =
                (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

            std::stringstream ss;
            ss.precision(5);
            ss << std::fixed;
            ss << "********** Drive FF Characterization Results **********"
               << std::endl;
            ss << "\tkS: " << kS << std::endl;
            ss << "\tkV: " << kV << std::endl;
            std::cout << ss.str();
          }));
}

/** Measures the robot's wheel radius by spinning in a circle. */
frc2::command::Command *
DriveCommands::WheelRadiusCharacterization(Drive &drive) {
  frc::SlewRateLimiter<units::radians_per_second> limiter(
      DriveCommands::WHEEL_RADIUS_RAMP_RATE);
  WheelRadiusCharacterizationState state;

  return frc2::command::Commands::Parallel(
      // Drive control sequence
      frc2::command::Commands::Sequence(
          // Reset acceleration limiter
          frc2::command::Commands::RunOnce(
              [&]() { limiter.Reset(0.0_rad_per_s); }),

          // Turn in place, accelerating up to full speed
          frc2::command::Commands::Run(
              [&]() {
                units::radians_per_second speed =
                    limiter.Calculate(DriveCommands::WHEEL_RADIUS_MAX_VELOCITY);
                drive.RunVelocity(frc::kinematics::ChassisSpeeds(
                    0.0_mps, 0.0_mps, speed.value()));
              },
              {&drive})),

      // Measurement sequence
      frc2::command::Commands::Sequence(
          // Wait for modules to fully orient before starting measurement
          frc2::command::Commands::Wait(1.0_s),

          // Record starting measurement
          frc2::command::Commands::RunOnce([&]() {
            state.positions = drive.GetWheelRadiusCharacterizationPositions();
            state.lastAngle = drive.GetGyroRotation();
            state.gyroDelta = 0.0;
          }),

          // Update gyro delta
          frc2::command::Commands::Run([&]() {
            frc::Rotation2d rotation = drive.GetGyroRotation();
            state.gyroDelta +=
                std::abs((rotation - state.lastAngle).Radians().value());
            state.lastAngle = rotation;

            std::array<double, 4> positions =
                drive.GetWheelRadiusCharacterizationPositions();
            double wheelDelta = 0.0;
            for (int i = 0; i < 4; i++) {
              wheelDelta += std::abs(positions[i] - state.positions[i]) / 4.0;
            }
            double wheelRadius =
                (state.gyroDelta * DriveConstants::driveBaseRadius) /
                wheelDelta;

            wpi::log::DataLogEntry(wpi::log::DoubleLog("Drive/WheelDelta"))
                .Append(wheelDelta);
            wpi::log::DataLogEntry(wpi::log::DoubleLog("Drive/WheelRadius"))
                .Append(wheelRadius);
          })

              // When cancelled, calculate and print results
              .FinallyDo([&](bool interrupted) {
                std::array<double, 4> positions =
                    drive.GetWheelRadiusCharacterizationPositions();
                double wheelDelta = 0.0;
                for (int i = 0; i < 4; i++) {
                  wheelDelta +=
                      std::abs(positions[i] - state.positions[i]) / 4.0;
                }
                double wheelRadius =
                    (state.gyroDelta * DriveConstants::driveBaseRadius) /
                    wheelDelta;

                std::stringstream ss;
                ss.precision(16);
                ss << std::fixed;
                ss << "********** Wheel Radius Characterization Results "
                      "**********"
                   << std::endl;
                ss << "\tWheel Delta: " << wheelDelta << " radians"
                   << std::endl;
                ss << "\tGyro Delta: " << state.gyroDelta << " radians"
                   << std::endl;
                ss << "\tWheel Radius: " << wheelRadius << " meters, "
                   << frc::units::meter_to_inch(wheelRadius) << " inches"
                   << std::endl;
                std::cout << ss.str();
              })));
}
