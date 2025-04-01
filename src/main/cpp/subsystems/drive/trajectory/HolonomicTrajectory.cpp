// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "frc/Filesystem.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "org/littletonrobotics/frc2025/Constants.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/trajectory/TrajectoryGenerationHelpers.h"
#include <filesystem>
#include <fstream>
#include <stdexcept>

HolonomicTrajectory::HolonomicTrajectory(const std::string &name) {
  std::filesystem::path deployDirectory =
      org::littletonrobotics::frc2025::Constants::disableHAL
          ? std::filesystem::path("src/main/deploy")
          : frc::filesystem::GetDeployDirectory();
  std::filesystem::path filePath =
      deployDirectory / "trajectories" / (name + ".pathblob");
  std::ifstream fileStream(filePath, std::ios::binary);
  if (!trajectory.ParseFromIstream(&fileStream)) {
    throw std::runtime_error("Could not load trajectory \"" + name + "\"");
  }
}

double HolonomicTrajectory::GetDuration() const {
  if (trajectory.states_size() > 0) {
    return trajectory.states(trajectory.states_size() - 1).time();
  } else {
    return 0.0;
  }
}

frc::Pose2d HolonomicTrajectory::GetStartPose() const {
  VehicleState startState = GetStartState();
  return frc::Pose2d(startState.x(), startState.y(),
                     frc::Rotation2d(startState.theta()));
}

std::vector<frc::Pose2d> HolonomicTrajectory::GetTrajectoryPoses() const {
  std::vector<frc::Pose2d> poses(trajectory.states_size());
  for (int i = 0; i < trajectory.states_size(); i++) {
    VehicleState state = trajectory.states(i).state();
    poses[i] =
        frc::Pose2d(state.x(), state.y(), frc::Rotation2d(state.theta()));
  }
  return poses;
}

std::vector<VehicleState> HolonomicTrajectory::GetStates() const {
  std::vector<VehicleState> states(trajectory.states_size());
  for (int i = 0; i < trajectory.states_size(); i++) {
    states[i] = trajectory.states(i).state();
  }
  return states;
}

VehicleState HolonomicTrajectory::GetStartState() const {
  if (trajectory.states_size() == 0) {
    return VehicleState();
  } else {
    return trajectory.states(0).state();
  }
}

VehicleState HolonomicTrajectory::GetEndState() const {
  if (trajectory.states_size() == 0) {
    return VehicleState();
  } else {
    return trajectory.states(trajectory.states_size() - 1).state();
  }
}

VehicleState HolonomicTrajectory::Sample(double timeSeconds) const {
  const TimestampedVehicleState *before = nullptr;
  const TimestampedVehicleState *after = nullptr;

  for (const auto &state : trajectory.states()) {
    if (state.time() == timeSeconds) {
      return state.state();
    }

    if (state.time() < timeSeconds) {
      before = &state;
    } else {
      after = &state;
      break;
    }
  }

  if (before == nullptr) {
    return trajectory.states(0).state();
  }

  if (after == nullptr) {
    return trajectory.states(trajectory.states_size() - 1).state();
  }

  double s = (timeSeconds - before->time()) / (after->time() - before->time());

  double interpolatedPoseX =
      frc::math::interpolate(before->state().x(), after->state().x(), s);
  double interpolatedPoseY =
      frc::math::interpolate(before->state().y(), after->state().y(), s);
  frc::Rotation2d interpolatedRotation =
      frc::Rotation2d(before->state().theta())
          .Interpolate(frc::Rotation2d(after->state().theta()), s);

  double interpolatedVelocityX =
      frc::math::interpolate(before->state().vx(), after->state().vx(), s);
  double interpolatedVelocityY =
      frc::math::interpolate(before->state().vy(), after->state().vy(), s);
  double interpolatedAngularVelocity = frc::math::interpolate(
      before->state().omega(), after->state().omega(), s);

  std::vector<ModuleForce> moduleForces(4);
  for (int i = 0; i < 4; i++) {
    double interpolatedFx =
        frc::math::interpolate(before->state().module_forces(i).fx(),
                               after->state().module_forces(i).fx(), s);
    double interpolatedFy =
        frc::math::interpolate(before->state().module_forces(i).fy(),
                               after->state().module_forces(i).fy(), s);
    moduleForces[i].set_fx(interpolatedFx);
    moduleForces[i].set_fy(interpolatedFy);
  }

  VehicleState interpolatedState;
  interpolatedState.set_x(interpolatedPoseX);
  interpolatedState.set_y(interpolatedPoseY);
  interpolatedState.set_theta(interpolatedRotation.Radians());
  interpolatedState.set_vx(interpolatedVelocityX);
  interpolatedState.set_vy(interpolatedVelocityY);
  interpolatedState.set_omega(interpolatedAngularVelocity);
  for (const auto &force : moduleForces) {
    *interpolatedState.add_module_forces() = force;
  }
  return interpolatedState;
}