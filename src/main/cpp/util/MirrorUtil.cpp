// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/MirrorUtil.h"

namespace util {

std::function<bool()> MirrorUtil::mirror;

void MirrorUtil::SetMirror(std::function<bool()> mirrorSupplier) {
  mirror = mirrorSupplier;
}

FieldConstants::CoralObjective
MirrorUtil::Apply(FieldConstants::CoralObjective coralObjective) {
  if (!mirror())
    return coralObjective;
  int shiftedBranchId = coralObjective.branchId - 1;
  if (shiftedBranchId == -1) {
    shiftedBranchId = 11;
  }
  int flippedBranchId = 11 - shiftedBranchId;
  flippedBranchId = (++flippedBranchId == 12) ? 0 : flippedBranchId;
  return FieldConstants::CoralObjective(flippedBranchId,
                                        coralObjective.reefLevel);
}

frc::geometry::Pose2d MirrorUtil::Apply(frc::geometry::Pose2d pose) {
  if (!mirror())
    return pose;
  return frc::geometry::Pose2d(
      pose.X(), FieldConstants::fieldWidth - pose.Y(),
      frc::geometry::Rotation2d(pose.Rotation().Cos(), -pose.Rotation().Sin()));
}

VehicleState MirrorUtil::Apply(const VehicleState &state) {
  if (!mirror())
    return state;
  frc::geometry::Pose2d pose =
      Apply(frc::geometry::Pose2d(state.x(), state.y(), state.theta()));
  VehicleState newState;
  newState.set_x(pose.X());
  newState.set_y(pose.Y());
  newState.set_theta(pose.Rotation().Radians());
  newState.set_vx(state.vx());
  newState.set_vy(-state.vy());
  newState.set_omega(-state.omega());

  for (const auto &force : state.module_forces()) {
    ModuleForce *newForce = newState.add_module_forces();
    newForce->set_fx(force.fx());
    newForce->set_fy(-force.fy());
  }
  return newState;
}

} // namespace util