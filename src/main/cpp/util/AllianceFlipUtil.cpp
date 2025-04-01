// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/AllianceFlipUtil.h"
#include "Constants.h"
#include "FieldConstants.h"

namespace util {

double AllianceFlipUtil::ApplyX(double x) {
  return ShouldFlip() ? FieldConstants::fieldLength - x : x;
}

double AllianceFlipUtil::ApplyY(double y) {
  return ShouldFlip() ? FieldConstants::fieldWidth - y : y;
}

frc::geometry::Translation2d
AllianceFlipUtil::Apply(frc::geometry::Translation2d translation) {
  return frc::geometry::Translation2d(ApplyX(translation.X()),
                                      ApplyY(translation.Y()));
}

frc::geometry::Rotation2d
AllianceFlipUtil::Apply(frc::geometry::Rotation2d rotation) {
  return ShouldFlip() ? rotation + frc::geometry::Rotation2d(M_PI) : rotation;
}

frc::geometry::Pose2d AllianceFlipUtil::Apply(frc::geometry::Pose2d pose) {
  return ShouldFlip() ? frc::geometry::Pose2d(Apply(pose.Translation()),
                                              Apply(pose.Rotation()))
                      : pose;
}

frc::geometry::Translation3d
AllianceFlipUtil::Apply(frc::geometry::Translation3d translation) {
  return frc::geometry::Translation3d(ApplyX(translation.X()),
                                      ApplyY(translation.Y()), translation.Z());
}

frc::geometry::Rotation3d
AllianceFlipUtil::Apply(frc::geometry::Rotation3d rotation) {
  return ShouldFlip() ? rotation + frc::geometry::Rotation3d(0.0, 0.0, M_PI)
                      : rotation;
}

frc::geometry::Pose3d AllianceFlipUtil::Apply(frc::geometry::Pose3d pose) {
  return frc::geometry::Pose3d(Apply(pose.Translation()),
                               Apply(pose.Rotation()));
}

VehicleState AllianceFlipUtil::Apply(const VehicleState &state) {
  if (ShouldFlip()) {
    VehicleState newState;
    newState.set_x(ApplyX(state.x()));
    newState.set_y(ApplyY(state.y()));
    newState.set_theta(
        Apply(frc::geometry::Rotation2d(state.theta())).Radians());
    newState.set_vx(-state.vx());
    newState.set_vy(-state.vy());
    newState.set_omega(state.omega());

    for (const auto &force : state.module_forces()) {
      ModuleForce *newForce = newState.add_module_forces();
      newForce->set_fx(-force.fx());
      newForce->set_fy(-force.fy());
    }
    return newState;
  } else {
    return state;
  }
}

bool AllianceFlipUtil::ShouldFlip() {
  return !Constants::disableHAL &&
         frc::DriverStation::GetAlliance().has_value() &&
         frc::DriverStation::GetAlliance().value() ==
             frc::DriverStation::Alliance::kRed;
}

} // namespace util