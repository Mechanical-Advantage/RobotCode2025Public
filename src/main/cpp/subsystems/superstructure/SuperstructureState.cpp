// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/SuperstructureState.h"

#include <optional>

#include <frc/geometry/Rotation2d.h>

#include "subsystems/superstructure/SuperstructurePose.h"
#include "subsystems/superstructure/SuperstructureStateData.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"

SuperstructureStateData GetValue(SuperstructureState state) {
  switch (state) {
  case SuperstructureState::START:
    return SuperstructureStateData::BuilderCreate().Build();
  case SuperstructureState::AUTO_START:
    return SuperstructureStateData::BuilderCreate().Build();
  case SuperstructureState::CHARACTERIZATION:
    return SuperstructureStateData::BuilderCreate().Build();
  case SuperstructureState::STOW:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::STOW))
        .Build();
  case SuperstructureState::INTAKE:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::INTAKE))
        .TunnelVolts(Dispenser::tunnelIntakeVolts)
        .Build();
  case SuperstructureState::GOODBYE_CORAL:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::GOODBYE_CORAL))
        .Build();
  case SuperstructureState::L1_CORAL:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::L1))
        .Build();
  case SuperstructureState::L2_CORAL:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::L2))
        .Height(SuperstructureStateData::Height::FIRST_STAGE)
        .Build();
  case SuperstructureState::L3_CORAL:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::L3))
        .Height(SuperstructureStateData::Height::FIRST_STAGE)
        .Build();
  case SuperstructureState::L4_CORAL:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::L4))
        .Height(SuperstructureStateData::Height::SECOND_STAGE)
        .Build();
  case SuperstructureState::GOODBYE_CORAL_EJECT:
    return GetValue(SuperstructureState::GOODBYE_CORAL)
        .ToBuilder()
        .TunnelVolts(Dispenser::tunnelDispenseVolts)
        .Build();
  case SuperstructureState::L1_CORAL_EJECT:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::L1_EJECT))
        .TunnelVolts(Dispenser::tunnelL1DispenseVolts)
        .GripperGoal(Dispenser::GripperGoal::L1_EJECT)
        .Build();
  case SuperstructureState::L2_CORAL_EJECT:
    return GetValue(SuperstructureState::L2_CORAL)
        .ToBuilder()
        .TunnelVolts(Dispenser::tunnelDispenseVolts)
        .GripperGoal(Dispenser::GripperGoal::L1_EJECT)
        .Build();
  case SuperstructureState::L3_CORAL_EJECT:
    return GetValue(SuperstructureState::L3_CORAL)
        .ToBuilder()
        .TunnelVolts(Dispenser::tunnelDispenseVolts)
        .Build();
  case SuperstructureState::L4_CORAL_EJECT:
    return GetValue(SuperstructureState::L4_CORAL)
        .ToBuilder()
        .TunnelVolts(Dispenser::tunnelDispenseVolts)
        .GripperGoal(Dispenser::GripperGoal::EJECT)
        .Build();
  case SuperstructureState::ALGAE_L2_INTAKE:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::ALGAE_L2_INTAKE))
        .GripperGoal(Dispenser::GripperGoal::GRIP)
        .Height(SuperstructureStateData::Height::FIRST_STAGE)
        .Build();
  case SuperstructureState::ALGAE_L3_INTAKE:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::ALGAE_L3_INTAKE))
        .GripperGoal(Dispenser::GripperGoal::GRIP)
        .Height(SuperstructureStateData::Height::FIRST_STAGE)
        .Build();
  case SuperstructureState::PRE_THROWN:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::PRE_THROW))
        .GripperGoal(Dispenser::GripperGoal::GRIP)
        .Height(SuperstructureStateData::Height::SECOND_STAGE)
        .Build();
  case SuperstructureState::THROWN:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::THROW))
        .GripperGoal(Dispenser::GripperGoal::EJECT)
        .Build();
  case SuperstructureState::ALGAE_STOW:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::ALGAE_STOW))
        .GripperGoal(Dispenser::GripperGoal::GRIP)
        .Build();
  case SuperstructureState::PRE_TOSS:
    return SuperstructureStateData::BuilderCreate()
        .Pose(SuperstructurePose(SuperstructurePose::Preset::ALGAE_STOW)
                  .elevatorHeight,
              []() { return frc::Rotation2d::Degrees(30.0); })
        .GripperGoal(Dispenser::GripperGoal::GRIP)
        .Build();
  case SuperstructureState::TOSS:
    return GetValue(SuperstructureState::PRE_TOSS)
        .ToBuilder()
        .GripperGoal(Dispenser::GripperGoal::EJECT)
        .Build();
  case SuperstructureState::ALGAE_STOW_INTAKE:
    return GetValue(SuperstructureState::ALGAE_STOW).ToBuilder().Build();
  case SuperstructureState::PROCESSED:
    return GetValue(SuperstructureState::ALGAE_STOW)
        .ToBuilder()
        .GripperGoal(Dispenser::GripperGoal::EJECT)
        .Build();
  default:
    return SuperstructureStateData::BuilderCreate().Build();
  }
}