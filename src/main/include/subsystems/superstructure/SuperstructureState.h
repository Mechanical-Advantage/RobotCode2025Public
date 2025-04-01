// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/geometry/Rotation2d.h>

#include "subsystems/superstructure/SuperstructurePose.h"
#include "subsystems/superstructure/SuperstructureStateData.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"

enum class SuperstructureState {
  START,
  AUTO_START,
  CHARACTERIZATION,
  STOW,
  INTAKE,
  GOODBYE_CORAL,
  L1_CORAL,
  L2_CORAL,
  L3_CORAL,
  L4_CORAL,
  GOODBYE_CORAL_EJECT,
  L1_CORAL_EJECT,
  L2_CORAL_EJECT,
  L3_CORAL_EJECT,
  L4_CORAL_EJECT,
  ALGAE_L2_INTAKE,
  ALGAE_L3_INTAKE,
  PRE_THROWN,
  THROWN,
  ALGAE_STOW,
  PRE_TOSS,
  TOSS,
  ALGAE_STOW_INTAKE,
  PROCESSED
};

struct SuperstructureStateData {
  SuperstructurePose pose;
  std::optional<double> tunnelVolts;
  std::optional<Dispenser::GripperGoal> gripperGoal;
  std::optional<Height> height;

  enum class Height { FIRST_STAGE, SECOND_STAGE };

  struct Builder {
    SuperstructurePose pose;
    std::optional<double> tunnelVolts;
    std::optional<Dispenser::GripperGoal> gripperGoal;
    std::optional<Height> height;

    Builder &Pose(const SuperstructurePose &pose);
    Builder &TunnelVolts(double tunnelVolts);
    Builder &GripperGoal(Dispenser::GripperGoal gripperGoal);
    Builder &Height(Height height);
    SuperstructureStateData Build();
  };

  static Builder BuilderCreate();

  Builder ToBuilder() const;
};

SuperstructureStateData GetValue(SuperstructureState state);