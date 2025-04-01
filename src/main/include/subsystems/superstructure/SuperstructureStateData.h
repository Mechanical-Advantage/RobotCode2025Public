// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <optional>

#include "subsystems/superstructure/SuperstructurePose.h"
#include "subsystems/superstructure/chariot/Chariot.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"

struct SuperstructureStateData {
  SuperstructurePose pose;
  std::optional<std::function<double()>> tunnelVolts;
  Dispenser::GripperGoal gripperGoal;
  std::optional<std::function<double()>> intakeVolts;
  Chariot::Goal chariotGoal;
  Height height;
  bool reversed;

  enum class Height { BOTTOM, FIRST_STAGE, SECOND_STAGE };

  struct Builder {
    SuperstructurePose pose;
    std::optional<std::function<double()>> tunnelVolts;
    Dispenser::GripperGoal gripperGoal;
    std::optional<std::function<double()>> intakeVolts;
    Chariot::Goal chariotGoal;
    Height height;
    bool reversed;

    Builder &Pose(const SuperstructurePose &pose);
    Builder &TunnelVolts(std::function<double()> tunnelVolts);
    Builder &GripperGoal(Dispenser::GripperGoal gripperGoal);
    Builder &IntakeVolts(std::function<double()> intakeVolts);
    Builder &ChariotGoal(Chariot::Goal chariotGoal);
    Builder &Height(Height height);
    Builder &Reversed(bool reversed);
    SuperstructureStateData Build();
  };

  static Builder BuilderCreate();

  Builder ToBuilder() const;
};