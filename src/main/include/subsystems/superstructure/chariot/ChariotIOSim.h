// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "org/littletonrobotics/frc2025/subsystems/superstructure/SuperstructureConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/superstructure/chariot/ChariotIO.h"
#include "org/littletonrobotics/frc2025/util/gslam/GenericSlamElevatorIOSim.h"

class ChariotIOSim : public GenericSlamElevatorIOSim, public ChariotIO {
public:
  ChariotIOSim();
  ~ChariotIOSim() override = default;
};