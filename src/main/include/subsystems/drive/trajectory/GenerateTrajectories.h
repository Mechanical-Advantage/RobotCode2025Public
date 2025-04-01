// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "org/littletonrobotics/vehicletrajectoryservice/VehicleTrajectoryService.pb.h"
#include <set>
#include <string>
#include <vector>

class GenerateTrajectories {
public:
  static std::string GetHashCode(const VehicleModel &model,
                                 const std::vector<PathSegment> &segments);
};